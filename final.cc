#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/uan-module.h"
#include "ns3/stats-module.h"
#include "ns3/mac8-address.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CssMacSimulationMerged");

uint32_t g_priPacketsReceived = 0;
double g_priTotalDelay = 0.0;
uint32_t g_stdPacketsReceived = 0;
double g_stdTotalDelay = 0.0;
uint32_t g_totalBits = 0;

bool g_isEmergencyMode = false;
const double COMMUNICATION_RADIUS = 3000.0;
const double ACOUSTIC_SPEED = 1500.0;
const double NOMINAL_DATA_RATE = 1000.0;

void TriggerPWaveInterrupt() {
    g_isEmergencyMode = true;
}

struct ChProximity {
    uint32_t chId;
    double distance;
};

struct UnderwaterNode {
    uint32_t id;
    Vector position;
    bool isClustered;
    bool isClusterHead;
    uint32_t clusterId;               
    std::vector<ChProximity> rankedChs; 
    uint32_t clusterHeadCoefficient; 
    double sendingPriority;          
    double assignedTransmissionTime; 
    uint32_t numEdges;
    uint32_t numVertices;
};

double GetAcousticDistance(Vector a, Vector b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + 
                     (a.y - b.y) * (a.y - b.y) + 
                     (a.z - b.z) * (a.z - b.z));
}

class CssMacHeader : public Header {
public:
    CssMacHeader() : m_ts(0.0) {}
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::CssMacHeader").SetParent<Header>().SetGroupName("Uan").AddConstructor<CssMacHeader>();
        return tid;
    }
    virtual TypeId GetInstanceTypeId(void) const override { return GetTypeId(); }
    virtual void Serialize(Buffer::Iterator start) const override { start.WriteU64(static_cast<uint64_t>(m_ts * 1000000.0)); }
    virtual uint32_t Deserialize(Buffer::Iterator start) override { m_ts = static_cast<double>(start.ReadU64()) / 1000000.0; return 8; }
    virtual uint32_t GetSerializedSize(void) const override { return 8; }
    virtual void Print(std::ostream &os) const override { os << "ts=" << m_ts; }
    void SetTimestamp(double ts) { m_ts = ts; }
    double GetTimestamp() const { return m_ts; }
private:
    double m_ts;
};

class PriorityTag : public Tag {
public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::PriorityTag").SetParent<Tag>().SetGroupName("Uan").AddConstructor<PriorityTag>();
        return tid;
    }
    virtual TypeId GetInstanceTypeId(void) const override { return GetTypeId(); }
    virtual uint32_t GetSerializedSize(void) const override { return 1; }
    virtual void Serialize(TagBuffer i) const override { i.WriteU8(m_priority ? 1 : 0); }
    virtual void Deserialize(TagBuffer i) override { m_priority = (i.ReadU8() == 1); }
    virtual void Print(std::ostream &os) const override { os << "Priority=" << (m_priority ? "High" : "Normal"); }
    void SetPriority(bool priority) { m_priority = priority; }
    bool GetPriority(void) const { return m_priority; }
private:
    bool m_priority = false;
};

class CssMac : public UanMac {
public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::CssMac").SetParent<UanMac>().SetGroupName("Uan").AddConstructor<CssMac>();
        return tid;
    }

    CssMac() : m_isBusy(false) { m_randVar = CreateObject<UniformRandomVariable>(); }
    virtual ~CssMac() {}

    virtual void SetAddress(Mac8Address addr) override { m_address = addr; }
    virtual Address GetAddress(void) override { return m_address; }

    virtual bool Enqueue(Ptr<Packet> pkt, uint16_t protocolNumber, const Address &dest) override {
        CssMacHeader tsHeader;
        tsHeader.SetTimestamp(Simulator::Now().GetSeconds());
        pkt->AddHeader(tsHeader);

        UanHeaderCommon header;
        header.SetDest(Mac8Address::ConvertFrom(dest));
        header.SetSrc(m_address);
        header.SetType(0); 
        pkt->AddHeader(header);
        
        if (g_isEmergencyMode || m_randVar->GetValue() < 0.20) {
            PriorityTag simTag;
            simTag.SetPriority(true);
            pkt->AddPacketTag(simTag);
        }

        PriorityTag priorityTag;
        bool isPriority = false;
        if (pkt->PeekPacketTag(priorityTag)) {
            isPriority = priorityTag.GetPriority();
        }

        if (isPriority) m_priorityQueue.push_back(pkt);
        else m_queue.push_back(pkt);
        
        if (!m_isBusy) CalculateSendingTime();
        return true;
    }

    virtual void SetForwardUpCb(Callback<void, Ptr<Packet>, uint16_t, const Mac8Address&> cb) override {
        m_forwardUpCb = cb;
    }

    virtual void AttachPhy(Ptr<UanPhy> phy) override {
        m_phy = phy;
        m_phy->SetReceiveOkCallback(MakeCallback(&CssMac::Receive, this));
    }

    void Receive(Ptr<Packet> pkt, double snr, UanTxMode mode) {
        UanHeaderCommon header;
        pkt->RemoveHeader(header);
        CssMacHeader tsHeader;
        pkt->RemoveHeader(tsHeader);

        if (header.GetDest() == m_address || header.GetDest() == Mac8Address(255)) {
            double delay = Simulator::Now().GetSeconds() - tsHeader.GetTimestamp();
            PriorityTag priorityTag;
            bool isPriority = false;
            if (pkt->PeekPacketTag(priorityTag)) {
                isPriority = priorityTag.GetPriority();
            }

            if (isPriority) {
                g_priPacketsReceived++;
                g_priTotalDelay += delay;
            } else {
                g_stdPacketsReceived++;
                g_stdTotalDelay += delay;
            }

            g_totalBits += pkt->GetSize() * 8;
            if (!m_forwardUpCb.IsNull()) m_forwardUpCb(pkt, 1, header.GetSrc());
        }
    }

    void FreeMacAndCheckQueue() {
        m_isBusy = false;
        if (!m_priorityQueue.empty() || !m_queue.empty()) CalculateSendingTime();
    }

    void CalculateSendingTime() {
        if (m_isBusy) return;
        
        Ptr<Packet> pkt = nullptr;
        if (!m_priorityQueue.empty()) {
            pkt = m_priorityQueue.front();
            m_priorityQueue.pop_front();
        } else if (!m_queue.empty()) {
            pkt = m_queue.front();
            m_queue.pop_front();
        }

        if (pkt != nullptr) {
            m_isBusy = true;
            double txDuration = (pkt->GetSize() * 8.0) / NOMINAL_DATA_RATE;
            Simulator::Schedule(Seconds(0.1), &UanPhy::SendPacket, m_phy, pkt, 0);
            Simulator::Schedule(Seconds(0.1 + txDuration + 0.05), &CssMac::FreeMacAndCheckQueue, this);
        }
    }

    virtual void Clear() override { m_queue.clear(); m_priorityQueue.clear(); }
    virtual int64_t AssignStreams(int64_t stream) override { return 0; }

private:
    Ptr<UanPhy> m_phy;
    Callback<void, Ptr<Packet>, uint16_t, const Mac8Address&> m_forwardUpCb;
    std::list<Ptr<Packet>> m_priorityQueue; 
    std::list<Ptr<Packet>> m_queue;         
    Mac8Address m_address;
    Ptr<UniformRandomVariable> m_randVar;
    bool m_isBusy;
};

int main(int argc, char *argv[]) {
    uint32_t numNodes = 35; 
    double pWaveTime = 50.0;
    double acousticSimTime = 1000.0; 
    double totalSimTime = pWaveTime + acousticSimTime;
    double rMin = 200.0, alpha = 0.4; 

    CommandLine cmd;
    cmd.Parse(argc, argv);
    Config::SetDefault("ns3::UanPhyPerGenDefault::Threshold", DoubleValue(0.0));

    NodeContainer nodes;
    nodes.Create(numNodes);
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                   "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=5000.0]"),
                                   "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=5000.0]"),
                                   "Z", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=2000.0]"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    std::vector<UnderwaterNode> uwsnTopology(numNodes);
    for (uint32_t i = 0; i < numNodes; ++i) {
        uwsnTopology[i].id = i;
        uwsnTopology[i].position = nodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        uwsnTopology[i].isClustered = false;
        uwsnTopology[i].isClusterHead = false;
        uwsnTopology[i].clusterHeadCoefficient = 0;
        uwsnTopology[i].sendingPriority = 0.0;
        uwsnTopology[i].assignedTransmissionTime = 0.0;
        uwsnTopology[i].numEdges = 0;
        uwsnTopology[i].numVertices = 0;
    }

    for (uint32_t i = 0; i < numNodes; ++i) {
        if (uwsnTopology[i].isClustered) continue;
        uwsnTopology[i].isClusterHead = true;
        uwsnTopology[i].isClustered = true;
        double currentRadius = rMin + (alpha * uwsnTopology[i].position.x);
        for (uint32_t j = 0; j < numNodes; ++j) {
            if (!uwsnTopology[j].isClustered) {
                double dist = GetAcousticDistance(uwsnTopology[i].position, uwsnTopology[j].position);
                if (dist <= currentRadius) {
                    uwsnTopology[j].isClustered = true;
                    uwsnTopology[j].clusterId = i;
                }
            }
        }
    }

    for (uint32_t i = 0; i < numNodes; ++i) {
        if (!uwsnTopology[i].isClusterHead) {
            for (uint32_t j = 0; j < numNodes; ++j) {
                if (uwsnTopology[j].isClusterHead) {
                    double dist = GetAcousticDistance(uwsnTopology[i].position, uwsnTopology[j].position);
                    uwsnTopology[i].rankedChs.push_back({j, dist});
                }
            }
            std::sort(uwsnTopology[i].rankedChs.begin(), uwsnTopology[i].rankedChs.end(), 
                      [](const ChProximity &a, const ChProximity &b) { return a.distance < b.distance; });
            uwsnTopology[i].clusterId = uwsnTopology[i].rankedChs[0].chId;
        }
    }

    for (uint32_t i = 0; i < numNodes; ++i) {
        for (uint32_t j = 0; j < numNodes; ++j) {
            if (i == j) continue;
            double dist = GetAcousticDistance(uwsnTopology[i].position, uwsnTopology[j].position);
            if (uwsnTopology[j].isClusterHead && dist <= COMMUNICATION_RADIUS) {
                uwsnTopology[i].clusterHeadCoefficient++;
            }
        }
        if (uwsnTopology[i].clusterHeadCoefficient == 0) uwsnTopology[i].clusterHeadCoefficient = 1;
    }

    for (uint32_t i = 0; i < numNodes; ++i) {
        if (uwsnTopology[i].isClusterHead) continue;
        std::vector<uint32_t> interferenceSet;
        for (uint32_t j = 0; j < numNodes; ++j) {
            if (i == j || uwsnTopology[j].isClusterHead) continue;
            if (uwsnTopology[i].clusterId == uwsnTopology[j].clusterId) {
                interferenceSet.push_back(j);
            } else {
                double distToOtherCh = GetAcousticDistance(uwsnTopology[i].position, uwsnTopology[uwsnTopology[j].clusterId].position);
                if (distToOtherCh <= COMMUNICATION_RADIUS) interferenceSet.push_back(j);
            }
        }
        uwsnTopology[i].numVertices = interferenceSet.size();
        uwsnTopology[i].numEdges = interferenceSet.size() * uwsnTopology[i].clusterHeadCoefficient;
    }

    std::vector<uint32_t> schedulingOrder;
    for (uint32_t i = 0; i < numNodes; ++i) {
        if (uwsnTopology[i].isClusterHead) continue;
        double chDist = uwsnTopology[i].rankedChs[0].distance;
        double E = uwsnTopology[i].numEdges;
        double V = uwsnTopology[i].numVertices;
        uwsnTopology[i].sendingPriority = (E * E) * (E - V) + (chDist / COMMUNICATION_RADIUS);
        schedulingOrder.push_back(i);
    }

    std::sort(schedulingOrder.begin(), schedulingOrder.end(), [&](uint32_t a, uint32_t b) {
        if (uwsnTopology[a].sendingPriority != uwsnTopology[b].sendingPriority) return uwsnTopology[a].sendingPriority > uwsnTopology[b].sendingPriority;
        return uwsnTopology[a].id < uwsnTopology[b].id;
    });

    uint32_t spatialLoadBalances = 0, temporalQueuesEnforced = 0;
    double maxCycleTime = 0.0, currentTimeSlot = 1.0; 
    std::vector<double> chNextAvailableTime(numNodes, 1.0); 

    for (uint32_t idx : schedulingOrder) {
        bool scheduledSuccessfully = false;
        for (const auto& targetCh : uwsnTopology[idx].rankedChs) {
            if (targetCh.distance <= COMMUNICATION_RADIUS) { 
                if (chNextAvailableTime[targetCh.chId] <= currentTimeSlot) {
                    uwsnTopology[idx].clusterId = targetCh.chId;
                    uwsnTopology[idx].assignedTransmissionTime = currentTimeSlot + (targetCh.distance / ACOUSTIC_SPEED);
                    chNextAvailableTime[targetCh.chId] = uwsnTopology[idx].assignedTransmissionTime + 0.4; 
                    scheduledSuccessfully = true;
                    if (targetCh.chId != uwsnTopology[idx].rankedChs[0].chId) spatialLoadBalances++;
                    break;
                }
            }
        }
        if (!scheduledSuccessfully) {
            double earliestAvailableTime = 999999.0;
            uint32_t bestChId = uwsnTopology[idx].rankedChs[0].chId;
            double bestChDist = uwsnTopology[idx].rankedChs[0].distance;
            bool reachableChFound = false;
            for (const auto& targetCh : uwsnTopology[idx].rankedChs) {
                if (targetCh.distance <= COMMUNICATION_RADIUS) {
                    reachableChFound = true;
                    if (chNextAvailableTime[targetCh.chId] < earliestAvailableTime) {
                        earliestAvailableTime = chNextAvailableTime[targetCh.chId];
                        bestChId = targetCh.chId;
                        bestChDist = targetCh.distance;
                    }
                }
            }
            if (reachableChFound) {
                uwsnTopology[idx].clusterId = bestChId;
                uwsnTopology[idx].assignedTransmissionTime = earliestAvailableTime + (bestChDist / ACOUSTIC_SPEED);
                chNextAvailableTime[bestChId] = uwsnTopology[idx].assignedTransmissionTime + 0.4;
                temporalQueuesEnforced++;
            }
        }
        if (uwsnTopology[idx].assignedTransmissionTime > maxCycleTime) maxCycleTime = uwsnTopology[idx].assignedTransmissionTime;
        currentTimeSlot += 0.2; 
    }

    TypeId cssMacTid = CssMac::GetTypeId();
    UanHelper uan;
    Ptr<UanChannel> channel = CreateObject<UanChannel>();
    Ptr<UanPropModelThorp> prop = CreateObject<UanPropModelThorp>();
    channel->SetPropagationModel(prop);
    uan.SetMac(cssMacTid.GetName());
    NetDeviceContainer devices = uan.Install(nodes, channel);

    PacketSocketHelper packetSocket;
    packetSocket.Install(nodes);
    ApplicationContainer apps;

    for (uint32_t i = 0; i < numNodes; ++i) {
        if (uwsnTopology[i].isClusterHead) {
            PacketSocketAddress anyAddress;
            anyAddress.SetSingleDevice(devices.Get(i)->GetIfIndex());
            anyAddress.SetProtocol(1);
            PacketSinkHelper sink("ns3::PacketSocketFactory", Address(anyAddress));
            ApplicationContainer sinkApp = sink.Install(nodes.Get(i));
            sinkApp.Start(Seconds(0.0));
            sinkApp.Stop(Seconds(totalSimTime));
        } else {
            uint32_t chId = uwsnTopology[i].clusterId;
            PacketSocketAddress sinkAddress;
            sinkAddress.SetSingleDevice(devices.Get(i)->GetIfIndex());
            sinkAddress.SetPhysicalAddress(devices.Get(chId)->GetAddress());
            sinkAddress.SetProtocol(1);
            OnOffHelper onoff("ns3::PacketSocketFactory", Address(sinkAddress));
            onoff.SetAttribute("DataRate", DataRateValue(DataRate(NOMINAL_DATA_RATE)));
            onoff.SetAttribute("PacketSize", UintegerValue(500));
            onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.1]"));
            onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"));
            ApplicationContainer app = onoff.Install(nodes.Get(i));
            apps.Add(app);
            app.Start(Seconds(uwsnTopology[i].assignedTransmissionTime));
            app.Stop(Seconds(totalSimTime));
        }
    }

    Simulator::Schedule(Seconds(pWaveTime), &TriggerPWaveInterrupt);
    Simulator::Stop(Seconds(totalSimTime + 10));
    Simulator::Run();

    uint32_t totalPacketsReceived = g_priPacketsReceived + g_stdPacketsReceived;
    double throughputKbps = (double)g_totalBits / (totalSimTime * 1000.0); 
    double priAvgDelay = (g_priPacketsReceived > 0) ? (g_priTotalDelay / g_priPacketsReceived) : 0.0;
    double stdAvgDelay = (g_stdPacketsReceived > 0) ? (g_stdTotalDelay / g_stdPacketsReceived) : 0.0;

    std::cout << "Spatial Load Balances Executed: " << spatialLoadBalances << std::endl;
    std::cout << "Temporal Queues Enforced: " << temporalQueuesEnforced << std::endl;
    std::cout << "Overall Throughput: " << throughputKbps << " kbps" << std::endl;
    std::cout << "Priority Delay: " << priAvgDelay << " s" << std::endl;
    std::cout << "Standard Delay: " << stdAvgDelay << " s" << std::endl;

    Simulator::Destroy();
    return 0;
}