# Mechatronics Projects & Simulations Portfolio

This repository serves as a centralized portfolio of source code, simulation scripts, and technical project documentation developed during my B.Tech in Mechatronics Engineering at Manipal Institute of Technology.

## 1. Project Documentation & Architecture Reports

### Automated Carwash System (PLC)
* **File:** `PLC IA project report (1).pdf`
* **Description:** Documentation detailing the design and ladder logic programming of an automated carwash system using a BOSCH REXROTH INDRAL20 PLC. Features include sequential operation management and a 75% water reclamation system.

### Autonomous Plant Watering Robot
* **File:** `Final_Complete_Autonomous_Plant_Watering_Robot_Report[2].pdf` 
* **Description:** Project report covering the integration of an ESP32 microcontroller, IR sensors, and a relay-switched 12V DC water pump to create an autonomous, line-following irrigation robot.

### Gesture-Controlled Vehicle Architecture
* **File:** `Vashisth_Research_ECS_14_10_01_2026.pdf`
* **Description:** A technical proposal and system architecture design for a tele-operated vehicle utilizing dual ESP32-S3 microcontrollers, ESP-NOW protocol, and capacitive touch safety interlocks.

---

## 2. Advanced Communication Simulations (Code)

### Underwater Network Simulation with Emergency Prioritization (MATLAB)
* **File:** `acoustic_communication_model.m`
* **Description:** 
  * This MATLAB script simulates an underwater acoustic network, evaluating average throughput, transmission delay, and energy consumption across a varying number of ordinary nodes. 
  * It implements a dynamic clustering algorithm where the probability of a node becoming a cluster head is influenced by its remaining energy and its spatial weight relative to the coast. 
  * The code features a scheduling mechanism based on an "SPN score" to calculate transmission times and avoid interference between nodes. 
  * Furthermore, it includes a traffic handling system that triggers priority flags and prioritizes packets when simulated sensor values exceed a designated pressure threshold.

### Custom MAC Protocol for UWSNs (ns-3 / C++)
* **File:** `final.cc`
* **Description:** 
  * This C++ script utilizes the ns-3 network simulator and its Underwater Acoustic Network (UAN) module to model acoustic communication topologies. 
  * It defines a custom MAC protocol class named `CssMac` that manages both priority queues and standard queues for packet transmission. 
  * The code calculates network topology, establishes cluster heads, and executes spatial load balancing by assigning transmission times based on acoustic distance and interference sets. 
  * Additionally, it incorporates an emergency mode triggered by a simulated "P-Wave interrupt," which dynamically tags packets with elevated sending priorities.

### Collision-Free Scheduling Simulation (MATLAB)
* **File:** `finalbaselinecode.m`
* **Description:** 
  * This MATLAB script models an underwater acoustic sensor network with a strong focus on clustering and collision-free transmission scheduling. 
  * It dynamically assigns cluster heads based on node energy levels and calculates clustering coefficients for the network members. 
  * The simulation utilizes a distinct spatial priority scheduling logic to determine conflict-free transmission windows, explicitly calculating and avoiding "forbidden" time slots to prevent data collisions. 
  * The script outputs plots detailing the network's average throughput, delay, and energy consumption calculated over multiple iterations.

---

## About the Author
**Vashisth Chandak**
* B.Tech in Mechatronics Engineering, MIT Manipal
