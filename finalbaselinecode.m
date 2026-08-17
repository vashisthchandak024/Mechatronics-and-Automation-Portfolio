ordinary_nodes_array = [24, 30, 36, 42, 47]; 
num_iterations = 20; 
grid_size = 5000;
comm_radius = 3000;
sound_speed = 1500;
data_rate = 1000;
data_packet_size = 500 * 8; 
control_packet_size = 20 * 8; 
T_delay_data = data_packet_size / data_rate;
T_delay_ctrl = control_packet_size / data_rate;
theta = 0.05;
network_load = 0.5; 
target_packets = 1000; 

calc_energy = @(d, bits) bits * ((2 * 50e-9) + (1e-6 * (max(d/1000, 0.001)^1.5) * (10^(((0.11 * 25^2 / (1 + 25^2)) + (44 * 25^2 / (4100 + 25^2)) + (2.75e-4 * 25^2) + 0.003) * max(d/1000, 0.001) / 10))));
avg_throughput = zeros(length(ordinary_nodes_array), 1);
avg_delay = zeros(length(ordinary_nodes_array), 1);
avg_energy = zeros(length(ordinary_nodes_array), 1);

for n_idx = 1:length(ordinary_nodes_array)
    num_ordinary = ordinary_nodes_array(n_idx);
    num_nodes = num_ordinary + 1; 
    lambda_per_node = network_load / num_ordinary;
    
    sum_throughput = 0; sum_delay = 0; sum_energy = 0;
    
    for iter = 1:num_iterations
        node_positions = rand(num_nodes, 3) * grid_size;
        node_positions(1, :) = [grid_size/2, grid_size/2, 0]; 
        node_energy = 50 + 50 * rand(num_nodes, 1);
        node_energy(1) = Inf; 
        
        dist_matrix = zeros(num_nodes, num_nodes);
        for i = 1:num_nodes
            for j = 1:num_nodes
                if i ~= j, dist_matrix(i,j) = norm(node_positions(i,:) - node_positions(j,:)); end
            end
        end
        prop_delay = dist_matrix / sound_speed;
        
        node_status = zeros(num_nodes, 1); 
        primary_cluster_head = zeros(num_nodes, 1);
        node_status(1) = 1; primary_cluster_head(1) = 1;
        total_energy_consumed = 0;
        
        while any(node_status == 0)
            pch_claim = zeros(num_nodes, 1);
            for i = 2:num_nodes
                if node_status(i) == 0
                    total_energy_consumed = total_energy_consumed + calc_energy(comm_radius, control_packet_size);
                    if rand() <= (node_energy(i)/max(node_energy)), pch_claim(i) = 1; end
                end
            end
            if sum(pch_claim) == 0
                [~, m] = max(node_energy(node_status==0)); idx = find(node_status==0); pch_claim(idx(m)) = 1;
            end
            
            for i = 2:num_nodes
                if node_status(i) == 0 && pch_claim(i) == 0
                    found_pchs = [];
                    for j = 1:num_nodes
                        if pch_claim(j) == 1 && dist_matrix(i,j) <= comm_radius, found_pchs = [found_pchs, j]; end
                    end
                    if ~isempty(found_pchs)
                        [~, best] = max(node_energy(found_pchs));
                        node_status(i) = 2; primary_cluster_head(i) = found_pchs(best);
                        total_energy_consumed = total_energy_consumed + calc_energy(dist_matrix(i, found_pchs(best)), control_packet_size);
                    end
                end
            end
            for i = 2:num_nodes, if pch_claim(i) == 1, node_status(i) = 1; primary_cluster_head(i) = i; end, end
        end
        
        CH_list = find(node_status == 1); member_list = find(node_status == 2);
        coeff = zeros(num_nodes, 1);
        for i = 1:length(member_list), m = member_list(i); coeff(m) = sum(dist_matrix(m, CH_list) <= comm_radius); end
        
        int_nodes = cell(num_nodes, 1); ST_CG = cell(num_nodes, 1);
        for i = 1:length(member_list)
            m = member_list(i);
            temp_int = [];
            for j = 1:length(member_list)
                other = member_list(j);
                if m ~= other && (primary_cluster_head(m) == primary_cluster_head(other) || dist_matrix(m, primary_cluster_head(other)) <= comm_radius)
                    temp_int = [temp_int, other];
                end
            end
            int_nodes{m} = temp_int;
            for idx = 1:length(int_nodes{m})
                other = int_nodes{m}(idx);
                ST_CG{m} = [ST_CG{m}; other, primary_cluster_head(other), prop_delay(m, primary_cluster_head(other)) - prop_delay(other, primary_cluster_head(other))];
            end
        end
        SPN = zeros(num_nodes, 1);
        for i = 1:length(member_list), m = member_list(i); SPN(m) = (size(ST_CG{m},1)^2)*(size(ST_CG{m},1)-length(int_nodes{m})) + (dist_matrix(m, primary_cluster_head(m))/comm_radius); end
        
        Status_msg = zeros(num_nodes, 1); Status_msg(CH_list) = 1; transmission_time = zeros(num_nodes, 1) - 1;
        for i = 1:length(member_list), if isempty(int_nodes{member_list(i)}), transmission_time(member_list(i)) = 0; Status_msg(member_list(i)) = 1; end, end
        
        unresolved = member_list(Status_msg(member_list) == 0);
        while ~isempty(unresolved)
            progress = false;
            for idx = 1:length(unresolved)
                curr = unresolved(idx);
                can_sch = true;
                for j = 1:length(int_nodes{curr})
                    other = int_nodes{curr}(j);
                    if ((coeff(other) > coeff(curr)) || (coeff(other) == coeff(curr) && SPN(other) > SPN(curr)) || (coeff(other) == coeff(curr) && SPN(other) == SPN(curr) && other < curr)) && Status_msg(other) == 0, can_sch = false; break; end
                end
                if can_sch
                    forbidden = [];
                    for j = 1:length(int_nodes{curr})
                        other = int_nodes{curr}(j);
                        if Status_msg(other)==1 && transmission_time(other)~=-1
                            match_idx = find(ST_CG{curr}(:,1)==other);
                            for k = 1:length(match_idx), W=ST_CG{curr}(match_idx(k),3); forbidden=[forbidden; transmission_time(other)-W-T_delay_data-theta, transmission_time(other)-W+T_delay_data+theta]; end
                        end
                    end
                    if isempty(forbidden), transmission_time(curr) = 0; else, [transmission_time(curr)] = get_earliest(forbidden); end
                    Status_msg(curr) = 1; progress = true;
                end
            end
            if ~progress, break; end
            unresolved = member_list(Status_msg(member_list) == 0);
        end
        
        frame = max(transmission_time) + T_delay_data + max(max(prop_delay));
        
        gen = []; for i=1:length(member_list), if transmission_time(member_list(i))~=-1, arr=cumsum(-log(rand(target_packets,1))/lambda_per_node); gen=[gen; arr, ones(target_packets,1)*member_list(i)]; end, end
        gen = sortrows(gen, 1);
        succ = 0; dly = 0; clock = 0; next_avail = zeros(num_nodes,1);
        for p=1:size(gen,1)
            if succ >= target_packets, break; end
            g = gen(p,1); n = gen(p,2); t_tx = transmission_time(n); ch = primary_cluster_head(n);
            ready = max(g, next_avail(n)); k = ceil((ready - t_tx)/frame); if k<0, k=0; end
            act = k*frame + t_tx; arr = act + T_delay_data + prop_delay(n, ch);
            dly = dly + (arr - g); total_energy_consumed = total_energy_consumed + calc_energy(dist_matrix(n, ch), data_packet_size);
            succ = succ + 1; next_avail(n) = act + frame; clock = max(clock, arr);
        end
        sum_throughput = sum_throughput + ((target_packets*data_packet_size)/clock);
        sum_delay = sum_delay + (dly/target_packets);
        sum_energy = sum_energy + total_energy_consumed;
    end
    avg_throughput(n_idx) = sum_throughput/num_iterations;
    avg_delay(n_idx) = sum_delay/num_iterations;
    avg_energy(n_idx) = sum_energy/num_iterations;
end

figure;
subplot(3,1,1); plot(ordinary_nodes_array, avg_throughput, '-o'); xlabel('Ordinary Nodes'); ylabel('Throughput (bps)'); title('Throughput'); grid on;
subplot(3,1,2); plot(ordinary_nodes_array, avg_delay, '-s'); xlabel('Ordinary Nodes'); ylabel('Delay (s)'); title('Delay'); grid on;
subplot(3,1,3); plot(ordinary_nodes_array, avg_energy, '-^'); xlabel('Ordinary Nodes'); ylabel('Energy (J)'); title('Energy'); grid on;

function [t] = get_earliest(f)
    f = sortrows(f,1); m = f(1,:);
    for i=2:size(f,1), if f(i,1)<=m(end,2), m(end,2)=max(m(end,2), f(i,2)); else, m=[m; f(i,:)]; end, end
    t = 0; for i=1:size(m,1), if t<m(i,1), break; else, t=max(t, m(i,2)); end, end
end