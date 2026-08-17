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
coast_x = 0;       
alpha = 0.8;

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
        %% Node Deployment
        node_positions = rand(num_nodes, 3) * grid_size;
        node_positions(1, :) = [grid_size/2, grid_size/2, 0]; 
        
        node_energy = 50 + 50 * rand(num_nodes, 1);
        node_energy(1) = Inf;
        
        dist_matrix = zeros(num_nodes, num_nodes);
        for i = 1:num_nodes
            for j = 1:num_nodes
                if i ~= j
                    dist_matrix(i,j) = norm(node_positions(i,:) - node_positions(j,:));
                end
            end
        end
        prop_delay = dist_matrix / sound_speed;
        
        node_status = zeros(num_nodes, 1); 
        primary_cluster_head = zeros(num_nodes, 1);
        node_status(1) = 1; 
        primary_cluster_head(1) = 1;
        total_energy_consumed = 0;
        
        %% Dynamic Clustering
        while any(node_status == 0)
            pch_claim = zeros(num_nodes, 1);
            for i = 2:num_nodes
                if node_status(i) == 0
                    total_energy_consumed = total_energy_consumed + calc_energy(comm_radius, control_packet_size);
                    d_coast = abs(node_positions(i, 1) - coast_x); 
                    spatial_weight = 1 - (alpha * (d_coast / grid_size));
                    prob_CH = (node_energy(i) / max(node_energy)) * spatial_weight;
                    
                    if rand() <= prob_CH 
                        pch_claim(i) = 1;
                    end
                end
            end
            
            if sum(pch_claim) == 0
                [~, m] = max(node_energy(node_status==0));
                idx = find(node_status==0); pch_claim(idx(m)) = 1;
            end
            
            for i = 2:num_nodes
                if node_status(i) == 0 && pch_claim(i) == 0
                    found_pchs = [];
                    for j = 1:num_nodes
                        if pch_claim(j) == 1 && dist_matrix(i,j) <= comm_radius
                            found_pchs = [found_pchs, j];
                        end
                    end
                    if ~isempty(found_pchs)
                        [~, best] = max(node_energy(found_pchs));
                        node_status(i) = 2; 
                        primary_cluster_head(i) = found_pchs(best);
                        total_energy_consumed = total_energy_consumed + calc_energy(dist_matrix(i, found_pchs(best)), control_packet_size);
                    end
                end
            end
            for i = 2:num_nodes
                if pch_claim(i) == 1
                    node_status(i) = 1;
                    primary_cluster_head(i) = i; 
                end
            end
        end
        
        CH_list = find(node_status == 1);
        member_list = find(node_status == 2);
        
        %% Clustering Coefficient 
        available_CHs = cell(num_nodes, 1);
        for i = 1:length(member_list)
            m = member_list(i);
            dist_to_all_CHs = dist_matrix(m, CH_list(CH_list~=1)); 
            
            valid_idx = find(dist_to_all_CHs <= comm_radius); 
            valid_CHs = CH_list(valid_idx);
            valid_dists = dist_to_all_CHs(valid_idx);
            
            if isempty(valid_CHs)
                available_CHs{m} = primary_cluster_head(m);
            else
                [~, sort_idx] = sort(valid_dists);
                available_CHs{m} = valid_CHs(sort_idx);
            end
        end

        coeff = zeros(num_nodes, 1);
        for i = 1:length(member_list)
            m = member_list(i); 
            coeff(m) = sum(dist_matrix(m, CH_list(CH_list~=1)) <= comm_radius);
        end
        
        int_nodes = cell(num_nodes, 1);
        ST_CG = cell(num_nodes, 1);
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
        
        %%  SPN 
        Status_msg = zeros(num_nodes, 1);
        Status_msg(CH_list) = 1; 
        transmission_time = zeros(num_nodes, 1) - 1;
        assigned_CH = primary_cluster_head; 

        SPN_score = zeros(num_nodes, 1);
        for i = 1:length(member_list)
            m = member_list(i);
            V_node = length(int_nodes{m});
            
            if V_node == 0
                SPN_score(m) = 0;
                transmission_time(m) = 0;
                Status_msg(m) = 1;
            else
                E_node = 0;
                for v1 = 1:V_node
                    for v2 = (v1+1):V_node
                        n1 = int_nodes{m}(v1);
                        n2 = int_nodes{m}(v2);
                        if dist_matrix(n1, n2) <= comm_radius
                            E_node = E_node + 1;
                        end
                    end
                end
                SPN_score(m) = E_node / V_node; 
            end
        end

        unresolved_base = member_list(Status_msg(member_list) == 0);
        current_sim_time = 0;
        loop_guard = 0;

        while ~isempty(unresolved_base)
            progress = false;
            loop_guard = loop_guard + 1;
            
            if loop_guard > 1000
                for stuck = 1:length(unresolved_base)
                    curr_stuck = unresolved_base(stuck);
                    assigned_CH(curr_stuck) = primary_cluster_head(curr_stuck);
                    transmission_time(curr_stuck) = current_sim_time;
                    Status_msg(curr_stuck) = 1;
                end
                break;
            end
            
            [~, sort_idx] = sort(SPN_score(unresolved_base), 'descend');
            unresolved = unresolved_base(sort_idx);
            
            for idx = 1:length(unresolved)
                curr = unresolved(idx);
                best_CH = -1;
                min_cost = Inf;
                
                peer_node = get_active_same_coeff_node(curr, coeff, member_list, Status_msg, transmission_time, current_sim_time, T_delay_data);
                PCH = primary_cluster_head(curr);
                
                for ch_idx = 1:length(available_CHs{curr})
                    target_CH = available_CHs{curr}(ch_idx);
                    T_prop = prop_delay(curr, target_CH);
                    
                    if Status_msg(target_CH) == 1 && transmission_time(target_CH) ~= -1
                        T_wait = max(0, transmission_time(target_CH) - current_sim_time);
                    else
                        T_wait = 0;
                    end
                    
                    current_cost = T_wait + T_prop;
                    
                    if peer_node ~= -1 
                        peer_CH = assigned_CH(peer_node);
                        if target_CH == PCH
                            current_cost = Inf; 
                        else
                            vec_sch1 = node_positions(target_CH, 1:2) - node_positions(PCH, 1:2);
                            vec_sch2 = node_positions(peer_CH, 1:2) - node_positions(PCH, 1:2);
                            if dot(vec_sch1, vec_sch2) >= 0 
                                current_cost = Inf; 
                            end
                        end
                    else
                        interfering_node = get_current_transmitter(target_CH, member_list, assigned_CH, Status_msg, transmission_time, current_sim_time, T_delay_data);
                        if interfering_node ~= -1
                            vec1 = node_positions(curr, 1:2) - node_positions(target_CH, 1:2);
                            vec2 = node_positions(interfering_node, 1:2) - node_positions(target_CH, 1:2);
                            if dot(vec1, vec2) >= 0 
                                current_cost = Inf;
                            end
                        end
                    end
                    
                    if current_cost < min_cost
                        min_cost = current_cost;
                        best_CH = target_CH;
                    end
                end
                
                if best_CH ~= -1
                    assigned_CH(curr) = best_CH;
                    transmission_time(curr) = current_sim_time + min_cost;
                    Status_msg(curr) = 1;
                    progress = true;
                end
            end
            
            unresolved_base = member_list(Status_msg(member_list) == 0);
            if ~progress && ~isempty(unresolved_base)
                 current_sim_time = current_sim_time + 0.1;
            end
        end
        
        %% 
        frame = max(transmission_time) + T_delay_data + max(max(prop_delay));
        gen = []; 
        pressure_threshold = 85; 
        
        for i=1:length(member_list)
            if transmission_time(member_list(i))~=-1
                arr = cumsum(-log(rand(target_packets,1))/lambda_per_node); 
                sensor_values = rand(target_packets, 1) * 100;
                priority_flags = double(sensor_values >= pressure_threshold);
                node_ids = ones(target_packets,1) * member_list(i);
                gen = [gen; arr, node_ids, sensor_values, priority_flags];
            end
        end
        
        gen = sortrows(gen, 1);
        
        succ = 0; dly = 0;
        clock = 0; next_avail = zeros(num_nodes,1);
        is_sent = false(size(gen,1), 1);
        
        alarm_triggered = false(num_nodes, 1); 
        
        while succ < target_packets && succ < size(gen,1)
            next_unprocessed = find(~is_sent, 1);
            if isempty(next_unprocessed)
                break; 
            end
            
            current_time_pointer = max(clock, gen(next_unprocessed, 1));
            available_idx = find(~is_sent & gen(:,1) <= current_time_pointer + frame);
            
            if isempty(available_idx)
                available_idx = next_unprocessed;
            end
            
            n_focus = gen(available_idx(1), 2);
            node_avail_idx = available_idx(gen(available_idx, 2) == n_focus);
            node_packets = gen(node_avail_idx, :);
            
            has_emergency = any(node_packets(:, 4) == 1);
            
            if has_emergency && ~alarm_triggered(n_focus)
                [~, sort_order] = sortrows(node_packets, [-4, 1]); 
                alarm_triggered(n_focus) = true; 
            else
                [~, sort_order] = sortrows(node_packets, 1); 
            end
            
            selected_rel_idx = sort_order(1);
            p = node_avail_idx(selected_rel_idx);
            
            is_sent(p) = true;
            
            g = gen(p,1);      
            n = gen(p,2);      
            
            t_tx = transmission_time(n); 
            ch = assigned_CH(n);
            
            ready = max(g, next_avail(n)); 
            k = ceil((ready - t_tx)/frame);
            if k < 0, k = 0; end
            
            act = k * frame + t_tx;
            arr_time = act + T_delay_data + prop_delay(n, ch);
            
            dly = dly + (arr_time - g);
            total_energy_consumed = total_energy_consumed + calc_energy(dist_matrix(n, ch), data_packet_size);
            
            succ = succ + 1; 
            next_avail(n) = act + frame;
            clock = max(clock, arr_time);
        end

        sum_throughput = sum_throughput + ((target_packets*data_packet_size)/clock);
        sum_delay = sum_delay + (dly/target_packets);
        sum_energy = sum_energy + total_energy_consumed;
    end
    
    avg_throughput(n_idx) = sum_throughput/num_iterations;
    avg_delay(n_idx) = sum_delay/num_iterations;
    avg_energy(n_idx) = sum_energy/num_iterations;
end

%% Plotting Results
figure;
subplot(3,1,1); plot(ordinary_nodes_array, avg_throughput, '-o'); xlabel('Ordinary Nodes'); ylabel('Throughput (bps)'); title('Throughput'); grid on;
subplot(3,1,2); plot(ordinary_nodes_array, avg_delay, '-s'); xlabel('Ordinary Nodes'); ylabel('Delay (s)'); title('Delay'); grid on;
subplot(3,1,3); plot(ordinary_nodes_array, avg_energy, '-^'); xlabel('Ordinary Nodes'); ylabel('Energy (J)'); title('Energy'); grid on;

%% 
function active_node = get_current_transmitter(target_CH, member_list, assigned_CH, Status_msg, transmission_time, current_time, T_data)
    active_node = -1;
    for i = 1:length(member_list)
        m = member_list(i);
        if assigned_CH(m) == target_CH && Status_msg(m) == 1
            if current_time >= transmission_time(m) && current_time <= (transmission_time(m) + T_data)
                active_node = m;
                break; 
            end
        end
    end
end

function peer_node = get_active_same_coeff_node(curr, coeff, member_list, Status_msg, transmission_time, current_time, T_data)
    peer_node = -1;
    for i = 1:length(member_list)
        m = member_list(i);
        if m ~= curr && coeff(m) == coeff(curr) && Status_msg(m) == 1
            if current_time >= transmission_time(m) && current_time <= (transmission_time(m) + T_data)
                peer_node = m;
                break;
            end
        end
    end
end