#include "HelpfulFinal.h"

//TODO incorporate effect of beta for x, y point calculations
bool CollisionCheckGlider(float x, float y, float z, float L, float W, float H){
    // Dont go below the floow
    if(z <= 0){
        return true;
    }
    // The Wall
    // if((x >= -20 && x <= 20) && (y >= 30 && y <= 40) && (z >= 0 && z <= 15)){
    //     return true;
    // }
    // // Struct 1
    // if((x >= -20 && x <= 5) && (y >= 30 && y <= 40) && (z >= 15 && z <= 30)){
    //     return true;
    // }
    // // Struct 2
    // if((x >= 10 && x <= 20) && (y >= 30 && y <= 40) && (z >= 15 && z <= 30)){
    //     return true;
    // }
    // // Roof
    // if((x >= -20 && x <= 20) && (y >= 30 && y <= 40) && (z >= 18 && z <= 30)){
    //     return true;
    // }

    //tree 1
    if((x >= -1 && x <= 1) && (y >= 35 && y <= 39) && (z >= 0 && z <= 20)){
        return true;
    }

    //tree 2
    if((x >= 3 && x <= 5) && (y >= 55 && y <= 59) && (z >= 0 && z <= 20)){
        return true;
    }

    //tree 3
    if((x >= -12 && x <= -7) && (y >= 45 && y <= 49) && (z >= 0 && z <= 20)){
        return true;
    }

    //tree 3
    if((x >= 13 && x <= 15) && (y >= 35 && y <= 39) && (z >= 0 && z <= 20)){
        return true;
    }
    
    // std::vector<std::vector<int> > obstacle_1 = {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}, {0, 10, 10}, {0, 0, 10}, {10, 0, 10}, {10, 10, 10}};
    // std::vector<float> x_vec = {x - L/2, x + L/2, x + L/2, x - L/2, x - L/2, x + L/2, x + L/2, x - L/2};
    // std::vector<float> y_vec = {y - W, y - W, y, y, y - W, y - W, y, y};
    // std::vector<float> z_vec = {z - H/2, z - H/2, z - H/2, z - H/2, z + H/2, z + H/2, z + H/2, z + H/2};
    // for(int i = 0; i < 8; i++){
    //     bool check_x = x_vec[i] >= obstacle_1[0][0] && x_vec[i] <= obstacle_1[7][0];
    //     bool check_y = y_vec[i] >= obstacle_1[0][1] && y_vec[i] <= obstacle_1[7][1];
    //     bool check_z = z_vec[i] >= obstacle_1[0][2] && z_vec[i] <= obstacle_1[7][2];
    //     if(check_x && check_y && check_z){
    //         return true;
    //     }
    // }
    return false;
}

 
std::vector<float> IntegrateGlider(float x, float y, float z, float beta, float gamma, float v, float beta_d, float gamma_d, float Length, float Width, float Height, float timestep){
    std::vector<float> input_state_vec = {x, y, z, beta, gamma, v, beta_d, gamma_d};
    float pi = 3.14159265;
    float rho = 1.225;
    //float c_l = 0.8;
    float c_d = 0.4;
    float s = 2;
    float m = 50;
    float g = 9.81;
    float dt = 0.03;
    float L, D, x_d, y_d, z_d, v_d; 
    float collision_in_traj = 0;

    for(int i = 0; i < 5; i++){
        //L = 0.5 * rho * std::pow(v, 2) * c_l * s;
        D = 0.5 * rho * std::pow(v, 2) * c_d * s;
        
        v_d = (-D - (m * g * std::sin(gamma * pi/180)))/m;
        x_d = v * std::cos(gamma * pi/180) * std::cos(beta * pi/180);
        y_d = v * std::cos(gamma * pi/180) * std::sin(beta * pi/180);
        z_d = v * std::sin(gamma * pi/180);

        x = x + x_d * dt;
        y = y + y_d * dt;
        z = z + z_d * dt;
        gamma = gamma + gamma_d * dt;
        beta = beta + beta_d * dt;
        v = v + v_d * dt;

        if(CollisionCheckGlider(x, y, z, Length, Width, Height)){
            collision_in_traj = 1;
            break;
        }
    }
    std::vector<float> state_vec = {x, y, z, beta, gamma, v, beta_d, gamma_d, timestep + 1, collision_in_traj};
    return state_vec;
}


std::vector<float> PotentialPointGlider(float prob_goal, std::vector<float> goal, std::vector<std::vector<float> > limits){
    float p_goal =  amp::RNG::randd(0, 1);
    std::vector<float> potential_point(3); 
    if(p_goal < prob_goal){
        potential_point[0] = goal[0];
        potential_point[1] = goal[1];
        potential_point[2] = goal[2];
    }
    else{
        potential_point[0] = amp::RNG::randd(limits[0][0] , limits[0][1]);
        potential_point[1] = amp::RNG::randd(limits[1][0] , limits[1][1]);
        potential_point[2] = amp::RNG::randd(limits[2][0] , limits[2][1]);
    }
    return potential_point;
}


float DistanceBetweenPointsGlider(float x, float y, float z, std::vector<float> point){
    return sqrt(pow(x - point[0], 2) + pow(y - point[1], 2) + pow(z - point[2], 2));
}

float DifferenceBetweenDirectionAndPoint(float beta, std::vector<float> next_point, std::vector<float> current_point){
    float pi = 3.14159265;
    beta = beta * pi/180;
    float current_to_next_x = next_point[0] - current_point[0]; 
    float current_to_next_y = next_point[1] - current_point[1]; 
    float angle = std::acos(current_to_next_x/sqrt(std::pow(current_to_next_x,2) + std::pow(current_to_next_y,2)));
    return abs(beta - angle);
}


amp::Node FindClosestNodeGlider(std::vector<float> next_positions, std::shared_ptr<amp::Graph<double> >& graph, std::map<amp::Node, std::vector<float> >& node_to_coord){
    float pi = 3.14159265;
    float smallest_total_distance = std::numeric_limits<float>::infinity();
    std::vector<std::vector<float> > distance_vec;
    amp::Node closest_node = std::numeric_limits<uint32_t>::infinity();
    for(int i = 0; i < graph.get()->nodes().size(); i++){
        float approx_x = node_to_coord[i][0] + node_to_coord[i][5] * std::cos(node_to_coord[i][4] * pi/180) * std::cos(node_to_coord[i][3] * pi/180) * 0.25;
        float approx_y = node_to_coord[i][1] + node_to_coord[i][5] * std::cos(node_to_coord[i][4] * pi/180) * std::sin(node_to_coord[i][3] * pi/180) * 0.25;
        float approx_z = node_to_coord[i][2] + node_to_coord[i][5] * std::sin(node_to_coord[i][4] * pi/180) * 0.25;
        float challenge_distance = DistanceBetweenPointsGlider(approx_x, approx_y, approx_z, next_positions);
        if(approx_z <= next_positions[2]){
            challenge_distance++;
        }

        distance_vec.push_back({(float)i, challenge_distance});
    }

    for(int i = 0; i < distance_vec.size() - 1; i++){
        for(int j = i + 1; j < distance_vec.size(); j++){
            if(distance_vec[j][1] > distance_vec[i][1]){
                std::vector<float> temp = distance_vec[j];
                distance_vec[j] = distance_vec[i];
                distance_vec[i] = temp;
            }
        }
    }

    int nodes_to_check;
    if(distance_vec.size() > 20){
        nodes_to_check = 20;
    }
    else{
        nodes_to_check = distance_vec.size();
    }

    float smallest_ang_dif = std::numeric_limits<float>::infinity();
    for(int i = 0; i < nodes_to_check; i++){
        int idx = distance_vec[distance_vec.size() - 1 - i][0];
        float ang_dif = DifferenceBetweenDirectionAndPoint(node_to_coord.at(idx)[3], next_positions, {node_to_coord.at(idx)[0], node_to_coord.at(idx)[1], node_to_coord.at(idx)[2]});
        if(ang_dif < smallest_ang_dif && graph.get()->children(idx).size() < 9){
            closest_node = idx;
            smallest_ang_dif = ang_dif;
        }
    }
    return closest_node;
}

//TODO MAKE SURE WE CANT GO PAST GOAL VEC SIZE 
amp::Node FindClosestNodeGliderMovingGoal(std::vector<std::vector<float> > goal_vec, std::shared_ptr<amp::Graph<double> >& graph, std::map<amp::Node, std::vector<float> >& node_to_coord){
    float pi = 3.14159265;
    float smallest_total_distance = std::numeric_limits<float>::infinity();
    std::vector<std::vector<float> > distance_vec;
    amp::Node closest_node = std::numeric_limits<uint32_t>::infinity();

    for(int i = 0; i < graph.get()->nodes().size(); i++){

        float cur_timestep = node_to_coord[i][8];
        // float cur_dist = DistanceBetweenPointsGlider(node_to_coord[i][0], node_to_coord[i][1], node_to_coord[i][2], goal_vec[cur_timestep]);
        // float time_to_hit_goal = cur_dist/node_to_coord[i][5]
        if(cur_timestep >= goal_vec.size()){
            continue;
        }
        std::vector<float> next_goal_pos;
        if(cur_timestep == 0){
            next_goal_pos = goal_vec[cur_timestep];
        }
        else{
            std::vector<float> previous_goal_pos = goal_vec[cur_timestep - 1];
            std::vector<float> current_goal_pos = goal_vec[cur_timestep];
            float dx = current_goal_pos[0] - previous_goal_pos[0];
            float dy = current_goal_pos[1] - previous_goal_pos[1];
            float dz = current_goal_pos[2] - previous_goal_pos[2];

            next_goal_pos = {current_goal_pos[0] + dx, current_goal_pos[1] + dy, current_goal_pos[2] + dz};
        }

        float approx_x = node_to_coord[i][0] + node_to_coord[i][5] * std::cos(node_to_coord[i][4] * pi/180) * std::cos(node_to_coord[i][3] * pi/180) * 0.225;
        float approx_y = node_to_coord[i][1] + node_to_coord[i][5] * std::cos(node_to_coord[i][4] * pi/180) * std::sin(node_to_coord[i][3] * pi/180) * 0.225;
        float approx_z = node_to_coord[i][2] + node_to_coord[i][5] * std::sin(node_to_coord[i][4] * pi/180) * 0.225;
        
        float challenge_distance = DistanceBetweenPointsGlider(approx_x, approx_y, approx_z, next_goal_pos);
        if(approx_z <= next_goal_pos[2]){
            challenge_distance++;
        }
        distance_vec.push_back({(float)i, challenge_distance});
    }

     for(int i = 0; i < distance_vec.size() - 1; i++){
        for(int j = i + 1; j < distance_vec.size(); j++){
            if(distance_vec[j][1] > distance_vec[i][1]){
                std::vector<float> temp = distance_vec[j];
                distance_vec[j] = distance_vec[i];
                distance_vec[i] = temp;
            }
        }
    }

    int nodes_to_check;
    if(distance_vec.size() > 20){
        nodes_to_check = 5;
    }
    else{
        nodes_to_check = distance_vec.size();
    }

    float smallest_ang_dif = std::numeric_limits<float>::infinity();
    for(int i = 0; i < nodes_to_check; i++){
        int idx = distance_vec[distance_vec.size() - 1 - i][0];
        float ang_dif = DifferenceBetweenDirectionAndPoint(node_to_coord.at(idx)[3], goal_vec[node_to_coord.at(idx)[8] + 1], {node_to_coord.at(idx)[0], node_to_coord.at(idx)[1], node_to_coord.at(idx)[2]});
        if(ang_dif < smallest_ang_dif && graph.get()->children(idx).size() < 9){
            closest_node = idx;
            smallest_ang_dif = ang_dif;
        }
    }
    return closest_node;
}


amp::AStar::GraphSearchResult GliderMyAstar::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic){

    amp::AStar::GraphSearchResult ans;

    std::vector<amp::Node> open_set = {problem.init_node};
    std::vector<amp::Node> closed_set = {};
    std::vector<amp::Node> temp_vec = problem.graph.get()->nodes();
    int max_size = 0;
    for(int i = 0; i < problem.graph.get()->nodes().size(); i++){
        if(temp_vec.at(i) >= max_size){
            max_size = (int)temp_vec.at(i) + 1;
        }
    }
    std::vector<double> g_score(max_size, std::numeric_limits<double>::infinity());
    std::vector<double> f_score(max_size, std::numeric_limits<double>::infinity());
    std::vector<amp::Node> came_from(max_size);
    f_score[problem.init_node] = heuristic.operator()(problem.init_node);
    g_score[problem.init_node] = 0;
    int number_of_iterations = 0;
    
    while(!open_set.empty()){
        double lowest_f_score = f_score[open_set[0]];
        int next_node_index = 0;

        for(int i = 0; i < open_set.size(); i++){
            double challenge_f_score = f_score[open_set[i]];

            if(challenge_f_score < lowest_f_score){
                lowest_f_score = challenge_f_score;
                next_node_index = i;
            }
        }

        amp::Node current_node = open_set[next_node_index];
        if(current_node == problem.goal_node){
            ans.success = true;
            amp::Node path_iter = problem.goal_node;
            std::list<amp::Node> path;
            path.push_front(path_iter);
            while(path_iter != problem.init_node){
                path_iter = came_from[path_iter];
                path.push_front(path_iter); 
            }
            ans.node_path = path;
            ans.path_cost = g_score[problem.goal_node];
            return ans;
        }
        
        open_set.erase(open_set.begin() + next_node_index);

        std::vector<amp::Node> children_of_current_node = problem.graph.get()->children(current_node);
        std::vector<double> edge_weights_from_current_node = problem.graph.get()->outgoingEdges(current_node); 
        std::map<amp::Node, int> node_to_vec_indx;   

        for(int i = 0; i < children_of_current_node.size(); i++){
            double tentative_gscore = g_score[current_node] + edge_weights_from_current_node[i];
            if(tentative_gscore < g_score.at(children_of_current_node[i])){
                came_from[children_of_current_node[i]] = current_node;
                g_score[children_of_current_node[i]] = tentative_gscore;
                f_score[children_of_current_node[i]] = tentative_gscore + heuristic.operator()(children_of_current_node[i]);
                
                bool is_neighbor_in_open_set = false;
                for(int j = 0; j < open_set.size(); j++){
                    if(open_set[j] == children_of_current_node[i]){
                        is_neighbor_in_open_set = true;
                    }
                }

                if(is_neighbor_in_open_set == false){
                    open_set.push_back(children_of_current_node[i]);
                }
            }
        }

        number_of_iterations = number_of_iterations + 1;
    }

    std::list<amp::Node> fail_path = {problem.init_node};
    ans.success = false;
    ans.node_path = fail_path;
    ans.path_cost = 0;

    return ans;
}