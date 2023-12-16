// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the header of the shared class
#include "HelpfulClass.h"
#include "HelpfulFinal.h"

using namespace amp;


class GliderRRT {
    public:

        float step_size;
        float p_goal;
        int max_iter;
        float epsilon;
        float Length;
        float Width;
        float Height;
        float min_gamma_rate;
        float min_beta_rate;
        float gamma_variation;
        float beta_variation;
        int opt_iter;
        int timesteps;
        bool is_goal_moving;
        std::string goal_trajectory_file;


        std::vector<std::vector<float> > plan(std::vector<float> goal, std::vector<float> start_pos, std::vector<std::vector<float> > limits){

            std::vector<std::vector<float> > goal_vec(timesteps, std::vector<float>(3));
            if(is_goal_moving){
                std::ifstream infile;
                infile.open(goal_trajectory_file);
                float temp = 0;
                for(int i = 0; i < timesteps; i++){
                    for(int j = 0; j < 3; j++){
                        infile >> temp;
                        goal_vec[i][j] = temp;
                    }
                }
                infile.close();
            }
            else{
                for(int i = 0; i < timesteps; i++){
                    goal_vec[i] = goal;
                }
            }

            // for(int i = 0; i < goal_vec.size(); i++){
            //     for(int j = 0; j < goal_vec[i].size(); j++){
            //         std::cout << goal_vec[i][j] << " ";
            //     }
            //     std::cout << "\n";
            // }
            // sleep(5);

            std::map<amp::Node, std::vector<float> > node_to_coord;
            std::map<std::vector<float>, bool> input_set_check;
            std::shared_ptr<amp::Graph<double> > graph = std::make_shared<amp::Graph<double> >();
            float initial_v = 1.0;
            float initial_beta = 90.0;
            float initial_gamma = 0.0;
            float initial_timestep = 0.0;
            std::vector<float> start_vec = {start_pos[0], start_pos[1], start_pos[2], initial_beta, initial_gamma, initial_v, min_beta_rate, min_gamma_rate, initial_timestep, 0.0};
            node_to_coord[0] = start_vec;
            input_set_check[start_vec] = true;

            int current_iter = 0;
            bool reached_end = false;

            while(!reached_end && current_iter < max_iter){
                //std::cout<< "in the while loop at iteration " << current_iter << "\n"; 
                std::vector<float> random_point = PotentialPointGlider(p_goal, goal, limits);
                
                bool is_point_goal = false;
                if(random_point[0] == goal[0] && random_point[1] == goal[1] && random_point[2] == goal[2]){
                    is_point_goal = true;
                }

                // ------------------------------ FINDING CLOSEST NODE AND GENERATE POTENTIAL TRAJECTORIES FROM THERE--------------------------------------------------
                amp::Node closest_node;
                float beta_rate = min_beta_rate;
                float gamma_rate = min_gamma_rate;
                
                std::vector<std::vector<float> > state_vec(25, std::vector<float>(10, 1));
                std::vector<std::vector<float> > input_vec(25, std::vector<float>(10, 1));
                if(graph.get()->nodes().size() == 0){
                    for(int i = 0; i < opt_iter; i++){
                        gamma_rate = min_gamma_rate;
                        for(int j = 0; j < opt_iter; j++){
                            input_vec[i * opt_iter + j] = {start_pos[0], start_pos[1], start_pos[2], initial_beta, initial_gamma, initial_v, beta_rate, gamma_rate,initial_timestep}; 
                            std::vector<float> temp_state = IntegrateGlider(start_pos[0], start_pos[1], start_pos[2], initial_beta, initial_gamma, initial_v, beta_rate, gamma_rate, Length, Width, Height, initial_timestep);
                            for(int k = 0; k < temp_state.size(); k++){
                                state_vec[i * opt_iter + j][k] = temp_state[k];
                            }
                            gamma_rate = gamma_rate + gamma_variation;
                        }
                        beta_rate = beta_rate + beta_variation;
                    }
                    closest_node = 0;
                }
                else{
                    if(is_goal_moving && is_point_goal){
                        closest_node = FindClosestNodeGliderMovingGoal(goal_vec, graph, node_to_coord);
                    }
                    else{
                        closest_node = FindClosestNodeGlider(random_point, graph, node_to_coord);
                    }

                    if(closest_node == std::numeric_limits<uint32_t>::infinity()){
                        continue;
                    }

                    for(int i = 0; i < opt_iter; i++){
                        gamma_rate = min_gamma_rate;
                        for(int j = 0; j < opt_iter; j++){
                            input_vec[i * opt_iter + j] = {node_to_coord[closest_node][0], node_to_coord[closest_node][1], node_to_coord[closest_node][2], node_to_coord[closest_node][3], node_to_coord[closest_node][4], node_to_coord[closest_node][5], beta_rate, gamma_rate, node_to_coord[closest_node][8]}; 
                            if(input_set_check[input_vec[i * opt_iter + j]]){
                                gamma_rate = gamma_rate + gamma_variation;
                                continue;
                            }
                            std::vector<float> temp_state = IntegrateGlider(node_to_coord[closest_node][0], node_to_coord[closest_node][1], node_to_coord[closest_node][2], node_to_coord[closest_node][3], node_to_coord[closest_node][4], node_to_coord[closest_node][5], beta_rate, gamma_rate, Length, Width, Height, node_to_coord[closest_node][8]);
                            for(int k = 0; k < temp_state.size(); k++){
                                state_vec[i * opt_iter + j][k] = temp_state[k];
                            }
                            gamma_rate = gamma_rate + gamma_variation;
                        }
                        beta_rate = beta_rate + beta_variation;
                    } 
                }
                //------------------------------------------ GO THROUGH ALL POSSIBLE ENDING STATE VECTORS AND PICK THE BEST ONE-------------------------------------
                //TODO refactor to combine these two loops into one
                bool any_valid_traj = false;
                for(int i = 0; i < state_vec.size(); i++){
                    if(state_vec[i].back() == 0){
                        any_valid_traj = true;
                    }
                }
                int best_traj = std::numeric_limits<int>::infinity();
                if(any_valid_traj){
                    if(is_goal_moving && is_point_goal){
                        std::vector<float> random_point;
                        std::vector<float> current_goal_pos = goal_vec[node_to_coord[closest_node][8]];
                        if(node_to_coord[closest_node][8] >= 1){
                            std::vector<float> previous_goal_pos = goal_vec[node_to_coord[closest_node][8] - 1];
                            float dx = current_goal_pos[0] - previous_goal_pos[0];
                            float dy = current_goal_pos[1] - previous_goal_pos[1];
                            float dz = current_goal_pos[2] - previous_goal_pos[2];

                            random_point = {current_goal_pos[0] + dx, current_goal_pos[1] + dy, current_goal_pos[2] + dz};
                        }
                        else{
                            random_point = {current_goal_pos[0], current_goal_pos[1], current_goal_pos[2]};
                        }
                    }

                    float smallest_dist = std::numeric_limits<float>::infinity();
                    for(int i = 0; i < state_vec.size(); i++){
                        if(state_vec[i].back() == 0){
                            float challenge_dist = DistanceBetweenPointsGlider(state_vec[i][0], state_vec[i][1], state_vec[i][2], random_point);
                            if(challenge_dist < smallest_dist){
                                smallest_dist = challenge_dist;
                                best_traj = i;
                            }
                        }
                    }
                }
                else{ //CODE HERE FOR SITUATION WHERE ALL TRAJ ARE INVALID
                    continue;
                }
                //------------------------------------------------------- ADD A NODE TO GRAPH ----------------------------------------------------------------
                amp::Node new_node;
                if(graph.get()->nodes().size() == 0){
                    closest_node = 0;
                    new_node = 1;
                }
                else{
                    if(is_goal_moving){
                        goal = goal_vec[node_to_coord[closest_node][8] + 1];
                    }
                    float dist_to_goal = DistanceBetweenPointsGlider(state_vec[best_traj][0], state_vec[best_traj][1], state_vec[best_traj][2], goal);
                    if(dist_to_goal < epsilon){
                        reached_end = true;
                    }
                    new_node = graph.get()->nodes().size();
                }
                graph.get()->connect(closest_node, new_node, step_size); // HAVE THE ABILITY TO REPLACE STEP SIZE WITH SOME SORT OF DISTANCE METRIC IF NEEDED IN THE FUTURE
                node_to_coord[new_node] = state_vec[best_traj]; 
                input_set_check[input_vec[best_traj]] = true;                
                
                current_iter = current_iter + 1;
            }
            //----------------------------------------------------------- IF WE FOUND WERE ABLE TO GET TO THE GOAL DO A* TO FIND PATH -------------------------------------
            std::vector<std::vector<float> > path;
            if(reached_end){
                NewMyAstar astar_search;
                ShortestPathProblem path_to_pass;
                path_to_pass.graph = graph;
                path_to_pass.goal_node = node_to_coord.size() - 1;
                path_to_pass.init_node = 0;
                amp::AStar::GraphSearchResult graph_path = astar_search.search(path_to_pass,amp::SearchHeuristic());
                // std::ofstream file_for_path;
                // file_for_path.open("/home/josef/asen_5254_algorithmic_motion_planning/AMP-Tools-public/path.txt");
                for(auto i : graph_path.node_path){
                    path.push_back({node_to_coord[i][0], node_to_coord[i][1], node_to_coord[i][2], node_to_coord[i][8]});
                    // for(int j = 0; j < node_to_coord[i].size(); j++){
                    //     file_for_path << node_to_coord[i][j] << " ";
                    // }
                    // file_for_path << "\n";
                }
                // file_for_path.close();
            }
            //---------------------------------------------------------- OUTPUT ALL NODES FROM TREE TO TEXT FILE ---------------------------------------------------------------------------
            // std::ofstream file_for_all_points;
            // file_for_all_points.open("/home/josef/asen_5254_algorithmic_motion_planning/AMP-Tools-public/all_nodes.txt");
            // for(int i = 0; i < node_to_coord.size(); i++){
            //     for(int j = 0; j < 3; j++){
            //         file_for_all_points << node_to_coord[i][j] << " ";
            //     }
            //     file_for_all_points << "\n";
            // }
            // file_for_all_points.close();

            float number_nodes_in_tree = graph.get()->nodes().size();
            float number_nodes_in_path = path.size();
            float was_a_path_found = (float)reached_end;
            float place_hold = 0.0;
            std::vector<float> important_info = {number_nodes_in_tree, number_nodes_in_path, was_a_path_found, place_hold};
            path.push_back(important_info);
            path.push_back({0.0, 0.0, 0.0, 0.0});

            return path;
        }
};


int main(int argc, char** argv) {    

    GliderRRT glider;
    glider.epsilon = 1.0;
    glider.max_iter = 4000;
    glider.p_goal = 0.1;
    glider.step_size = 1;
    glider.min_beta_rate = -20.0;
    glider.min_gamma_rate = -20.0;
    glider.beta_variation = 20.0;
    glider.gamma_variation = 20.0;
    glider.opt_iter = 3;
    //glider.goal_trajectory_file = "/mnt/c/Users/Josef\ Michelsen/Documents/ASEN_5254_Algorithmic_Motion_Planning/Final/straight_goal.txt";
    glider.goal_trajectory_file = "/mnt/c/Users/Josef\ Michelsen/Documents/ASEN_5254_Algorithmic_Motion_Planning/Final/random_goal.txt";
    glider.timesteps = 100;
    glider.is_goal_moving = true;

    std::vector<float> goal = {-15, 70, 10};
    std::vector<float> start_position = {0, 0, 20};
    std::vector<std::vector<float> > limits = {{-20, 20}, {0, 90}, {0, 30}};
    
    Profiler profile;
    std::vector<std::vector<float> > total_info;
    std::vector<float> time_vec;

    //std::vector<std::vector<float> > important_info = glider.plan(goal, start_position, limits);


    // ------------------------------------------------- CODE FOR MONTE CARLO ------------------------------------------------------------------
    for(int i = 0; i < 20; i++){
        std::cout << "on MC run " << i << "\n";
        Timer time("glider");
        std::vector<std::vector<float> > important_info = glider.plan(goal, start_position, limits);
        
        time.stop();
        float glider_time = profile.getMostRecentProfile("glider", TimeUnit::s);
        time_vec.push_back(glider_time);
        for(int j = 0; j < important_info.size(); j++){
            total_info.push_back(important_info[j]);
        }
    }

    std::ofstream monte_carlo;
    monte_carlo.open("/home/josef/asen_5254_algorithmic_motion_planning/AMP-Tools-public/monte_carlo.txt");
    for(int i = 0; i < total_info.size(); i++){
        for(int j = 0; j < total_info[i].size(); j++){
            monte_carlo << total_info[i][j] << " ";
        }
        monte_carlo << time_vec[i];
        monte_carlo << "\n";
    }
    monte_carlo.close();

    return 0;
}
