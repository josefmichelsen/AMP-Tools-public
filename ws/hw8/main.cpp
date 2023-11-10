// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW8.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

class MyCentralizedMultiAgentRRT : public CentralizedMultiAgentRRT {
    public:

        double step_size;
        double p_goal;
        int max_iter;
        double epsilon;
        int tree_size = 0;


        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem){
            amp::MultiAgentPath2D path(problem.numAgents());
            std::map<amp::Node, std::vector<Eigen::Vector2d> > node_to_coord;
            std::shared_ptr<amp::Graph<double> > graph = std::make_shared<amp::Graph<double> >();
            std::vector<Eigen::Vector2d> start_positions;
            
            for(int i = 0; i < problem.agent_properties.size(); i++){
                start_positions.push_back(problem.agent_properties[i].q_init);
            }
            
            node_to_coord[0] = start_positions;

            int current_iter = 0;
            bool reached_end = false;
            std::vector<bool> robots_at_goal(problem.agent_properties.size(), false); 
            while(!reached_end && current_iter < max_iter){
                std::cout<< "in the while loop at iteration " << current_iter << "\n"; 
                std::vector<Eigen::Vector2d> next_positions;
                std::vector<Eigen::Vector2d> potential_points;
                for(int i = 0; i < problem.agent_properties.size(); i++){
                    potential_points.push_back(PotentialPoint(p_goal, problem, i));
                }
                // ------------------------------ FINDING CLOSEST NODE AND AUGMENTING THE NEXT POSITIONS --------------------------------------------------
                amp::Node closest_node;
                if(graph.get()->nodes().size() == 0){
                    for(int i = 0; i < problem.numAgents(); i++){
                        double dist_from_closest_node = DistanceBetweenNodes(problem.agent_properties[i].q_init, potential_points[i]);
                        if(dist_from_closest_node > step_size){
                            next_positions.push_back(NewCloserPoint(problem.agent_properties[i].q_init, potential_points[i], step_size));
                        }
                        else{
                            next_positions.push_back(potential_points[i]);
                        }
                    }
                }
                else{
                    closest_node = FindClosestNode(potential_points, graph, node_to_coord, problem, robots_at_goal); 
                    for(int i = 0; i < problem.numAgents(); i++){
                        if(robots_at_goal[i]){
                            next_positions.push_back(problem.agent_properties[i].q_goal);
                        }
                        else{
                            double dist_from_closest_node = DistanceBetweenNodes(node_to_coord.at(closest_node)[i], potential_points[i]);
                            if(dist_from_closest_node > step_size){
                                next_positions.push_back(NewCloserPoint(node_to_coord.at(closest_node)[i], potential_points[i], step_size));
                            }
                            else{
                                next_positions.push_back(potential_points[i]);
                            }
                        }
                    }
                }
                //------------------------CHECK IF ANY OF THE POTENTIAL POINTS HAVE COLLISIONS, IF YES DONT DO THAT TIMESTEP ----------------------
                bool robot_v_obstacle = false;
                for(int i = 0; i < problem.numAgents(); i++){
                    double robot_radius = problem.agent_properties[i].radius;
                    std::vector<Eigen::Vector2d> corners = SquareAgentCorners(next_positions[i], robot_radius); 
                    for(int j = 0; j < corners.size() ; j++){
                        bool corner_collision = PointCollisionCheckMultiAgent(corners[j][0], corners[j][1], problem);
                        if(corner_collision){
                            robot_v_obstacle = true;
                        }
                    }
                }
                if(!robot_v_obstacle){
                    for(int i = 0; i < problem.numAgents(); i++){
                        double robot_radius = problem.agent_properties[i].radius;
                        std::vector<Eigen::Vector2d> corners = SquareAgentCorners(next_positions[i], robot_radius);
                        for(int j = 0; j < corners.size() ; j++){
                            bool edge_collision;
                            if(j == corners.size() - 1){
                                edge_collision = NewConnectionCollisionCheck(corners[j], corners[0], problem);
                            }
                            else{
                                edge_collision = NewConnectionCollisionCheck(corners[j], corners[j + 1], problem);
                            }
                            if(edge_collision){
                                robot_v_obstacle = true;
                            }
                        }
                    }
                }
                bool robot_v_robot = WillRobotsCollide(next_positions, problem);
                //------------------------ADD A NODE IF WE DONT FIND ANY COLLISIONS----------------------------------------------------------------
                if(robot_v_obstacle == false && robot_v_robot == false){
                    
                    amp::Node new_node;
                    if(graph.get()->nodes().size() == 0){
                        closest_node = 0;
                        new_node = 1;
                    }
                    else{
                        for(int i = 0; i < problem.numAgents(); i++){
                            double dist_to_goal = DistanceBetweenNodes(next_positions[i], problem.agent_properties[i].q_goal);
                            if(dist_to_goal < epsilon){
                                next_positions[i] = problem.agent_properties[i].q_goal;
                                robots_at_goal[i] = true;
                                
                            }
                        }
                        new_node = graph.get()->nodes().size();
                    }
                    graph.get()->connect(closest_node, new_node, step_size);
                    node_to_coord[new_node] = next_positions;                 
                    
                    current_iter = current_iter + 1;
                }
                if(std::all_of(robots_at_goal.begin(), robots_at_goal.end(), [](bool v) { return v; })){
                    reached_end = true;
                }
            }
            if(reached_end){
                NewMyAstar astar_search;
                ShortestPathProblem path_to_pass;
                path_to_pass.graph = graph;
                path_to_pass.goal_node = node_to_coord.size() - 1;
                path_to_pass.init_node = 0;
                amp::AStar::GraphSearchResult graph_path = astar_search.search(path_to_pass,amp::SearchHeuristic());
                
                for(int j = 0; j < problem.numAgents(); j++){
                    Path2D temp_path;
                    for(auto i : graph_path.node_path){
                        temp_path.waypoints.push_back(node_to_coord[i][j]);
                    }
                    path.agent_paths[j] = temp_path;
                }
                path.valid = true;
                // Visualizer test;
                // test.makeFigure(problem, path);
                // test.showFigures();
            }
            else{
                path.valid = false;
            }
            tree_size = graph.get()->nodes().size();
            DoesRobotCollisionExist(path);
            return path;
        }
};

class MyDecentralizedMultiAgentRRT : public DecentralizedMultiAgentRRT {
    public:

        double step_size;
        double p_goal;
        int max_iter;
        double epsilon;

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem){
            amp::MultiAgentPath2D path(problem.numAgents());
            std::vector<bool> path_validity_vec(problem.numAgents(), false);

            for(int agent = 0; agent < problem.numAgents(); agent++){ 
                // ------------------------------- CREATING NEW MAP, GRAPH, AND START POSITION FOR EACH ROBOT ------------------------------------------------
                std::map<amp::Node, Eigen::Vector2d> node_to_coord; 
                std::shared_ptr<amp::Graph<double> > graph = std::make_shared<amp::Graph<double> >();

                node_to_coord[0] = problem.agent_properties[agent].q_init;

                int current_iter = 0;
                bool reached_end = false;

                while(!reached_end && current_iter < max_iter){
                    std::cout<< "in the while loop at iteration " << current_iter << " for agent " << agent << "\n"; 
                    Eigen::Vector2d next_position;
                    Eigen::Vector2d potential_point = PotentialPoint(p_goal, problem, agent);
                    // ------------------------------ FINDING CLOSEST NODE AND AUGMENTING THE NEXT POSITIONS --------------------------------------------------
                    amp::Node closest_node;
                    if(graph.get()->nodes().size() == 0){
                        double dist_from_closest_node = DistanceBetweenNodes(problem.agent_properties[agent].q_init, potential_point);
                        if(dist_from_closest_node > step_size){
                            next_position = NewCloserPoint(problem.agent_properties[agent].q_init, potential_point, step_size);
                        }
                        else{
                            next_position = potential_point;
                        }
                    }
                    else{
                        closest_node = FindClosestNodeDecentralized(potential_point, graph, node_to_coord);
                        double dist_from_closest_node = DistanceBetweenNodes(node_to_coord.at(closest_node), potential_point);
                        if(dist_from_closest_node > step_size){
                            next_position = NewCloserPoint(node_to_coord.at(closest_node), potential_point, step_size);
                        }
                        else{
                            next_position = potential_point;
                        }
                    }
                    //------------------------CHECK IF ANY OF THE POTENTIAL POINTS HAVE COLLISIONS, IF YES DONT DO THAT TIMESTEP ----------------------
                    bool robot_v_obstacle = false;
                    double robot_radius = problem.agent_properties[agent].radius;

                    std::vector<Eigen::Vector2d> corners = SquareAgentCorners(next_position, robot_radius); 
                    for(int j = 0; j < corners.size() ; j++){
                        bool corner_collision = PointCollisionCheckMultiAgent(corners[j][0], corners[j][1], problem);
                        if(corner_collision){
                            robot_v_obstacle = true;
                        }
                    }

                    //------------------------ADD A NODE IF WE DONT FIND ANY COLLISIONS----------------------------------------------------------------
                    if(robot_v_obstacle == false){
                        bool robot_v_robot;
                        amp::Node new_node;
                        if(graph.get()->nodes().size() == 0){
                            closest_node = 0;
                            new_node = 1;
                            robot_v_robot = false;
                        }
                        else{
                            double dist_to_goal = DistanceBetweenNodes(next_position, problem.agent_properties[agent].q_goal);
                            if(dist_to_goal < epsilon){
                                next_position = problem.agent_properties[agent].q_goal;
                                reached_end = true;
                            }
                            new_node = graph.get()->nodes().size();
                            
                            NewMyAstar collision_astar;
                            ShortestPathProblem collision_path_to_pass;
                            collision_path_to_pass.graph = graph;
                            collision_path_to_pass.goal_node = closest_node;
                            collision_path_to_pass.init_node = 0;
                            amp::AStar::GraphSearchResult collision_graph_path = collision_astar.search(collision_path_to_pass,amp::SearchHeuristic());

                            robot_v_robot = WillRobotsCollideDecentralized(next_position, path, agent, collision_graph_path.node_path.size(), robot_radius);//checking wrong time step
                        }

                        if(!robot_v_robot){
                            graph.get()->connect(closest_node, new_node, step_size);
                            node_to_coord[new_node] = next_position;                 
                            
                            current_iter = current_iter + 1;
                        }
                    }
                }

                if(reached_end){
                    NewMyAstar astar_search;
                    ShortestPathProblem path_to_pass;
                    path_to_pass.graph = graph;
                    path_to_pass.goal_node = node_to_coord.size() - 1;
                    path_to_pass.init_node = 0;
                    amp::AStar::GraphSearchResult graph_path = astar_search.search(path_to_pass,amp::SearchHeuristic());
                    
                    Path2D temp_path;
                    for(auto i : graph_path.node_path){
                        temp_path.waypoints.push_back(node_to_coord[i]);
                    }
                    path.agent_paths[agent] = temp_path;
                    path_validity_vec[agent] = true;
                }
                else{
                    path_validity_vec[agent] = false;
                }
                graph.get()->clear();
            }

            int longest_path = 0;
            for(int i = 0; i < problem.numAgents(); i++){
                if(path.agent_paths[i].waypoints.size() > longest_path){
                    longest_path = path.agent_paths[i].waypoints.size();
                }
            }

            for(int i = 0; i < problem.numAgents(); i++){
                if(path.agent_paths[i].waypoints.size() < longest_path){
                    int path_size_dif = longest_path - path.agent_paths[i].waypoints.size();
                    for(int j = 0; j < path_size_dif; j++){
                        path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);
                    }
                }
            }

            if(std::all_of(path_validity_vec.begin(), path_validity_vec.end(), [](bool v) { return v; })){
                path.valid = true;
            }
            else{
                path.valid = false;
            }

            // Visualizer test;
            // test.makeFigure(problem, path);
            // test.showFigures();

            return path;
        }
};



int main(int argc, char** argv) {    

    amp::MultiAgentProblem2D ex_1 = amp::HW8::getWorkspace1();
    std::vector<CircularAgentProperties> agent_properties_custom;
    CircularAgentProperties robot_1;
    robot_1.radius = 0.5;
    robot_1.q_init = {2,2};
    robot_1.q_goal = {14,14};
     CircularAgentProperties robot_2;
    robot_2.radius = 0.5;
    robot_2.q_init = {2,14};
    robot_2.q_goal = {14,2};
     CircularAgentProperties robot_3;
    robot_3.radius = 0.5;
    robot_3.q_init = {8,14};
    robot_3.q_goal = {8,2};
     CircularAgentProperties robot_4;
    robot_4.radius = 0.5;
    robot_4.q_init = {2,8};
    robot_4.q_goal = {14,8};
     CircularAgentProperties robot_5;
    robot_5.radius = 0.5;
    robot_5.q_init = {11,2};
    robot_5.q_goal = {5,14};
     CircularAgentProperties robot_6;
    robot_6.radius = 0.5;
    robot_6.q_init = {11,14};
    robot_6.q_goal = {5,2};

    agent_properties_custom.push_back(robot_1);
    agent_properties_custom.push_back(robot_2);
    // agent_properties_custom.push_back(robot_3);
    // agent_properties_custom.push_back(robot_4);
    // agent_properties_custom.push_back(robot_5);
    // agent_properties_custom.push_back(robot_6);


    ex_1.agent_properties = agent_properties_custom;

    // --------------------------------------------------------------- CENTRALIZED RRT ----------------------------------------------------------

    // MyCentralizedMultiAgentRRT central_rrt;
    // central_rrt.p_goal = 0.05;
    // central_rrt.step_size = 0.3;
    // central_rrt.max_iter = 7500;
    // central_rrt.epsilon = 0.25;

    

    
    //bool check_ex1 = amp::HW8::check(path_ex1, ex_1,collision_states, true);

    // for(int i = 0; i < 20; i++){
    //     amp::MultiAgentPath2D path_ex1 = central_rrt.plan(ex_1);
    //     std::vector<std::vector<Eigen::Vector2d>> collision_states = {{{}}};
    //     bool check_ex1 = amp::HW8::check(path_ex1, ex_1, collision_states, true);
    //     if(!check_ex1){
    //         Visualizer bug;
    //         bug.makeFigure(ex_1, path_ex1, collision_states);
    //         bug.showFigures();
    //     }
    // }

    // Visualizer path_plot;
    // path_plot.makeFigure(ex_1, path_ex1,collision_states);
    // path_plot.showFigures();

    // MyCentralizedMultiAgentRRT* centralrrt = new MyCentralizedMultiAgentRRT();
    // centralrrt->p_goal = 0.05;
    // centralrrt->step_size = 0.5;
    // centralrrt->max_iter = 7500;
    // centralrrt->epsilon = 0.25;
    // std::unique_ptr<MultiAgentCircleMotionPlanner2D> centralrrt_to_pass;
    // centralrrt_to_pass.reset(centralrrt);
 
    // //bool pass_ex1 = amp::HW8::generateAndCheck(*centralrrt_to_pass, path_ex1, ex_1, true, 0u);

    // for(int i = 0; i < 15; i++){
    //     bool pass = amp::HW8::generateAndCheck(*centralrrt_to_pass, true, 0u);
    // }
    
    
    // std::cout << "tree size is " << central_rrt.tree_size << "\n";




    // ------------------------------------------------------ BOX PLOTS FOR CENTRAL ---------------------------------------------------------

    // std::vector<std::string> labels = {"time (ms)", "tree size"};
    // std::string title = "Exercise 1 with 6 robots n = 7500, r = 0.5, p_goal = 0.05, epsilon = 0.25";
    // std::string xlabel = "benchmark categories";
    // std::string ylabel = "benchmark values";

    // Profiler profile;

    // std::vector<double> time_vec = {};
    // std::vector<double> tree_size = {};

    // for(int i = 0; i < 100; i++){
    //     Timer time("test");
    //     amp::MultiAgentPath2D path = central_rrt.plan(ex_1);
    //     time.stop();
    //     double central_rrt_time = profile.getMostRecentProfile("test", TimeUnit::ms);
    //     time_vec.push_back(central_rrt_time);
    //     tree_size.push_back(central_rrt.tree_size);
       
    // }
    // std::list<std::vector<double> > data_sets = {time_vec, tree_size};

    // Visualizer box_plots_central;
    // box_plots_central.makeBoxPlot(data_sets, labels, title, xlabel, ylabel);
    // box_plots_central.showFigures();
    
    // int tree_size_sum = 0;
    // int average_time = 0;
    // for(int i = 0; i < tree_size.size(); i++){
    //     tree_size_sum = tree_size_sum + tree_size[i]; 
    //     average_time = average_time + time_vec[i];   
    // }
    // std::cout << "average tree size is " << tree_size_sum/100 << "\n";
    // std::cout << "average time is " << average_time/100 << "\n";

    //-------------------------------------------------- DECENTRALIZED RRT -------------------------------------

    // MyDecentralizedMultiAgentRRT decentralized_rrt;
    // decentralized_rrt.p_goal = 0.05;
    // decentralized_rrt.step_size = 0.3;
    // decentralized_rrt.max_iter = 7500;
    // decentralized_rrt.epsilon = 0.25;

    // for(int i = 0; i < 100; i++){
    //     amp::MultiAgentPath2D path_ex1 = decentralized_rrt.plan(ex_1);
    //     std::vector<std::vector<Eigen::Vector2d>> collision_states = {{{}}};
    //     bool check_ex1 = amp::HW8::check(path_ex1, ex_1, collision_states, true);
    //     if(!check_ex1){
    //         Visualizer bug;
    //         bug.makeFigure(ex_1, path_ex1, collision_states);
    //         bug.showFigures();
    //     }
    // }

    // amp::MultiAgentPath2D decentral_path = decentralized_rrt.plan(ex_1);
    
    // std::vector<std::vector<Eigen::Vector2d>> decentral_collision_states;

    // bool check_ex1_decentral = amp::HW8::check(decentral_path, ex_1, decentral_collision_states, true);

    // Visualizer decentral;
    // decentral.makeFigure(ex_1, decentral_path, decentral_collision_states);
    // decentral.showFigures();

    // bool did_i_pass_decentral = amp::HW8::check(decentral_path, ex_1, collision_states, true);
    // std::cout << "did I pass the decentralized example " << did_i_pass_decentral << "\n";
    //std::cout << "size of collision states " << collision_states.size() << "\n";

    // for(int i = 0; i < collision_states.size(); i++){
    //     std::cout << "collision state " << i << "\n";
    //     for(int j = 0; j < collision_states[i].size(); j++){
    //         std::cout << collision_states[i][j][0] << " " << collision_states[i][j][1] << "\n"; 
    //     }
    // }

    // for(int i = 0; i < decentral_path.numAgents(); i++){
    //     std::cout << "agent " << i << " path length " << decentral_path.agent_paths[i].waypoints.size() << "\n";
    // }

    // Visualizer test;
    // test.makeFigure(ex_1, decentral_path);
    // test.showFigures();

    // int smallest_path = decentral_path.agent_paths[0].waypoints.size();
    // for(int i = 0; i < decentral_path.numAgents(); i++){
    //     if(decentral_path.agent_paths[i].waypoints.size() < smallest_path){
    //         smallest_path = decentral_path.agent_paths[i].waypoints.size();
    //     }
    // }

    // ------------------------------------------------------ BOX PLOTS FOR DE-CENTRAL ---------------------------------------------------------

    MyDecentralizedMultiAgentRRT decentralized_rrt_box;
    decentralized_rrt_box.p_goal = 0.05;
    decentralized_rrt_box.step_size = 0.3;
    decentralized_rrt_box.max_iter = 7500;
    decentralized_rrt_box.epsilon = 0.25;
    
    std::vector<std::string> labels = {"time (ms)"};
    std::string title = "Exercise 2 with 2 robots n = 7500, r = 0.5, p_goal = 0.05, epsilon = 0.25";
    std::string xlabel = "benchmark categories";
    std::string ylabel = "benchmark values";

    Profiler profile;

    std::vector<double> time_vec = {};

    for(int i = 0; i < 100; i++){
        Timer time("test");
        amp::MultiAgentPath2D path = decentralized_rrt_box.plan(ex_1);
        time.stop();
        double de_central_rrt_time = profile.getMostRecentProfile("test", TimeUnit::ms);
        time_vec.push_back(de_central_rrt_time);

        // if(i == 99){
        //     Visualizer de_central_plots;
        //     de_central_plots.makeFigure(ex_1, path);
        //     de_central_plots.showFigures();
        // }
    }
    std::list<std::vector<double> > data_sets = {time_vec};

    Visualizer box_plots_central;
    box_plots_central.makeBoxPlot(data_sets, labels, title, xlabel, ylabel);
    box_plots_central.showFigures();
    
    int average_time = 0;
    for(int i = 0; i < time_vec.size(); i++){
        average_time = average_time + time_vec[i];   
    }

    std::cout << "average time is " << average_time/100 << "\n";

    // --------------------------------------------------------------------- GRADER --------------------------------------------------------------------------------

    // MyCentralizedMultiAgentRRT* central = new MyCentralizedMultiAgentRRT();
    // central->p_goal = 0.05;
    // central->step_size = 0.3;
    // central->max_iter = 7500;
    // central->epsilon = 0.25;
    // std::unique_ptr<CentralizedMultiAgentRRT> central_pass;
    // central_pass.reset(central);

    // MyDecentralizedMultiAgentRRT* de_central = new MyDecentralizedMultiAgentRRT();
    // de_central->p_goal = 0.05;
    // de_central->step_size = 0.3;
    // de_central->max_iter = 7500;
    // de_central->epsilon = 0.25;
    // std::unique_ptr<DecentralizedMultiAgentRRT> de_central_pass;
    // de_central_pass.reset(de_central);
    
    
    // int final_grade = amp::HW8::grade(*central_pass, *de_central_pass, "jomi7243@colorado.edu", argc, argv);

    return 0;
}
