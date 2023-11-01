// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW7.h"
//#include "hw/HW6.h"
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

class MyPRM2D : public PRM2D {
    
    public:

        int n;
        double r;

        virtual amp::Path2D plan(const amp::Problem2D& problem){
            amp::Path2D path;
            
            int point = 0;

            std::vector<Eigen::Vector2d> point_list = {{problem.q_init}};
            while(point < n){
                //std::cout << "here\n";
                double potential_point_x = amp::RNG::randd(problem.x_min , problem.x_max);
                double potential_point_y = amp::RNG::randd(problem.y_min , problem.y_max);

                bool node_collision = PointCollisionCheck(potential_point_x, potential_point_y, problem);
                if(!node_collision){
                    Eigen::Vector2d point_to_add(potential_point_x, potential_point_y);
                    point_list.push_back(point_to_add);
                }

                point = point + 1;
            }
            point_list.push_back(problem.q_goal);
            std::shared_ptr<amp::Graph<double> > graph = std::make_shared<amp::Graph<double> >();
            std::map<amp::Node, Eigen::Vector2d> node_to_coord;
            for(int i = 0; i < point_list.size(); i++){
                node_to_coord[i] = point_list[i];
            }
            for(int i = 0; i < point_list.size(); i++){
                for(int j = 0; j < point_list.size(); j++){
                    if(i != j){
                        double dist = DistanceBetweenNodes(point_list[i], point_list[j]);
                        if(dist <= r){
                            bool link_collision = ConnectionCollisionCheck(point_list[i], point_list[j], problem);
                            if(!link_collision){
                                graph.get()->connect(i,j,dist);
                            }
                        }
                    }
                }
            }

            NewMyAstar astar_search;
            ShortestPathProblem path_to_pass;
            //std::cout << "size of graph I am passing in " << graph.get()->nodes().size() << "\n";
            //graph.get()->print();
            path_to_pass.graph = graph;
            path_to_pass.goal_node = point_list.size() - 1;
            path_to_pass.init_node = 0;
            amp::AStar::GraphSearchResult graph_path = astar_search.search(path_to_pass,amp::SearchHeuristic());
            for(auto i : graph_path.node_path){
                path.waypoints.push_back(node_to_coord[i]);
            }
            //Path smoothing
            std::vector<Eigen::Vector2d> temp;
            for(int k = 0; k < 10 ; k++){
                for(int i = 0; i < path.waypoints.size() - 2; i++){
                    bool link_collision_temp = ConnectionCollisionCheck(path.waypoints[i], path.waypoints[i + 2], problem);
                    if(!link_collision_temp){
                        for(int j = 0; j < path.waypoints.size() - 1; j++){
                            if(j == i + 1){
                                continue;
                            }
                            else{
                                temp.push_back(path.waypoints[j]);
                            }
                        }
                        path.waypoints = temp;
                        break;
                    }
                }
            }

            // Visualizer test;
            // test.makeFigure(problem, path);
            // test.makeFigure(problem, *graph, node_to_coord);
            // test.showFigures();
            path.valid = graph_path.success;
            //graph.reset();
            return path;
        }

        // amp::Path2D PathSmoothing(amp::Problem2D& problem, std::shared_ptr<amp::Graph<double> > graph){
        //     amp::Path2D smooth_path;
        //     smooth_path.waypoints = {path.waypoints[0]};
        //     for(int i = 0; i < path.waypoints.size(); i++){
                
        //     }
        //     return smooth_path;
        // }

};


class MyGoalBiasRRT2D : public GoalBiasRRT2D {
    public:
        double epsilon;
        double r;
        double p;
        int n;

        virtual amp::Path2D plan(const amp::Problem2D& problem){
            amp::Path2D path;
            std::shared_ptr<amp::Graph<double> > graph = std::make_shared<amp::Graph<double> >();
            std::map<amp::Node, Eigen::Vector2d> node_to_coord;
            node_to_coord[0] = problem.q_init;
            bool reached_end = false;
            bool first_connection = false;
            int iter = 0;
            
            while(!reached_end && iter < n){
                double potential_point_x, potential_point_y;
                double prob_of_goal =  amp::RNG::randd(0 ,1);
                //std::cout << "iter " << iter << "\n";
                if(prob_of_goal < p){
                    //std::cout << "goal\n";
                    potential_point_x = problem.q_goal[0];
                    potential_point_y = problem.q_goal[1];
                }
                else{
                    potential_point_x = amp::RNG::randd(problem.x_min , problem.x_max);
                    potential_point_y = amp::RNG::randd(problem.y_min , problem.y_max);
                }

                bool link_collision;
                double closest_node_dist;
                amp::Node closest_node;
                Eigen::Vector2d point_to_add(potential_point_x, potential_point_y);
                std::vector<amp::Node> current_nodes;

                if(!first_connection){
                    closest_node_dist = DistanceBetweenNodes(problem.q_init, point_to_add);
                    closest_node = 0;
                    if(closest_node_dist > r){
                        point_to_add[0] = problem.q_init[0] + r * (point_to_add[0] - problem.q_init[0]) / std::abs(point_to_add[0] - problem.q_init[0]);
                        point_to_add[1] = problem.q_init[1] + r * (point_to_add[1] - problem.q_init[1]) / std::abs(point_to_add[1] - problem.q_init[1]);
                    }
                    link_collision = ConnectionCollisionCheck(problem.q_init, point_to_add, problem);
                }
                else{
                    current_nodes =  graph.get()->nodes();
                    closest_node = current_nodes[0];
                    closest_node_dist = DistanceBetweenNodes(node_to_coord[closest_node], point_to_add);
                    for(int i = 0; i < current_nodes.size(); i++){
                        double cur_dist = DistanceBetweenNodes(node_to_coord[current_nodes[i]], point_to_add);
                        if(cur_dist < closest_node_dist){
                            closest_node_dist = cur_dist;
                            closest_node = current_nodes[i];
                        }
                    }

                    if(closest_node_dist > r){
                        double x = point_to_add[0] - node_to_coord[closest_node][0];
                        double y = point_to_add[1] - node_to_coord[closest_node][1];
                        float theta = std::atan2(y,x);
                        point_to_add[0] = node_to_coord[closest_node][0] + (std::cos(theta) * r);
                        point_to_add[1] = node_to_coord[closest_node][1] + (std::sin(theta) * r);
                    }
                    link_collision = ConnectionCollisionCheck(node_to_coord[closest_node], point_to_add, problem);
                }
                bool node_collision = PointCollisionCheck(point_to_add[0], point_to_add[1], problem);

                if(!node_collision){
                    if(!link_collision){
                        first_connection = true;
                        amp::Node new_node = current_nodes.size();
                        node_to_coord.insert({new_node, point_to_add});
                        graph.get()->connect(closest_node, new_node, closest_node_dist);
                        double dist_to_goal = DistanceBetweenNodes(point_to_add, problem.q_goal);
                        if(dist_to_goal <= epsilon){
                            reached_end = true;
                            amp::Node goal_node = current_nodes.size();
                            node_to_coord.insert({goal_node, point_to_add});;
                            graph.get()->connect(new_node, goal_node, dist_to_goal);
                        }
                        iter = iter + 1;
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
                for(auto i : graph_path.node_path){
                    path.waypoints.push_back(node_to_coord[i]);
                }
                // Visualizer test;
                // test.makeFigure(problem, path);
                // test.makeFigure(problem, *graph, node_to_coord);
                // test.showFigures();
                path.valid = true;
            }
            else{
                path.valid = false;
            }

            return path;
        }
};


int main(int argc, char** argv) {    

    amp::Problem2D hw5_environment = amp::HW5::getWorkspace1();
    hw5_environment.x_min = -1;
    hw5_environment.x_max = 11;
    hw5_environment.y_min = -3;
    hw5_environment.y_max = 3;

    amp::Problem2D hw2_w1 = amp::HW2::getWorkspace1();
    amp::Problem2D hw2_w2 = amp::HW2::getWorkspace2();

    //hw5_environment.obstacles[1].

    // int n = 200;
    // double r = 1.0;

    MyPRM2D prm;
    MyGoalBiasRRT2D rrt;
    Profiler profile;
    
    std::vector<std::string> labels = {"time (ms)", "length", "success"};
    std::string title = "Exercise 2 part b HW02 Workspace 2 n = 5000, r = 0.5, p_goal = 0.05, epsilon = 0.25";
    std::string xlabel = "benchmark categories";
    std::string ylabel = "benchmark values";

    std::vector<double> time_vec = {};
    std::vector<double> length_vec = {};
    std::vector<double> success_vec = {};

    prm.n = 1000;
    prm.r = 2.0;

    rrt.n = 5000;
    rrt.r = 0.5;
    rrt.p = 0.05;
    rrt.epsilon = 0.25;

    // amp::Path2D path = rrt.plan(hw2_w2);
    // std::cout << "path length " << path.length() << "\n";
    

    // amp::Path2D path = prm.plan(hw2_w2);
    // std::cout << "path length " << path.length() << "\n";


    for(int i = 0; i < 100; i++){
        //std::cout << "we are in loop " << i << "\n";

        Timer time("test");
        amp::Path2D path = prm.plan(hw2_w1);
        time.stop();
        double prm_time = profile.getMostRecentProfile("test", TimeUnit::ms);
        time_vec.push_back(prm_time);
        if(path.valid){
            length_vec.push_back(path.length());
        }
        else{
            length_vec.push_back(0.0);
        }
        success_vec.push_back(path.valid);
    }

    // for(int i = 0; i < 1000; i++){
    //     //std::cout << "we are in loop " << i << "\n";

    //     Timer time("test");
    //     amp::Path2D path = rrt.plan(hw2_w2);
    //     time.stop();
    //     double prm_time = profile.getMostRecentProfile("test", TimeUnit::ms);
    //     time_vec.push_back(prm_time);
    //     if(path.valid){
    //         length_vec.push_back(path.length());
    //     }
    //     else{
    //         length_vec.push_back(0.0);
    //     }
    //     success_vec.push_back(path.valid);
    // }


    std::list<std::vector<double> > data_sets = {time_vec, length_vec, success_vec};
    //std::cout << "geting to vis \n";
    Visualizer box;
    box.makeBoxPlot(data_sets, labels, title, xlabel, ylabel);
    box.showFigures();

    // MyPRM2D* Myprm_algo = new MyPRM2D();
    // Myprm_algo->n = 1000;
    // Myprm_algo->r = 2.0;
    // std::unique_ptr<PRM2D> prm_algo;
    // prm_algo.reset(Myprm_algo);

    // MyGoalBiasRRT2D* Myrrt_algo = new MyGoalBiasRRT2D();
    // Myrrt_algo->epsilon = 0.25;
    // Myrrt_algo->n = 2000;
    // Myrrt_algo->r = 1.0;
    // Myrrt_algo->p = 0.05;
    // std::unique_ptr<GoalBiasRRT2D> rrt_algo;
    // rrt_algo.reset(Myrrt_algo);

    // int mygrade = amp::HW7::grade(*prm_algo, *rrt_algo, "jomi7243@colorado.edu", argc, argv);

    return 0;
}
