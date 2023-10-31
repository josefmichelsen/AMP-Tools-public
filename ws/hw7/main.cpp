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
            //std::cout << "before path\n";
            amp::Path2D path;
            
            // int n = this -> n;
            // double r = 1.0;
            int point = 0;

            std::vector<Eigen::Vector2d> point_list = {{problem.q_init}};
            //std::cout << "before while\n";
            while(point < n){
                double potential_point_x = amp::RNG::randd(problem.x_min , problem.x_max);
                double potential_point_y = amp::RNG::randd(problem.y_min , problem.y_max);
                //std::cout << "before point collision\n";
                bool node_collision = PointCollisionCheck(potential_point_x, potential_point_y, problem);
                //std::cout << "after point collision\n";
                if(!node_collision){
                    Eigen::Vector2d point_to_add(potential_point_x, potential_point_y);
                    //std::cout << "before pushing back point\n";
                    point_list.push_back(point_to_add);
                    //std::cout << "after pushing back point\n";
                }

                point = point + 1;
            }
            //std::cout << "after while\n";
            point_list.push_back(problem.q_goal);
            //std::cout << "before graph\n";
            std::shared_ptr<amp::Graph<double> > graph = std::make_shared<amp::Graph<double> >();
            //std::cout << "after graph\n";
            std::map<amp::Node, Eigen::Vector2d> node_to_coord;
            //std::cout << "before map\n";
            for(int i = 0; i < point_list.size(); i++){
                node_to_coord[i] = point_list[i];
            }
            //std::cout << "after map\n";
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
            //std::cout << "after connections\n";

            // Visualizer test;
            // test.makeFigure(problem, *graph, node_to_coord);

            NewMyAstar astar_search;
            ShortestPathProblem path_to_pass;
            path_to_pass.graph = graph;
            path_to_pass.goal_node = point_list.size() - 1;
            path_to_pass.init_node = 0;
            //std::cout << "before astar\n";
            amp::AStar::GraphSearchResult graph_path = astar_search.search(path_to_pass,amp::SearchHeuristic());
            //std::cout << "aftr astar\n";
            //std::cout << "path cost " << graph_path.path_cost << "\n";
            //double new_dist = 0;
            for(auto i : graph_path.node_path){
                path.waypoints.push_back(node_to_coord[i]);
            }
            //std::cout << "length of waypoints " << path.waypoints.size() << "\n";
            //std::cout << "new path cost " << path.length() << "\n";
            //path.length() = 4;
            Visualizer test;
            test.makeFigure(problem, path);
            //std::cout << "make 1\n";
            test.makeFigure(problem, *graph, node_to_coord);
            //std::cout << "make 2\n";
            test.showFigures();
            //std::cout << "graph path success " << graph_path.success << "\n";
            path.valid = graph_path.success;
            //graph.reset();
            //std::cout << "returning\n";
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
    //amp::Problem2D hw2_w2 = amp::HW2::getWorkspace2();

    //hw5_environment.obstacles[1].

    // int n = 200;
    // double r = 1.0;

    MyPRM2D prm;
    Profiler profile;
    
    std::vector<std::string> labels = {"time (ms)", "length", "success"};
    std::string title = "Exercise 1.b Workspace 1 n = 500, r = 1.0";
    std::string xlabel = "benchmark categories";
    std::string ylabel = "benchmark values";

    std::vector<double> time_vec = {};
    std::vector<double> length_vec = {};
    std::vector<double> success_vec = {};

    // std::vector<double> time_vec_1 = {};
    // std::vector<double> length_vec_1 = {};
    // std::vector<double> success_vec_1 = {};
    // prm.n = 200;
    // prm.r = 1.0;
    // amp::Path2D path = prm.plan(hw5_environment);
    // std::cout << "is meme?" << "\n";
    // //std::cout << prm.l << "\n";
    // std::cout << path.length() << "\n";
    prm.n = 500;
    prm.r = 1.0;

    //amp::Path2D path = prm.plan(hw2_w1);


    for(int i = 0; i < 100; i++){

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


    std::list<std::vector<double> > data_sets = {time_vec, length_vec, success_vec};
    //std::cout << "geting to vis \n";
    Visualizer box;
    box.makeBoxPlot(data_sets, labels, title, xlabel, ylabel);
    box.showFigures();

    return 0;
}