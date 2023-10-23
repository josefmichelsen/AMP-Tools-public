// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW6.h"
#include "hw/HW2.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

class MyGridCSpace2D : public GridCSpace2D{

    public:

    double grid_side_length;

    MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
        : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max){}

    virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const{

        std::pair<double, double> x_dimensions = this->m_x0_bounds;
        std::pair<double, double> y_dimensions = this->m_x1_bounds;

        std::size_t i = floor((x0 - x_dimensions.first) / grid_side_length);
        std::size_t j = floor((x1 - y_dimensions.first) / grid_side_length);

        std::pair<std::size_t, std::size_t> ans(i,j);

        return ans;
    }
};

class MyPointWaveFrontAlgorithm : public PointWaveFrontAlgorithm{

    public:
    
    // int x0_cells;
    // int x1_cells;
    // double grid_side;
    
    virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment){
        double grid_side = 0.25;
        MyGridCSpace2D* grid = new MyGridCSpace2D((environment.x_max - environment.x_min)/grid_side, (environment.y_max - environment.y_min)/grid_side, environment.x_min, environment.x_max, environment.y_min, environment.y_max);
        
        grid->grid_side_length = 0.25;
        //std::cout << "starting to create discretized workspace\n";
        for(double i = grid_side/2 + environment.x_min; i < environment.x_max; i += grid_side){
            for(double j = grid_side/2 + environment.y_min; j < environment.y_max; j += grid_side){
                bool hit = this->collision(environment, i, j);
                auto[x, y] = grid->getCellFromPoint(i, j);
                grid->operator()(x, y) = hit;
            }
        }
        //std::cout << "returning from constructing discretized workspace\n";
        std::pair<std::size_t, std::size_t> size = grid->size();
        std::unique_ptr<amp::GridCSpace2D> ans;
        ans.reset(grid);
        return ans;

    }

    virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){

        amp::Path2D path;
        path.waypoints.push_back(q_init);
        double current_x = q_goal[0];
        double current_y = q_goal[1];

        std::pair<std::size_t, std::size_t> grid_size = grid_cspace.size();
        auto[x_finish, y_finish] = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
        auto[x_start, y_start] = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

        std::vector<std::vector<int> > wave_grid(grid_size.first, std::vector<int>(grid_size.second));

        for (int i = 0; i < grid_size.first; i++){
            for(int j = 0; j < grid_size.second; j++){
                wave_grid[i][j] = (int)grid_cspace.operator()(i,j);
            }
        }

        wave_grid[x_start][y_start] = 2;
        int highest_num = 2;
        int iter = 0;
        std::pair<double, double> x_bounds = grid_cspace.x0Bounds();
        std::pair<double, double> y_bounds = grid_cspace.x1Bounds();

        while(wave_grid[x_finish][y_finish] == 0){
            for(int j = 0; j < grid_size.second; j++){
                for(int i = 0; i < grid_size.first; i++){
                    if(wave_grid[i][j] == highest_num){
                        
                        if(i + 1 < grid_size.first && wave_grid[i + 1][j] == 0){
                            wave_grid[i + 1][j] = highest_num + 1;
                        }
                        else if(i + 1 == grid_size.first && wave_grid[0][j] == 0){
                            wave_grid[0][j] == highest_num + 1;
                        }

                        if(j + 1 < grid_size.second && wave_grid[i][j + 1] == 0){
                            wave_grid[i][j + 1] = highest_num + 1;
                        }
                        else if(j + 1 == grid_size.first && wave_grid[i][0] == 0){
                            wave_grid[i][0] == highest_num + 1;
                        }

                        if(i - 1 >= 0 && wave_grid[i - 1][j] == 0){
                            wave_grid[i - 1][j] = highest_num + 1;
                        }
                        else if(i - 1 < 0 && wave_grid[grid_size.first - 1][j] == 0){
                            wave_grid[grid_size.first - 1][j] == highest_num + 1;
                        }

                        if(j - 1 >= 0 && wave_grid[i][j - 1] == 0){
                            wave_grid[i][j - 1] = highest_num + 1;
                        }
                        else if(j - 1 < 0 && wave_grid[i][grid_size.second - 1] == 0){
                            wave_grid[i][grid_size.second - 1] == highest_num + 1;
                        }

                    }
                }
            }
            highest_num = highest_num + 1;
            iter = iter + 1;
        }

        int path_x = x_finish;
        int path_y = y_finish;
        int cell_value = wave_grid[path_x][path_y];
        int path_iter = 0;
        //std::cout << "path x " << path_x << " x start " << x_start << " path y " << path_y << " y start " << y_start << "\n";
        while(path_x != x_start || path_y != y_start){
            
            if(path_x + 1 < grid_size.first && wave_grid[path_x + 1][path_y] == cell_value - 1){
                path_x = path_x + 1;
            }
            else if(path_x + 1 == grid_size.first && wave_grid[0][path_y] == cell_value - 1){
                path_x = 0;
            }

            if(path_y + 1 < grid_size.second && wave_grid[path_x][path_y + 1] == cell_value - 1){
                path_y = path_y + 1;
            }
            else if(path_y + 1 == grid_size.second && wave_grid[path_x][0] == cell_value - 1){
                path_y = 0;
            }

            if(path_x - 1 >= 0 && wave_grid[path_x - 1][path_y] == cell_value - 1){
                path_x = path_x - 1;
            }
            else if(path_x - 1 < 0 && wave_grid[grid_size.first - 1][path_y] == cell_value - 1){
                path_x = grid_size.first - 1;
            }

            if(path_y - 1 >= 0 && wave_grid[path_x][path_y - 1] == cell_value - 1){
                path_y = path_y - 1;
            }
            else if(path_y - 1 < 0 && wave_grid[path_x][grid_size.second - 1] == cell_value - 1){
                path_y = grid_size.second - 1;
            }

            cell_value = wave_grid[path_x][path_y];

            double x_to_push = path_x * 0.25 + x_bounds.first + 0.125;
            double y_to_push = path_y * 0.25 + y_bounds.first + 0.125;
            Eigen::Vector2d next_point(x_to_push, y_to_push);
            path.waypoints.push_back(next_point);
            path_iter = path_iter + 1;

        }

        path.waypoints.push_back(q_goal);

        Visualizer test;
        test.makeFigure(grid_cspace, path);
        test.showFigures();

        return path;
    }

    bool collision(const amp::Environment2D& env, double x, double y){
        for (int i = 0; i < env.obstacles.size(); i++){
                
            double min_x = env.obstacles[i].verticesCCW()[0][0];
            double max_x = env.obstacles[i].verticesCCW()[0][0];
            double min_y = env.obstacles[i].verticesCCW()[0][1];
            double max_y = env.obstacles[i].verticesCCW()[0][1];
            
            for (int j = 0; j < env.obstacles[i].verticesCCW().size(); j ++){

                if (env.obstacles[i].verticesCCW()[j][0] < min_x){
                    min_x = env.obstacles[i].verticesCCW()[j][0];
                }
                else if (env.obstacles[i].verticesCCW()[j][0] > max_x){
                    max_x = env.obstacles[i].verticesCCW()[j][0];
                }
                else if (env.obstacles[i].verticesCCW()[j][1] < min_y){
                    min_y = env.obstacles[i].verticesCCW()[j][1];
                }
                else if (env.obstacles[i].verticesCCW()[j][1] > max_y){
                    max_y = env.obstacles[i].verticesCCW()[j][1];
                }

            }            

            if (x >= min_x && x <= max_x && y >= min_y && y <= max_y){

                int cw = 0;
                int ccw = 0;

                for(int k = 0; k < env.obstacles[i].verticesCCW().size(); k++){
                    
                    double x2, y2, cross, x1, y1;

                    if (k == env.obstacles[i].verticesCCW().size() - 1){

                        x2 = env.obstacles[i].verticesCCW()[0][0];
                        y2 = env.obstacles[i].verticesCCW()[0][1];
                    }
                    else{

                        x2 = env.obstacles[i].verticesCCW()[k + 1][0];
                        y2 = env.obstacles[i].verticesCCW()[k + 1][1];

                    }

                    x1 = env.obstacles[i].verticesCCW()[k][0];
                    y1 = env.obstacles[i].verticesCCW()[k][1];

                    cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1);

                    if (cross > 0 ){
                        ccw = ccw + 1;
                    }
                    else if (cross < 0){
                        cw = cw + 1;
                    }
                    else if (cross == 0){
                        //this means they are all on the same line
                        if(x >= std::min(x1,x2) && x <= std::max(x1,x2) && y >= std::min(y1,y2) && y <= std::max(y1,y2)){
                            return true;
                        }
                    }

                }

                if (ccw > 0 && cw > 0 ){
                    return false; // it is not inside
                }
                else{
                    return true;
                }
            }
        }
        return false;
    }
};

class MyGridCSpace2DConstructor : public amp::GridCSpace2DConstructor{

    public:

    virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
        std::vector<double> links_to_inc = manipulator.getLinkLengths();
        links_to_inc[0] = links_to_inc[0] + 0.01;
        links_to_inc[1] = links_to_inc[1] + 0.01;

        MyGridCSpace2D_ForLinks* grid_obj = new MyGridCSpace2D_ForLinks(360.0, 360.0, 0.0, 360.0, 0.0, 360.0);
        grid_obj->env = env;
        grid_obj->link_lengths = links_to_inc;

        bool hit;
        //std::cout << "working to construct grid\n";
    
        for (int i = 0; i < 360; i ++){
            for(int j = 0; j < 360; j++){
                hit = grid_obj->CollisionCheck(i,j);
                grid_obj->operator()(i,j) = hit;
            }
        }

        std::unique_ptr<amp::GridCSpace2D> ans;
        ans.reset(grid_obj);
        //std::cout << "returning grid\n";
        Visualizer test;
        test.makeFigure(env);
        test.showFigures();
        return ans;
    }

};

class MyAstar : public AStar{

    public:

    virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic){

        GraphSearchResult ans;

        std::vector<amp::Node> open_set = {problem.init_node};
        std::vector<amp::Node> closed_set = {};
        std::vector<double> g_score(problem.graph.get()->nodes().size(), 1000);
        std::vector<double> f_score(problem.graph.get()->nodes().size(), 1000);
        std::vector<amp::Node> came_from(problem.graph.get()->nodes().size());
        f_score[0] = heuristic.operator()(problem.init_node);
        g_score[0] = 0;
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
                std::cout << "number of iterations to get to goal is " << number_of_iterations << "\n";
                return ans;
            }
            
            open_set.erase(open_set.begin() + next_node_index);

            std::vector<amp::Node> children_of_current_node = problem.graph.get()->children(current_node);
            std::vector<double> edge_weights_from_current_node = problem.graph.get()->outgoingEdges(current_node);            

            for(int i = 0; i < children_of_current_node.size(); i++){
                
                double tentative_gscore = g_score[current_node] + edge_weights_from_current_node[i];
                //std::cout << "I am at node  " << current_node << "  child of current node  " << children_of_current_node[i] << " tentative g score is g score current node " << g_score[current_node]<< "  plus  " <<edge_weights_from_current_node[i]<< "\n";
                if(tentative_gscore < g_score[children_of_current_node[i]]){
                    came_from[children_of_current_node[i]] = current_node;
                    g_score[children_of_current_node[i]] = tentative_gscore;
                    f_score[children_of_current_node[i]] = tentative_gscore + heuristic.operator()(children_of_current_node[i]);
                    //std::cout << "heuristic value is " <<  heuristic.operator()(children_of_current_node[i]) << "\n";
                    
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

        ans.success = false;
        return ans;
    }
};


class MyPointMotionPlanner2D : public PointMotionPlanner2D{

    public:

    virtual amp::Path2D plan(const amp::Problem2D& problem){
        //std::cout << "inside point motion plan\n";
        std::size_t x0_cells = 100; //This gets returned when calling the this->size() in the class
        std::size_t x1_cells = 100; //This gets returned when calling the this->size() in the class
        double x0_min = problem.x_min;
        double x0_max = problem.x_max;
        double x1_min = problem.y_min;
        double x1_max = problem.y_max;
        
        MyGridCSpace2D grid(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);

        grid.grid_side_length = 0.25;

        MyPointWaveFrontAlgorithm wave;

        // wave.x0_cells = (int)x0_cells;
        // wave.x1_cells = (int)x1_cells;
        // wave.grid_side = 0.25;

        std::unique_ptr<amp::GridCSpace2D> c_space = wave.constructDiscretizedWorkspace(problem);

        //std::pair<std::size_t, std::size_t> test_c_size = c_space.get()->size();

        amp::Path2D ex_1_path = wave.planInCSpace(problem.q_init, problem.q_goal, *c_space);
        return ex_1_path;

    }

};

class MyManipulatorWaveFrontAlgorithm : public ManipulatorWaveFrontAlgorithm{

    public:

    MyManipulatorWaveFrontAlgorithm(const std::shared_ptr<GridCSpace2DConstructor>& c_space_constructor)
        : ManipulatorWaveFrontAlgorithm(c_space_constructor)
        //, m_c_space_constructor(c_space_constructor) 
        {}
    
    virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
        
        amp::Path2D path;
        amp::Path2D path_test;
        path.waypoints.push_back(q_init);
        path_test.waypoints.push_back(q_init * 180/3.141592653589793238463);
        // double current_x = q_goal[0];
        // double current_y = q_goal[1];

        std::pair<std::size_t, std::size_t> grid_size = grid_cspace.size();
        // auto[x_finish, y_finish] = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
        // auto[x_start, y_start] = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
        int x_finish = q_init[0] * 180/3.141592653589793238463;
        int y_finish = q_init[1] * 180/3.141592653589793238463;
        int x_start = q_goal[0] * 180/3.141592653589793238463;
        int y_start = q_goal[1] * 180/3.141592653589793238463;
        // std::cout << "goal x " << x_start << " goal y " << y_start << "\n";
        // std::cout << "init x " << x_finish << " init y " << y_finish << "\n";
        // std::cout << "true goal x " << q_goal[0] << " true goal y " << q_goal[1] << "\n";
        // std::cout << "true init x " << q_init[0] << " true init y " << q_init[1] << "\n";

        std::vector<std::vector<int> > wave_grid(grid_size.first, std::vector<int>(grid_size.second));

        for (int i = 0; i < grid_size.first; i++){
            for(int j = 0; j < grid_size.second; j++){
                wave_grid[i][j] = (int)grid_cspace.operator()(i,j);
            }
        }
        
        wave_grid[x_start][y_start] = 2;

        // for (int j = grid_size.second-1; j < grid_size.second; j--){
        //     for(int i = 0; i < grid_size.first; i++){

        //         std::cout<< wave_grid[i][j];

        //     }
        //     std::cout <<"\n";

        // }

        int highest_num = 2;
        int iter = 0;
        std::pair<double, double> x_bounds = grid_cspace.x0Bounds();
        std::pair<double, double> y_bounds = grid_cspace.x1Bounds();

        Visualizer test_broke;
        test_broke.makeFigure(grid_cspace);
        test_broke.showFigures();

        while(wave_grid[x_finish][y_finish] == 0 && iter < 5000){
            for(int j = 0; j < grid_size.second; j++){
                for(int i = 0; i < grid_size.first; i++){
                    if(wave_grid[i][j] == highest_num){

                        if(i + 1 < grid_size.first && wave_grid[i + 1][j] == 0){
                            wave_grid[i + 1][j] = highest_num + 1;
                        }
                        else if(i + 1 == grid_size.first && wave_grid[0][j] == 0){
                            wave_grid[0][j] = highest_num + 1;
                        }

                        if(j + 1 < grid_size.second && wave_grid[i][j + 1] == 0){
                            wave_grid[i][j + 1] = highest_num + 1;
                        }
                        else if(j + 1 == grid_size.first && wave_grid[i][0] == 0){
                            wave_grid[i][0] = highest_num + 1;
                        }

                        if(i - 1 >= 0 && wave_grid[i - 1][j] == 0){
                            wave_grid[i - 1][j] = highest_num + 1;
                        }
                        else if(i - 1 < 0 && wave_grid[grid_size.first - 1][j] == 0){
                            wave_grid[grid_size.first - 1][j] = highest_num + 1;
                            // std::cout << "i - 1 < 0     x  " << i << "  y  " << j << "\n";
                            // std::cout << "other edge with coordinates " << grid_size.first - 1 << " " << j << "  is  " << wave_grid[grid_size.first - 1][j] << "\n";
                            // sleep(3);
                        }

                        if(j - 1 >= 0 && wave_grid[i][j - 1] == 0){
                            wave_grid[i][j - 1] = highest_num + 1;
                        }
                        else if(j - 1 < 0 && wave_grid[i][grid_size.second - 1] == 0){
                            wave_grid[i][grid_size.second - 1] = highest_num + 1;
                            // std::cout << "j - 1 < 0     x  " << i << "  y  " << j << "\n";
                            // std::cout << "other edge with coordinates " << i << " " << grid_size.second - 1 << "  is  " << wave_grid[i][grid_size.second - 1] << "\n";
                            // sleep(3);
                        }

                    }
                }
            }
            highest_num = highest_num + 1;
            iter = iter + 1;
            //std::cout << "num iterations in while loop building wave grid numbers " << iter << "\n";
        }
        //sleep(5);

        int path_x = x_finish;
        int path_y = y_finish;
        int cell_value = wave_grid[path_x][path_y];
        int path_iter = 0;
        //std::cout << "init x " << path_x << " goal x " << x_start << " init y  " << path_y << " goal y " << y_start << "\n";
        while((path_x != x_start || path_y != y_start) && path_iter < 5000){
            
            if(path_x + 1 < grid_size.first && wave_grid[path_x + 1][path_y] == cell_value - 1){
                path_x = path_x + 1;
            }
            else if(path_x + 1 == grid_size.first && wave_grid[0][path_y] == cell_value - 1){
                path_x = 0;
            }

            if(path_y + 1 < grid_size.second && wave_grid[path_x][path_y + 1] == cell_value - 1){
                path_y = path_y + 1;
            }
            else if(path_y + 1 == grid_size.second && wave_grid[path_x][0] == cell_value - 1){
                path_y = 0;
            }

            if(path_x - 1 >= 0 && wave_grid[path_x - 1][path_y] == cell_value - 1){
                path_x = path_x - 1;
            }
            else if(path_x - 1 < 0 && wave_grid[grid_size.first - 1][path_y] == cell_value - 1){
                // std::cout<< "trying to move left\n";
                // sleep(5);
                path_x = grid_size.first - 1;
            }

            if(path_y - 1 >= 0 && wave_grid[path_x][path_y - 1] == cell_value - 1){
                path_y = path_y - 1;
            }
            else if(path_y - 1 < 0 && wave_grid[path_x][grid_size.second - 1] == cell_value - 1){
                // std::cout<< "trying to move down\n";
                // sleep(5);
                path_y = grid_size.second - 1;
            }

            cell_value = wave_grid[path_x][path_y];

            double x_to_push = path_x + x_bounds.first - 0.01;
            double y_to_push = path_y + y_bounds.first - 0.01;
            Eigen::Vector2d next_point_test(x_to_push, y_to_push);
            if(x_to_push * 3.141592653589793238463/180 < 0){
                x_to_push = 0;
            }
            if( y_to_push * 3.141592653589793238463/180){
                y_to_push = 0;
            }
            Eigen::Vector2d next_point(x_to_push * 3.141592653589793238463/180, y_to_push * 3.141592653589793238463/180);
            path.waypoints.push_back(next_point);
            path_test.waypoints.push_back(next_point_test);
            path_iter = path_iter + 1;
            //std::cout << "num iterations in while loop " << path_iter << "\n";

        }
        
        //std::cout << "number of iterations = " << iter << "\n";
        path.waypoints.push_back(q_goal);

        unwrapPath(path, Eigen::Vector2d(0,0), Eigen::Vector2d(2*3.141592653589793238463, 2*3.141592653589793238463));
        path_test.waypoints.push_back(q_goal* 180/3.141592653589793238463);
        Visualizer test;
        test.makeFigure(grid_cspace, path_test);
        test.showFigures();
        // std::cout << "size of path is " << path.waypoints.size() << "\n";
        // for(int i = 0; i < path.waypoints.size(); i++){
        //     std::cout<< path.waypoints[i][0] << "  " << path.waypoints[i][1] << "\n";
        // }
        return path;


    }
};


class MyLinkManipulatorMotionPlanner2D : public LinkManipulatorMotionPlanner2D{
    public:
    typedef LinkManipulator2D AgentType;
    virtual amp::ManipulatorTrajectory2Link plan(const AgentType& link_manipulator_agent, const amp::Problem2D& problem){

        //MyLinkManipulator2D mylink(link_manipulator_agent.getLinkLengths());
        //std::shared_ptr<amp::Graph<double> > graph = std::make_shared<amp::Graph<double> >();
        //std::cout << "in the plan\n";
        MyGridCSpace2DConstructor* Myc_space_constructor = new MyGridCSpace2DConstructor();
        std::shared_ptr<GridCSpace2DConstructor> c_space_constructor;
        c_space_constructor.reset(Myc_space_constructor);
        //std::cout << "got vectors\n";
        MyManipulatorWaveFrontAlgorithm algo(c_space_constructor);
        amp::ManipulatorTrajectory2Link ans = algo.plan(link_manipulator_agent, problem);
        
        return ans;
    }
    

};


int main(int argc, char** argv) {    
    //Problem2D ex_1 = HW2::getWorkspace1();
    //Problem2D ex_1 = HW2::getWorkspace2();

    // MyPointMotionPlanner2D test_prob_1;
    // amp::Path2D test_path = test_prob_1.plan(ex_1);
    // double path_length = 0;
    // for(int i = 0; i < test_path.waypoints.size()-1; i++){
    //     path_length = path_length + sqrt(pow(test_path.waypoints[i][0] - test_path.waypoints[i+1][0],2) + pow(test_path.waypoints[i][1] - test_path.waypoints[i+1][1],2));
    // }
    // std::cout << "path length is " << path_length << "\n";
    // sleep(3);
    // Visualizer c_space_path;
    // c_space_path.makeFigure(ex_1, test_path);
    // c_space_path.showFigures();

    //Obstacle2D problem1 =  HW4::getEx1TriangleObstacle();

    //bool pass_ex_1 = HW6::checkPointAgentPlan(test_path, ex_1, true);
    //std::cout << "Did I pass exercise 1 " << pass_ex_1 << "\n";

    //LINK BOI
    // std::vector<Eigen::Vector2d> o1 = {{-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1}};
    // std::vector<Eigen::Vector2d> o2 = {{-2.0,-2.0},{-2.0,-1.8},{2.0,-1.8},{2.0,-2.0}};
    // std::vector<Eigen::Vector2d> o1 = {{-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1}};
    // std::vector<Eigen::Vector2d> o2 = {{-2.0,-0.5},{-2.0,-0.3},{2.0,-0.3},{2.0,-0.5}};

    // Obstacle2D obs1(o1);
    // Obstacle2D obs2(o2);
    // std::vector<Obstacle2D> obs_to_pass = {obs1,obs2};
    // amp::Environment2D env;
    // env.obstacles = obs_to_pass;

    // std::size_t x0_cells_link_grid = 360; //This gets returned when calling the this->size() in the class
    // std::size_t x1_cells_link_grid = 360; //This gets returned when calling the this->size() in the class
    // double x0_min_link_grid = 0.0;
    // double x0_max_link_grid = 360.0;
    // double x1_min_link_grid = 0.0;
    // double x1_max_link_grid = 360.0;

    // MyGridCSpace2DConstructor grid_constructor;

    //MyGridCSpace2D_ForLinks link_grid(x0_cells_link_grid, x1_cells_link_grid, x0_min_link_grid, x0_max_link_grid, x1_min_link_grid, x1_max_link_grid);
    //amp::LinkManipulator2D link;
    
    // link_grid.env = env;
    // link_grid.link_lengths = {1.0,1.0};

    //std::unique_ptr<amp::GridCSpace2D> grid_c_space = grid_constructor.construct(link, env);

    



    // Visualizer link;
    // link.makeFigure(env);
    // link.makeFigure(link_grid);
    // link.showFigures();

    //MyManipulatorWaveFrontAlgorithm* I_hate_this_class = new MyManipulatorWaveFrontAlgorithm();
    // MyLinkManipulator2D* link_to_pass = new MyLinkManipulator2D({1.0,1.0});
    // std::unique_ptr<amp::LinkManipulator2D> this_is_so_stupid;
    // this_is_so_stupid.reset(link_to_pass);
    // amp::Problem2D problem_link_traj = HW6::getHW4Problem1(); 
    // //amp::ManipulatorTrajectory2Link traj = I_hate_this_class.plan(*this_is_so_stupid, problem_link_traj);

    // MyLinkManipulatorMotionPlanner2D link_motion_plan;
    // amp::ManipulatorTrajectory2Link traj = link_motion_plan.plan(*this_is_so_stupid, problem_link_traj);
    // //std::cout<< "got back from the traj plan\n";
    
    

    // bool pass_link = amp::HW6::checkLinkManipulatorPlan(traj, *this_is_so_stupid, problem_link_traj, true);
    // std::cout << "did I pass link check " << pass_link << "\n";
    // Visualizer link_prob;
    // link_prob.makeFigure(problem_link_traj, *this_is_so_stupid, traj );
    // link_prob.makeFigure(problem_link_traj);
    // link_prob.showFigures();



    std::shared_ptr<amp::Graph<double> > graph = std::make_shared<amp::Graph<double> >();
    graph.get()->connect(0, 1, 3);
    graph.get()->connect(0, 2, 1);
    graph.get()->connect(0, 3, 3);
    graph.get()->connect(0, 4, 1);
    graph.get()->connect(0, 5, 3);
    graph.get()->connect(1, 6, 1);
    graph.get()->connect(1, 7, 3);
    graph.get()->connect(2, 1, 0);
    graph.get()->connect(2, 7, 3);
    graph.get()->connect(2, 8, 2);
    graph.get()->connect(2, 9, 1);
    graph.get()->connect(3, 9, 1);
    graph.get()->connect(4, 9, 1);
    graph.get()->connect(4, 10, 2);
    graph.get()->connect(4, 11, 3);
    graph.get()->connect(4, 5, 2);
    graph.get()->connect(5, 11, 1);
    graph.get()->connect(5, 12, 1);
    graph.get()->connect(6, 7, 1);
    graph.get()->connect(7, 13, 1);
    graph.get()->connect(8, 13, 3);
    graph.get()->connect(9, 13, 3);
    graph.get()->connect(10, 13, 3);
    graph.get()->connect(11, 13, 1);
    graph.get()->connect(12, 11, 3);

    amp::LookupSearchHeuristic test_heuristic = HW6::getEx3Heuristic();
    //amp::LookupSearchHeuristic djikstra = amp::operator(); 

    MyAstar astar_search;
    ShortestPathProblem path_to_pass;
    path_to_pass.graph = graph;
    path_to_pass.goal_node = 13;
    path_to_pass.init_node = 0;
    amp::AStar::GraphSearchResult graph_path = astar_search.search(path_to_pass,amp::SearchHeuristic());
    std::cout << "path cost " << graph_path.path_cost << "\n";
    for(auto i : graph_path.node_path){
        std::cout << i << "\n";
    }
    
    bool did_i_pass_graph = HW6::checkGraphSearchResult(graph_path, path_to_pass, amp::SearchHeuristic(), true);
    
    // MyPointMotionPlanner2D* my_motion_plan_grade = new MyPointMotionPlanner2D();
    // MyLinkManipulatorMotionPlanner2D* my_link_motion_plan_grade = new MyLinkManipulatorMotionPlanner2D();
    // MyAstar* my_astar_grade = new MyAstar();

    // std::unique_ptr<amp::PointMotionPlanner2D> point_motion_to_pass;
    // std::unique_ptr<amp::LinkManipulatorMotionPlanner2D> link_motion_to_pass;
    // std::unique_ptr<amp::AStar> astar_to_pass;

    // point_motion_to_pass.reset(my_motion_plan_grade);
    // link_motion_to_pass.reset(my_link_motion_plan_grade);
    // astar_to_pass.reset(my_astar_grade);

    // MyGridCSpace2DConstructor* Myc_space_constructor = new MyGridCSpace2DConstructor();
    // std::shared_ptr<GridCSpace2DConstructor> c_space_constructor;
    // c_space_constructor.reset(Myc_space_constructor);
    //amp::HW6::grade<MyPointWaveFrontAlgorithm, MyManipulatorWaveFrontAlgorithm, MyAstar>("jomi7243@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(c_space_constructor), std::make_tuple());
    

    return 0;
}