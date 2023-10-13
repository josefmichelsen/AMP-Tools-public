// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
//#include <Eigen/Core>
// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

class translating_robot{
    
    public:

    std::vector<std::vector<double>> calculate_c_obst(Obstacle2D problem1, amp::Polygon robot){
        std::vector<std::vector<double>> v_and_ang_robot, v_and_ang_obst;
        double edge_x, edge_y, dot, det, angle;
        std::vector<double> cur_edge;

        for(int i = 0; i < problem1.verticesCCW().size(); i++){
            if (i == problem1.verticesCCW().size() -1){
                edge_x = problem1.verticesCCW()[0][0] - problem1.verticesCCW()[i][0];
                edge_y = problem1.verticesCCW()[0][1] - problem1.verticesCCW()[i][1];
                angle = std::atan2(edge_y,edge_x) * 180.0/3.141592653589793238463 - 90;  //subtracting 90 to get the normal to that vec
                if(angle < 0){
                    angle = angle + 360;
                }
                else if (angle == 360){
                    angle = 0;
                }
                cur_edge = {problem1.verticesCCW()[i][0], problem1.verticesCCW()[i][1], angle};
                v_and_ang_obst.push_back(cur_edge);
            }
            else{
                edge_x = problem1.verticesCCW()[i+1][0] - problem1.verticesCCW()[i][0];
                edge_y = problem1.verticesCCW()[i+1][1] - problem1.verticesCCW()[i][1];
                angle = std::atan2(edge_y,edge_x) * 180.0/3.141592653589793238463 - 90;//subtracting 90 to get the normal to that vec
                if(angle < 0){
                    angle = angle + 360;
                }
                else if (angle == 360){
                    angle = 0;
                }
                cur_edge = {problem1.verticesCCW()[i][0], problem1.verticesCCW()[i][1], angle}; 
                v_and_ang_obst.push_back(cur_edge);
            }

        }

        for(int i = 0; i < robot.verticesCCW().size(); i++){
            if (i == robot.verticesCCW().size() -1){
                edge_x = robot.verticesCCW()[0][0] - robot.verticesCCW()[i][0];
                edge_y = robot.verticesCCW()[0][1] - robot.verticesCCW()[i][1];
                angle = std::atan2(edge_y,edge_x) * 180.0/3.141592653589793238463 + 90; //adding 90 to get the normal to that vec
                if(angle < 0){
                    angle = angle + 360;
                }
                else if (angle == 360){
                    angle = 0;
                }
                cur_edge = {robot.verticesCCW()[i][0], robot.verticesCCW()[i][1], angle};
                v_and_ang_robot.push_back(cur_edge);
            }
            else{
                edge_x = robot.verticesCCW()[i+1][0] - robot.verticesCCW()[i][0];
                edge_y = robot.verticesCCW()[i+1][1] - robot.verticesCCW()[i][1];
                angle = std::atan2(edge_y,edge_x) * 180.0/3.141592653589793238463 + 90; // adding 90 to get normal to that vec
                if(angle < 0){
                    angle = angle + 360;
                }
                else if (angle == 360){
                    angle = 0;
                }
                cur_edge = {robot.verticesCCW()[i][0], robot.verticesCCW()[i][1], angle}; 
                v_and_ang_robot.push_back(cur_edge);
            }

        }

        std::vector<double> all_angles;
        for (int i = 0; i < v_and_ang_obst.size() + v_and_ang_robot.size(); i++){
            if (i <= 2){
                all_angles.push_back(v_and_ang_obst[i][2]);
            }
            else{
                all_angles.push_back(v_and_ang_robot[i-3][2]);
            }
        }

        std::sort(all_angles.begin(),all_angles.end());

        std::vector<double> a, b; //a is robot, b is obst
        b = {v_and_ang_obst[0][0], v_and_ang_obst[0][1]};
        a = {v_and_ang_robot[0][0], v_and_ang_robot[0][1]};
        std::vector<std::vector<double>> c_space_vertices = {{a[0] + b[0], a[1] + b[1]}};
        std::vector<double> next_c_space_point, old_point;

        int obst_iter = 0;
        int robot_iter = 0;

        for (int i = 0; i < all_angles.size() -1 ; i++ ){
            for (int j = 0; j < v_and_ang_obst.size(); j++){

                if(all_angles[i] == v_and_ang_obst[j][2]){
                    obst_iter = obst_iter + 1;
                    if(obst_iter == v_and_ang_obst.size()){
                        obst_iter = 0;
                    }
                    b = {v_and_ang_obst[obst_iter][0], v_and_ang_obst[obst_iter][1]};
                    next_c_space_point = {a[0] + b[0], a[1] + b[1]};
                    c_space_vertices.push_back(next_c_space_point);
                    continue;

                }
                else if (all_angles[i] == v_and_ang_robot[j][2]){
                    robot_iter = robot_iter + 1;
                    if(robot_iter == v_and_ang_robot.size()){
                        robot_iter = 0;
                    }
                    a = {v_and_ang_robot[robot_iter][0], v_and_ang_robot[robot_iter][1]};
                    next_c_space_point = {a[0] + b[0], a[1] + b[1]};
                    c_space_vertices.push_back(next_c_space_point);
                    continue;

                }

            }
        }
        return c_space_vertices;
    }

};


class link_solver : public LinkManipulator2D{

    public:

    link_solver(const std::vector<double>& link_lengths)
    :  LinkManipulator2D(link_lengths){}

    link_solver()
    :  LinkManipulator2D(){}

    Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const{

        double x_sum, y_sum, angle_sum;

        const std::vector<double> links =  getLinkLengths();
        const Eigen::Vector2d& base_loc = getBaseLocation();

        std::vector<double> temp;

        x_sum = base_loc[0];
        y_sum = base_loc[1];
        angle_sum = 0;

        for (int i = 0; i < joint_index; i++){

            angle_sum = angle_sum + state[i];

            x_sum = x_sum + links[i] * cos(angle_sum);
            y_sum = y_sum + links[i] * sin(angle_sum);
            
        }   
        Eigen::Vector2d ans(x_sum,y_sum);

        return ans;

    }

    ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
        
        double t3 = 2 * 3.141592653589793238463/4; // assume that this is theta 3 because at the moment it is a 3DOF problem in a 2D workspace meaning we have infinite
        //potential answers so we can arbitrarily pick the final angle.

        const std::vector<double> links = getLinkLengths();
        
        double l1 = links[0];
        double l2 = links[1];
        double p3_x, p3_y;
        if(links.size() == 3){
            double l3 = links[2];
            p3_x = end_effector_location[0] - l3 * cos(t3);
            p3_y = end_effector_location[1] - l3 * sin(t3);
        }
        else{
            p3_x = end_effector_location[0];
            p3_y = end_effector_location[1];
        }

        double t2 = acos( (1/(2 * l1 * l2)) * ( (pow(p3_x,2) + pow(p3_y,2)) - (pow(l1,2) + pow(l2,2)) ) );

        double t1 = asin( (1/(pow(p3_x,2) + pow(p3_y,2))) * (p3_y * (l1 + l2 * cos(t2)) - p3_x * l2 * sqrt(1 - pow(cos(t2),2)) ) );
     
        ManipulatorState ans;
        if(links.size() == 3){
            ans = {t1,t2,t3 - t2 - t1};
        }
        else{
            ans = {t1,t2};
        }
        
        return ans;

    }


};


class config : public GridCSpace2D{

    public: 

        Environment2D env;
        std::vector<double> link_lengths;


        config(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
        : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max){}

        int orientation(double x1, double y1, double x2, double y2, double x3, double y3) const {

            float turn = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
            if (turn > 0){ //clockwise
                return 1;
            }
            else if (turn < 0){//counter clockwise 
                return 2;
            }
            else return 0; //showing they are colinear 

        }

        bool inCollision(double x0, double x1) const{
            
            link_solver link_obj(link_lengths);
            
            const std::vector<double> l = link_obj.getLinkLengths();
            
            const ManipulatorState state = {x0 * 3.141592653589793238463/180.0 , x1 * 3.141592653589793238463/180.0};
            uint32_t joint_index = 2;
            Eigen::Vector2d base = link_obj.getJointLocation(state, 0);
            Eigen::Vector2d point1 = link_obj.getJointLocation(state, 1);
            Eigen::Vector2d end_point = link_obj.getJointLocation(state, joint_index);

            double x_base = base[0];
            double y_base = base[1];
            double x_f_link = point1[0];
            double y_f_link = point1[1];
            double x = end_point[0];
            double y = end_point[1];

            double min_x_links = std::min(std::min(x_base, x_f_link), x);
            double min_y_links = std::min(std::min(y_base, y_f_link), y);

            double min_x, max_x, min_y, max_y;

            for (int i = 0; i < env.obstacles.size(); i++){
                
                min_x = env.obstacles[i].verticesCCW()[0][0];
                max_x = env.obstacles[i].verticesCCW()[0][0];
                min_y = env.obstacles[i].verticesCCW()[0][1];
                max_y = env.obstacles[i].verticesCCW()[0][1];
                
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
                
                if( min_x_links < max_x && min_y_links < max_y){
                    for(int iter = 0; iter < env.obstacles[i].verticesCCW().size(); iter++){
                        double vertex_x = env.obstacles[i].verticesCCW()[iter][0];
                        double vertex_y = env.obstacles[i].verticesCCW()[iter][1];
                        double next_vertex_x, next_vertex_y;
                        //std::cout << "in here lo0p is at " << iter << " size of vertices is " << env.obstacles[i].verticesCCW().size() << "\n";
                        
                        if(iter == env.obstacles[i].verticesCCW().size() - 1){
                            next_vertex_x = env.obstacles[i].verticesCCW()[0][0];
                            next_vertex_y = env.obstacles[i].verticesCCW()[0][1];
                        }
                        else{
                            next_vertex_x = env.obstacles[i].verticesCCW()[iter + 1][0];
                            next_vertex_y = env.obstacles[i].verticesCCW()[iter + 1][1];
                        }

                        int orient1 = this->orientation(x_base, y_base, x_f_link, y_f_link, vertex_x, vertex_y);
                        int orient2 = this->orientation(x_base, y_base, x_f_link, y_f_link, next_vertex_x, next_vertex_y);
                        int orient3 = this->orientation(vertex_x, vertex_y, next_vertex_x, next_vertex_y, x_base, y_base);
                        int orient4 = this->orientation(vertex_x, vertex_y, next_vertex_x, next_vertex_y, x_f_link, y_f_link);

                        int orient5 = this->orientation(x_f_link, y_f_link, x, y, vertex_x, vertex_y);
                        int orient6 = this->orientation(x_f_link, y_f_link, x, y, next_vertex_x, next_vertex_y);
                        int orient7 = this->orientation(vertex_x, vertex_y, next_vertex_x, next_vertex_y, x_f_link, y_f_link);
                        int orient8 = this->orientation(vertex_x, vertex_y, next_vertex_x, next_vertex_y, x, y);

                        if ((orient1 != orient2 && orient3 != orient4) || (orient5 != orient6 && orient7 != orient8)){
                            return true;            
                        }                    
                    }
                }
                


                if (x >= min_x && x <= max_x && y >= min_y && y <= max_y){ //this is where we need collision logic for end effector

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

class grader_class : public GridCSpace2DConstructor{

    public:

    std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){

        std::size_t x0 = 360;
        std::size_t x1 = 360;
        config get_col_c_space( x0, x1, 0.0, 360.0, 0.0, 360.0);
        get_col_c_space.env = env;
        get_col_c_space.link_lengths = manipulator.getLinkLengths();

        bool hit;
        
        for (int i = 0; i < 360; i ++){
            for(int j = 0; j < 360; j++){
                hit = get_col_c_space.inCollision(i,j);
                get_col_c_space.operator()(i,j) = hit;

            }

        }

        std::unique_ptr<amp::GridCSpace2D> grid;
        grid.reset(&get_col_c_space);
        return grid;

    }

};


int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    //amp::RNG::seed(amp::RNG::randiUnbounded());
    
    std::vector<Eigen::Vector2d> O1 = {{-1,-2},{0,-2},{0,0}}; // pre set the vertices to be negative of the original
    amp::Polygon robot(O1);
    Obstacle2D problem1 =  HW4::getEx1TriangleObstacle();

    translating_robot solving_c_space;

    std::vector<std::vector<double>> vertices_c_space = solving_c_space.calculate_c_obst(problem1, robot);

    std::vector<amp::Polygon> c_space_polygons = {};
    //std::vector<double> thetas = {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0, 300.0, 330.0};
    double pi = 3.141592653589793238463;
    std::vector<double> thetas = {0.0, pi/6, 2*pi/6, 3*pi/6, 4*pi/6, 5*pi/6, 6*pi/6, 7*pi/6, 8*pi/6, 9*pi/6, 10*pi/3, 11*pi/3};

    for (int i = 0; i < thetas.size(); i++){

        std::vector<Eigen::Vector2d> robot_vertices = {{-1,-2},{0,-2},{0,0}};
        
        for(int j = 0; j < robot_vertices.size(); j++){
            
            double x_before = robot_vertices[j][0];
            double y_before = robot_vertices[j][1];
            robot_vertices[j][0] = (x_before * cos(thetas[i])) - (y_before * sin(thetas[i])); 
            robot_vertices[j][1] = (y_before * cos(thetas[i])) + (x_before * sin(thetas[i])); 
        }

        amp::Polygon rotated_robot(robot_vertices);
        std::vector<std::vector<double>> vertices_c_space = solving_c_space.calculate_c_obst(problem1,rotated_robot);
        std::vector<Eigen::Vector2d> hold;
        for(int k = 0; k < vertices_c_space.size(); k++){
            Eigen::Vector2d temp(vertices_c_space[k][0],vertices_c_space[k][1]);
            hold.push_back(temp);
        }
        amp::Polygon c_obst(hold);
        c_space_polygons.push_back(c_obst);
    }
    
    // Visualizer first;
    // first.makeFigure(c_space_polygons, thetas);
    // first.showFigures();


    const std::vector<double> link_lengths = {1.0,0.5,1.0};
    uint32_t joint_index = 2;
    const link_solver link_robot(link_lengths);
    Eigen::Vector2d end(2.0,0.0);
    //Eigen::Vector2d sol = link_robot.getJointLocation(thetas, joint_index);
    ManipulatorState sol = link_robot.getConfigurationFromIK(end);
    
    
    //const amp::ManipulatorState thetas2 = {3.141592653589793238463/6,3.141592653589793238463/3,7*3.141592653589793238463/4};
    //const std::vector<double> thetas2 = {90.0,90.0,90.0};
    // Visualizer link_vis;
    // link_vis.makeFigure(link_robot, sol);
    // link_vis.showFigures();

    //std::cout << "position from new function x " << sol[0] << " y " << sol[1] << "\n";

    const Eigen::Vector2d end_effector_location(0.0,4.0);

    //ManipulatorState test2 =  link_robot.getConfigurationFromIK(end_effector_location);

    // std::vector<Eigen::Vector2d> o1 = {{-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1}};
    // std::vector<Eigen::Vector2d> o2 = {{-2.0,-2.0},{-2.0,-1.8},{2.0,-1.8},{2.0,-2.0}};
    // Obstacle2D obs1(o1);
    // Obstacle2D obs2(o2);
    // std::vector<Obstacle2D> obs_to_pass = {obs1,obs2};

    // std::vector<Eigen::Vector2d> triang = {{0.25,0.25},{0.0,0.75},{-0.25,0.25}};
    // Obstacle2D obs_triang(triang);
    // std::vector<Obstacle2D> obs_to_pass = {triang};

    std::vector<Eigen::Vector2d> o1 = {{-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1}};
    std::vector<Eigen::Vector2d> o2 = {{-2.0,-0.5},{-2.0,-0.3},{2,-0.3},{2.0,-0.5}};
    Obstacle2D obs1(o1);
    Obstacle2D obs2(o2);
    std::vector<Obstacle2D> obs_to_pass = {obs1,obs2};

    std::size_t x_config_size = 360;
    std::size_t y_config_size = 360;
    double lower_bound_c_space_1st_dim = 0.0;
    double upper_bound_c_space_1st_dim = 360.0;
    double lower_bound_c_space_2st_dim = 0.0;
    double upper_bound_c_space_2st_dim = 360.0;

    //config problem_3_a(lower_bound_c_space_1st_dim, upper_bound_c_space_1st_dim, lower_bound_c_space_2st_dim, upper_bound_c_space_2st_dim, x_config_size, y_config_size);
    config problem_3_b(x_config_size, y_config_size, lower_bound_c_space_1st_dim, upper_bound_c_space_1st_dim, lower_bound_c_space_2st_dim, upper_bound_c_space_2st_dim);
    //config problem_3_c(lower_bound_c_space_1st_dim, upper_bound_c_space_1st_dim, lower_bound_c_space_2st_dim, upper_bound_c_space_2st_dim, x_config_size, y_config_size);
    
    problem_3_b.env.obstacles = obs_to_pass; 
    problem_3_b.env.x_min = -3.0;
    problem_3_b.env.x_max = 3.0;
    problem_3_b.env.y_min = -3.0;
    problem_3_b.env.y_max = 3.0;

    // for (int i = 0; i < problem_3_b.env.obstacles[0].verticesCCW().size(); i++){
    //    //std::cout << "vertex " << problem_3_b.env.obstacles[0].verticesCCW()[i][0] << " " << problem_3_b.env.obstacles[0].verticesCCW()[i][1] << "\n";
    // }

    std::vector<double> link_lengths_to_set = {1.0,1.0}; 
    problem_3_b.link_lengths = link_lengths_to_set;

    bool hit;

    //problem_3_b[0][0] = 3.0;
    int iter_i = 0;
    int iter_j = 0;

    for (int i = (int)lower_bound_c_space_1st_dim; i < (int)upper_bound_c_space_1st_dim; i++){
        iter_j = 0;
        for (int j = (int)lower_bound_c_space_2st_dim; j < (int)upper_bound_c_space_2st_dim; j++){

            hit = problem_3_b.inCollision(i,j);
            problem_3_b.operator()(iter_i,iter_j) = hit;
            iter_j = iter_j + 1;

        }
        iter_i = iter_i + 1;

    }

    //bool testing_hit = problem_3_b.inCollision(270.0,270.0);
   //std::cout << testing_hit << "\n";

    // Visualizer plots;
    // plots.makeFigure(problem_3_b);
    // plots.makeFigure(problem_3_b.env);
    // plots.showFigures();




    //std::cout<<"getting to testing\n";
    //bool testing = problem_3_b.inCollision(2.0,1.0);
    //double x_min,y_min,x_max,y_max;
    //std::cout << "am I in an obstacle? " << testing << "\n";
    // x_min = 0;
    // y_min = 0;
    // x_max = 10;
    // y_max = 10;
    // std::size_t x_cell, y_cell = 10;
    // GridCSpace2D bool_array(x_cell, y_cell, x_min, x_max, y_min, y_max);

    //std::cout << bool_array[0][0] << "\n";
    
    
    // Visualizer::makeFigure(link_robot, test2);
    // Visualizer::showFigures();
    //const std::vector<double> link_lengths_grade = {};
    //const ManipulatorState thetas = {45.0,90.0,23.6};
    //uint32_t joint_index = 2;
    // std::vector<Eigen::Vector2d> o1_grade = {{-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1}};
    // std::vector<Eigen::Vector2d> o2_grade = {{-2,-2},{-2,-1.8},{2,-1.8},{2,-2}};
    // Obstacle2D obs1_grade(o1_grade);
    // Obstacle2D obs2_grade(o2_grade);
    // std::vector<Obstacle2D> obs_to_pass_grade = {obs1_grade,obs2_grade};


    // GridCSpace2DConstructor grade
    // config grade(lower_bound_c_space_1st_dim, upper_bound_c_space_1st_dim, lower_bound_c_space_2st_dim, upper_bound_c_space_2st_dim, x_config_size, y_config_size);
    grader_class grade;
    amp::HW4::grade<link_solver>(grade, "jomi7243@colorado.edu", argc, argv);
    return 0;
}


