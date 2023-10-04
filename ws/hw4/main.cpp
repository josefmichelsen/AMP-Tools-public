// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
//#include <Eigen/Core>
// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;



class link_solver : public LinkManipulator2D{

    public:

    link_solver(const std::vector<double>& link_lengths)
    :  LinkManipulator2D(link_lengths){}

    link_solver()
    :  LinkManipulator2D(){}

    Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const{

        double x_sum, y_sum, angle_sum;
        //std::cout << " here 1 \n";

        const std::vector<double>& links =  getLinkLengths();
         //std::cout << " here 2 \n";
        
        const Eigen::Vector2d& base_loc = getBaseLocation();
         //std::cout << " here 3 \n";

        
        std::vector<double> temp;
        

        x_sum = base_loc[0];//[0];
        y_sum = base_loc[1];//[1];
        angle_sum = 0;
         //std::cout << " here 4 joint index is " << joint_index << "\n";

         //std::cout << "size of state is " << state.size() << "size of links is " << links.size() << "\n";

         //std::cout << "state 2 is " << state[2] << "\n";

        for (int i = 0; i <= joint_index; i++){
            //std::cout << "currently at iterator " << i << "\n";
            angle_sum = angle_sum + state[i];
            //std::cout << "at iterator " << i << " state is " << state[i] << " link is " << links[i] << "\n";
            x_sum = x_sum + links[i] * cos(angle_sum);
            y_sum = y_sum + links[i] * sin(angle_sum);
            
        }   
        Eigen::Vector2d ans(x_sum,y_sum);

        return ans;

    }

    ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
        
        double t3 = 5 * 3.141592653589793238463/4; // assume that this is theta 3 because at the moment it is a 3DOF problem in a 2D workspace meaning we have infinite
        //potential answers so we can arbitrarily pick the final angle.

        const std::vector<double>& links =  getLinkLengths();
        
        double l1 = links[0];
        double l2 = links[1];
        double l3 = links[2];

        double p3_x = end_effector_location[0] - l3 * cos(t3);
        double p3_y = end_effector_location[1] - l3 * sin(t3);

        std::cout << "point 3 is at " << p3_x << " " << p3_y << "\n";

        double t2 = acos( (1/(2 * l1 * l2)) * ( (pow(p3_x,2) + pow(p3_y,2)) - (pow(l1,2) + pow(l2,2)) ) );

        double t1 = asin( (1/(pow(p3_x,2) + pow(p3_y,2))) * (p3_y * (l1 + l2 * cos(t2)) - p3_x * l2 * sqrt(1 - pow(cos(t2),2)) ) );

        std::cout << "Theta 1 is " << t1 * 180.0/3.141592653589793238463<< " Theta 2 is " << t2 * 180.0/3.141592653589793238463<< " Theta 3 is " << (t3 - t2 - t1) * 180.0/3.141592653589793238463<< "\n";     
        ManipulatorState ans = {t1,t2,t3 - t2 - t1};
        
        return ans;

    }


};

class config : public ConfigurationSpace2D{

    public: 

        Environment2D env;
        std::vector<double> link_lengths;


        config(double x0_min, double x0_max, double x1_min, double x1_max)
        : ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max){}

        bool inCollision(double x0, double x1) const{
            
            link_solver link_obj(link_lengths);
            const std::vector<double> l = link_obj.getLinkLengths();
            const ManipulatorState state = {x0,x1};
            uint32_t joint_index = 1;
            
            Eigen::Vector2d point = link_obj.getJointLocation(state, joint_index);
            std::cout << "point of the end effector is " << point[0] << "  " << point[1] << "\n";
            
            double x = point[0];
            double y = point[1];

            double min_x, max_x, min_y, max_y;

            for (int i = 0; i < env.obstacles.size(); i++){

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

                if (x >= min_x && x <= max_x && y >= min_y && y <= max_y){ //this is where we need collision logic
                    
                    int ccw, cw = 0;

                    for(int i = 0; i < env.obstacles[i].verticesCCW().size(); i++){
                        
                        double x2, y2, cross, x1, y1;

                        if (i == env.obstacles[i].verticesCCW().size() - 1){

                            x2 = env.obstacles[i].verticesCCW()[0][0];
                            y2 = env.obstacles[i].verticesCCW()[0][1];
                        }
                        else{

                            x2 = env.obstacles[i].verticesCCW()[i + 1][0];
                            y2 = env.obstacles[i].verticesCCW()[i + 1][1];

                        }

                        x1 = env.obstacles[i].verticesCCW()[i][0];
                        y1 = env.obstacles[i].verticesCCW()[i][1];

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


int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    
    //= amp::LinkManipulator2D::LinkManipulator2D(test2);
    //std::vector<Eigen::Vector2d> O1 = {{-1,-2},{0,-2},{0,0}}; // pre set the vertices to be negative of the original
    std::vector<Eigen::Vector2d> O1 = {{0,0},{1,0},{1,2}}; // this works if we consider 0,0 the bottom left vertex after the flip
    amp::Polygon robot(O1);

    //std::cout<< W1_obstacle.verticesCCW()[0][0] << " " << W1_obstacle.verticesCCW()[0][1] << std::endl;

    //config c(x_min,y_min,x_max,y_max);
    //amp::ConfigurationSpace2D config_space(x_min,y_min,x_max,y_max);
    Obstacle2D problem1 =  HW4::getEx1TriangleObstacle();

    for(int i = 0; i < problem1.verticesCCW().size(); i++){
        std::cout << problem1.verticesCCW()[i][0] << " " << problem1.verticesCCW()[i][1] << "\n";
    }

    std::vector<std::vector<double>> v_and_ang_robot, v_and_ang_obst;
    double edge_x, edge_y, dot, det, angle;
    std::vector<double> cur_edge;

    for(int i = 0; i < problem1.verticesCCW().size(); i++){
        if (i == problem1.verticesCCW().size() -1){
            edge_x = problem1.verticesCCW()[0][0] - problem1.verticesCCW()[i][0];
            edge_y = problem1.verticesCCW()[0][1] - problem1.verticesCCW()[i][1];
            angle = std::atan2(edge_y,edge_x) * 180.0/3.141592653589793238463;  //subtracting 90 to get the normal to that vec
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
            angle = std::atan2(edge_y,edge_x) * 180.0/3.141592653589793238463;//subtracting 90 to get the normal to that vec
            if(angle < 0){
                angle = angle + 360;
            }
            else if (angle == 360){
                angle = 0;
            }
            cur_edge = {problem1.verticesCCW()[i][0], problem1.verticesCCW()[i][1], angle}; 
            v_and_ang_obst.push_back(cur_edge);
        }
        //std::cout << problem1.verticesCCW()[i][0] << " " << problem1.verticesCCW()[i][1] << "\n";
    }
    
    std::cout << "EDGES OBSTACLE----------------\n";
    for(int i = 0; i < problem1.verticesCCW().size(); i++){
        std::cout << v_and_ang_obst[i][0] << " " << v_and_ang_obst[i][1] << " " << v_and_ang_obst[i][2] << "\n";
    }

    //std::cout << "----------------\n";

    for(int i = 0; i < robot.verticesCCW().size(); i++){
        if (i == robot.verticesCCW().size() -1){
            edge_x = robot.verticesCCW()[0][0] - robot.verticesCCW()[i][0];
            edge_y = robot.verticesCCW()[0][1] - robot.verticesCCW()[i][1];
            angle = std::atan2(edge_y,edge_x) * 180.0/3.141592653589793238463; //adding 90 to get the normal to that vec
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
            angle = std::atan2(edge_y,edge_x) * 180.0/3.141592653589793238463; // adding 90 to get normal to that vec
            if(angle < 0){
                angle = angle + 360;
            }
            else if (angle == 360){
                angle = 0;
            }
            cur_edge = {robot.verticesCCW()[i][0], robot.verticesCCW()[i][1], angle}; 
            v_and_ang_robot.push_back(cur_edge);
        }
        //std::cout << "robot " << robot.verticesCCW()[i][0] << " " << robot.verticesCCW()[i][1] << "\n";
    }

    std::cout << "EDGES ROBOT----------------\n";
    for(int i = 0; i < problem1.verticesCCW().size(); i++){
        std::cout << v_and_ang_robot[i][0] << " " << v_and_ang_robot[i][1] << " " << v_and_ang_robot[i][2] << "\n";
    }

    std::vector<double> all_angles;
    for (int i = 0; i < v_and_ang_obst.size() + v_and_ang_robot.size(); i++){
        if (i <= 2){
            std::cout << i << "\n";
            all_angles.push_back(v_and_ang_obst[i][2]);
        }
        else{
            all_angles.push_back(v_and_ang_robot[i-3][2]);
        }
    }
    std::cout << "FULL ANGLES LIST" << "\n";
    for(int i = 0; i < all_angles.size(); i++){
        std::cout << all_angles[i] << "\n";
    }

    std::sort(all_angles.begin(),all_angles.end());

    std::cout << "sorted full ANGLES LIST" << "\n";
    for(int i = 0; i < all_angles.size(); i++){
        std::cout << all_angles[i] << "\n";
    }
    std::vector<double> a, b; //a is robot, b is obst
    b = {v_and_ang_obst[0][0], v_and_ang_obst[0][1]};
    a = {v_and_ang_robot[0][0], v_and_ang_robot[0][1]};
    std::vector<std::vector<double>> c_space_vertices = {{a[0] + b[0], a[1] + b[1]}};
    std::vector<double> next_c_space_point, old_point;

    //old_point = c_space_vertices[0];
    for (int i = 0; i < all_angles.size() ; i++ ){
        for (int j = 0; j < v_and_ang_obst.size(); j++){

            if(all_angles[i] == v_and_ang_obst[j][2]){
                b = {v_and_ang_obst[j][0], v_and_ang_obst[j][1]};
                next_c_space_point = {a[0] + b[0], a[1] + b[1]};
                c_space_vertices.push_back(next_c_space_point);
                std::cout << "object vertex being added " << b[0] << " " << b[1] << "  being added to " <<  a[0] << " " << a[1] << "\n";

                //old_point = {v_and_ang_obst[j][0], v_and_ang_obst[j][1]};
            }
            else if (all_angles[i] == v_and_ang_robot[j][2]){
                a = {v_and_ang_robot[j][0], v_and_ang_robot[j][1]};
                next_c_space_point = {a[0] + b[0], a[1] + b[1]};
                c_space_vertices.push_back(next_c_space_point);
                 std::cout << "robot vertex being added " << a[0] << " " << a[1] << "  being added to " <<  b[0] << " " << b[1] << "\n";
                //old_point = {v_and_ang_robot[j][0], v_and_ang_robot[j][1]};
            }

        }
    }

    std::cout << "DISPLAYING C SPACE OBJECT VERTEX LIST" << "\n";
    for(int i = 0; i < all_angles.size(); i++){
        std::cout << c_space_vertices[i][0] << " " << c_space_vertices[i][1] << "\n";
    }
    // std::cout << "----------------\n";
    // for(int i = 0; i < edges.size(); i++){
    //     std::cout << edges[i][0] << " " << edges[i][1] << " " << edges[i][2] << "\n";
    // }

    //changing angles to be total sum
    
    
    const std::vector<double> link_lengths = {8,8,9};
    const ManipulatorState thetas = {45,90,23.6};
    uint32_t joint_index = 2;
    link_solver linky_boi(link_lengths);
    Eigen::Vector2d sol = linky_boi.getJointLocation(thetas, joint_index);
    
    // Visualizer::makeFigure(linky_boi, thetas);
    // Visualizer::showFigures();

    std::cout << "position from new function x " << sol[0] << " y " << sol[1] << "\n";

    const Eigen::Vector2d end_effector_location(0.0,4.0);

    ManipulatorState test2 =  linky_boi.getConfigurationFromIK(end_effector_location);

    // Eigen::Vector2d 01({-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1});
    // Eigen::Vector2d 01({-2,-2},{-2,-1.8},{2,-1.8},{2,-2});
    std::vector<Eigen::Vector2d> o1 = {{-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1} };
    std::vector<Eigen::Vector2d> o2 = {{-2,-2},{-2,-1.8},{2,-1.8},{2,-2}};
    Obstacle2D obs1(o1);
    Obstacle2D obs2(o2);
    config problem_3(0.0,180.0,0.0,180.0);

    std::vector<Obstacle2D> obs_to_pass = {o1,o2};
    
    problem_3.env.obstacles = obs_to_pass; 

    for (int i = 0; i < problem_3.env.obstacles[0].verticesCCW().size(); i++){
        std::cout << "vertex " << problem_3.env.obstacles[0].verticesCCW()[i][0] << " " << problem_3.env.obstacles[0].verticesCCW()[i][1] << "\n";
    }
    std::vector<double> link_lengths_to_set = {1.0,1.0}; 
    problem_3.link_lengths = link_lengths_to_set;
    
    bool testing = problem_3.inCollision(2.0,1.0);
    double x_min,y_min,x_max,y_max;
    std::cout << "am I in an obstacle? " << testing << "\n";
    // x_min = 0;
    // y_min = 0;
    // x_max = 10;
    // y_max = 10;
    // std::size_t x_cell, y_cell = 10;
    // GridCSpace2D bool_array(x_cell, y_cell, x_min, x_max, y_min, y_max);

    //std::cout << bool_array[0][0] << "\n";
    
    
    // Visualizer::makeFigure(linky_boi, test2);
    // Visualizer::showFigures();


    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}


