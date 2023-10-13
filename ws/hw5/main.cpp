// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

std::vector<std::vector<double>> computeRepulsiveForce(std::vector<double> cur, Problem2D problem, double nu, double q_star);
std::vector<double> computeAttractiveForce(std::vector<double> goal, std::vector<double> cur, double weight, double d_star);
std::vector<double> point_to_line_dist(std::vector<double> vert_1, std::vector<double> vert_2, std::vector<double> point);
double dist_between_two_points(double x1, double y1, double x2, double y2);

class MyGDAlgo : public amp::GDAlgorithm{
    public:

    double epsilon; //how close we need to get to goal to terminate
    double att_force_weight;//scaling factor for attractive force
    double d_star; //threshold distance from goal to change how we calculate attractive force
    double nu; //gain on repulsive gradient
    double q_star;//distance at which point we start considering obstacles repulsive force
    int iter_max;
    amp::Path2D path;
    
    virtual amp::Path2D plan(const amp::Problem2D& problem) override{
        amp::Path2D path_reset;
        path = path_reset;
        path.waypoints.push_back(problem.q_init);
        int iter = 0;
        double dist_to_goal = dist_between_two_points(path.waypoints.back()[0], path.waypoints.back()[1], problem.q_goal[0], problem.q_goal[1]);
        double gradient = 100;
        std::vector<double> goal_vec = {problem.q_goal[0], problem.q_goal[1]};

        //while((abs(gradient) > epsilon && dist_to_goal > epsilon) && iter < iter_max){
        while((dist_to_goal > d_star || gradient > epsilon)  && iter < iter_max){
            std::vector<double> current_point = {};
            std::vector<std::vector<double>> rep = {{}};
            std::vector<double> att = {};
            current_point.push_back(path.waypoints.back()[0]);
            current_point.push_back(path.waypoints.back()[1]);
            rep = computeRepulsiveForce(current_point, problem, nu, q_star);
            //std::cout << "computed rep force\n";
            att = computeAttractiveForce(goal_vec, current_point, att_force_weight, d_star);
            double x_rep = 0;
            double y_rep = 0;
            double rep_mag = 0;

            for (int i = 0; i < rep.size(); i++){
                x_rep = x_rep + rep[i][0];
                y_rep = y_rep + rep[i][1];
                //rep_mag = rep_mag + rep[i][2];
            }

            //std::cout << "rep mag is " << rep_mag  << " att mag " << att[2] << "\n";
            // std::cout << "attraction contribution in x " << att[0] << " in y " << att[1] << "\n";
            //std::cout << "current x " << current_point[0] << " current y " << current_point[1] << "           y repulsion " << y_rep << " y attraction " << att[1] << "\n";
            double alpha = 0.054;
            
            // std::cout << "x_rep is " << trunc(x_rep * 1000)  << " x att " << trunc(-att[0] * 1000) << "  y rep is " << trunc(y_rep)  << " y attraction is " << trunc(att[1])<< "\n";
            // std::cout << "rounding " << round(x_rep * 10)/10 << "   " << round(-att[0] * 10)/10 << "\n";
            if(round(x_rep * 10)/10 == round(-att[0] * 10)/10 && (trunc(y_rep) == 0 && trunc(att[1]) == 0)){
                rep = computeRepulsiveForce(current_point, problem, nu, 1000);
                for (int i = 0; i < rep.size(); i++){
                    y_rep = y_rep + rep[i][1];
                }

                if( y_rep > 0){
                    y_rep = 10.0;
                }
                else{
                    y_rep = -20.0;
                }
                x_rep = 0;
            }
            std::cout << "x_rep is " << x_rep  << " y rep is " << y_rep << "  x attactino is " << att[0]  << " y attraction is " << att[1]<< "\n";
            double next_x = current_point[0] - alpha * (x_rep + att[0]);
            double next_y = current_point[1] - alpha * (y_rep + att[1]);

            Eigen::Vector2d next_point(next_x, next_y);
            path.waypoints.push_back(next_point);
            iter = iter + 1;
            gradient = sqrt(pow(x_rep + att[0], 2) + pow(y_rep + att[1], 2));
            //std::cout << "gradient is " << gradient << "\n";
            dist_to_goal = dist_between_two_points(path.waypoints.back()[0], path.waypoints.back()[1], problem.q_goal[0], problem.q_goal[1]);
        }
        std::cout << "number of iterations was " << iter<< "\n";
        //Eigen::Vector2d test_end(0.0, 10.0);
        path.waypoints.push_back(problem.q_goal);
        //path.waypoints.push_back(test_end);
        return path;
        
    }

    double calcPathDist(amp::Path2D path){
        double distance = 0;
        for(int i = 0; i < path.waypoints.size() - 1; i++){
            distance = distance + sqrt(pow(path.waypoints[i][0] - path.waypoints[i + 1][0], 2) + pow(path.waypoints[i][1] - path.waypoints[i + 1][1], 2));
        }
        return distance;
    }

};

int main(int argc, char** argv) {    
    // workspaces fro hw2, to use these needed to include the #include "hw/HW2.h" line
    //Problem2D hw5_environtment = HW2::getWorkspace1();    
    Problem2D hw5_environtment = HW2::getWorkspace2();

    //workspace from specifically hw5
    // std::vector<Eigen::Vector2d> o1 = {{3.5,0.5},{4.5,0.5},{4.5,1.5},{3.5,1.5}};
    // std::vector<Eigen::Vector2d> o2 = {{6.5,-1.5},{7.5,-1.5},{7.5,-0.5},{6.5,-0.5}};
    // Obstacle2D obs1(o1);
    // Obstacle2D obs2(o2);
    // std::vector<Obstacle2D> obs_to_pass = {obs1,obs2};
    // Problem2D hw5_environtment;
    // hw5_environtment.obstacles = obs_to_pass;
    
    // Eigen::Vector2d start(0.0,0.0);
    // Eigen::Vector2d goal(10.0,0.0);
    // //std::vector<double> goal_vec = {10.0, 0.0};
    // hw5_environtment.q_init = start;
    // hw5_environtment.q_goal = goal;

    hw5_environtment.x_min = -10.0;
    hw5_environtment.x_max = 40.0;
    hw5_environtment.y_min = -10.0;
    hw5_environtment.y_max = 20.0;
    // hw5_environtment.x_min = -5.0;
    // hw5_environtment.x_max = 15.0;
    // hw5_environtment.y_min = -10.0;
    // hw5_environtment.y_max = 10.0;

    MyGDAlgo solver;

    solver.epsilon = 0.25;//when to stop the search
    solver.att_force_weight = 1.1;//scaling factor for attractive force
    solver.d_star = 1.0; //threshold distance from goal to change how we calculate attractive force
    solver.nu = 1.2; //gain on repulsive gradient
    solver.q_star = 0.2;//distance at which point we start considering obstacles repulsive force
    solver.iter_max = 10000;

    amp::Path2D final_path = solver.plan(hw5_environtment);
    double path_dist = solver.calcPathDist(final_path);
    std::cout << "distance we traveled was " << path_dist << "\n";

    Visualizer workspace;
    workspace.makeFigure(hw5_environtment,final_path);
    workspace.showFigures();

    amp::HW5 grade;
    bool did_i_pass = grade.check(final_path,hw5_environtment,true);
    std::cout << "did you pass? " << did_i_pass << "\n"; 
    
    amp::GDAlgorithm* new_point;

    new_point = &solver;

    int final_grade = amp::HW5::grade(*new_point, "jomi7243@colorado.edu", argc, argv);
    //std::cout << "final grade is " << final_grade << "\n";
    return 0;
}



std::vector<std::vector<double>> computeRepulsiveForce(std::vector<double> cur, Problem2D problem, double nu, double q_star){
    std::vector<std::vector<double>> rep_force;

    for(int i = 0; i < problem.obstacles.size(); i++){ //calculate closest point from each obstacle to my current point
        //std::cout<<"getting in first for loop \n";
        double min_dist = q_star + 0.5;
        std::vector<double> dist_to_obst = {};
        double closest_point_x = 0;
        double closest_point_y = 0;
        double closest_point_dist = 0;
        //rep_force = {{}};
        for(int j = 0; j < problem.obstacles[i].verticesCCW().size(); j++){
            //std::cout<<"getting in second for loop \n";
            std::vector<double> vertex_1 = {};
            std::vector<double> vertex_2 = {};

            vertex_1.push_back(problem.obstacles[i].verticesCCW()[j][0]);
            vertex_1.push_back(problem.obstacles[i].verticesCCW()[j][1]);
            //std::cout<<"assigning first vertices \n";

            if ( j == problem.obstacles[i].verticesCCW().size() - 1){
                vertex_1[0] = problem.obstacles[i].verticesCCW()[0][0];
                vertex_1[1] = problem.obstacles[i].verticesCCW()[0][1];
                vertex_2.push_back(problem.obstacles[i].verticesCCW()[j][0]);
                vertex_2.push_back(problem.obstacles[i].verticesCCW()[j][1]);
            }
            else{
                vertex_2.push_back(problem.obstacles[i].verticesCCW()[j + 1][0]);
                vertex_2.push_back(problem.obstacles[i].verticesCCW()[j + 1][1]);
            }
            //std::cout<<"assigning second vertices \n";
            dist_to_obst = point_to_line_dist(vertex_1, vertex_2, cur);
            // std::cout <<"getting here\n";
            if(dist_to_obst[2] < min_dist){
                //std::cout <<"THERE SHOULD BE REPULSIVE FORCE\n";
                min_dist = dist_to_obst[2];
                closest_point_x = dist_to_obst[0];
                closest_point_y = dist_to_obst[1];
                closest_point_dist = dist_to_obst[2];
            }

        }

        if(min_dist <= q_star){
            
            // double gradient_dir_x = (cur[0] - closest_point_x) / closest_point_dist;
            // double gradient_dir_y = (cur[1] - closest_point_y) / closest_point_dist;
            // std::cout << "1/qstar = " << 1/q_star <<"\n";
            // std::cout << "1/cp dist = " << 1/closest_point_dist<< "\n";
            // std::cout << "1/cp dist squared = " << 1/pow(closest_point_dist, 2) <<"\n";
            // std::cout << "x direction = " << cur[0] - closest_point_x <<"\n";
            // std::cout << "y direction = " << cur[1] - closest_point_y<<"\n";
            double gradient_x = nu * ((1/q_star) - (1/closest_point_dist)) * (1/pow(closest_point_dist, 2)) * (cur[0] - closest_point_x);
            double gradient_y = nu * ((1/q_star) - (1/closest_point_dist)) * (1/pow(closest_point_dist, 2)) * (cur[1] - closest_point_y);
            // gradient_x = gradient_x * gradient_dir_x;
            // gradient_y = gradient_y * gradient_dir_y;
            //std::cout << "able to see I am within distance of obstacle " << gradient_x << " " << gradient_y << "\n";
            double mag = (0.5 * nu * pow(((1/closest_point_dist) - (1/q_star)),2));
            //std::cout<<"gradient going out " << gradient_x << " " <<gradient_y << "\n";
            // sleep(1);
            rep_force.push_back({gradient_x, gradient_y, mag});
        }

    }

    return rep_force;

}

std::vector<double> computeAttractiveForce(std::vector<double> goal, std::vector<double> cur, double weight, double d_star){
    
    double goal_x = goal[0];
    double goal_y = goal[1];

    double cur_x = cur[0];
    double cur_y = cur[1];
    
    double dist_to_goal = sqrt( pow(goal_x - cur_x, 2) + pow(goal_y - cur_y, 2));
    std::vector<double> att_force;
    //std::cout << "got all the input variables\n";
    if(dist_to_goal <= d_star){
        att_force.push_back(weight * (cur_x - goal_x));
        att_force.push_back(weight * (cur_y - goal_y));
        att_force.push_back(0.5 * weight * pow(dist_to_goal,2));
    }
    else{
        att_force.push_back((d_star * weight * (cur_x - goal_x))/dist_to_goal);
        att_force.push_back((d_star * weight * (cur_y - goal_y))/dist_to_goal);
        att_force.push_back(d_star * weight * dist_to_goal - 0.5 * weight * pow(d_star,2));
    }

    return att_force;
    
}

std::vector<double> point_to_line_dist(std::vector<double> vert_1, std::vector<double> vert_2, std::vector<double> point){

    std::vector<double> v_1_to_2 = {vert_2[0] - vert_1[0], vert_2[1] - vert_1[1]};
    std::vector<double> v_1_to_p = {point[0] - vert_1[0], point[1] - vert_1[1]};
    
    double dot = v_1_to_2[0] * v_1_to_p[0] + v_1_to_2[1] * v_1_to_p[1];
    double line_length_sqrd = pow(v_1_to_2[0],2) + pow(v_1_to_2[1],2);
    // if(line_length_sqrd == 0){
    //     std::cout << "dividiing by zero \n";
    //     sleep(1);
    // }
    double d_on_line = dot/line_length_sqrd;

    
    
    std::vector<double> closest_point = {};
    //std::cout << "getting to compare\n";
    if(d_on_line >= 1){
        closest_point = vert_2;
    }
    else if (d_on_line <= 0){
        closest_point = vert_1;
    }
    else{
        double x_cord = vert_1[0] + v_1_to_2[0] * d_on_line; 
        double y_cord = vert_1[1] + v_1_to_2[1] * d_on_line;
        // std::cout << "d on line = " << d_on_line << " x verts " << vert_1[0] << " " << v_1_to_2[0] << "\n";
        // std::cout << "x cord = " << x_cord << " " << y_cord << "\n";
        //sleep(1);
        closest_point.push_back(x_cord);
        closest_point.push_back(y_cord);
    }
    //std::cout<<"to assign ans \n";
    std::vector<double> ans = {};
    ans.push_back(closest_point[0]);
    ans.push_back(closest_point[1]);
    ans.push_back(sqrt( pow(point[0] - closest_point[0], 2) + pow(point[1] - closest_point[1], 2)));
    //std::cout<<"returning ans\n";
    // if(vert_2[0] == vert_1[0]){
    //     // std::cout << "vertex 1 = " << vert_1[0] << "  " << vert_1[1] << "  vertex 2 = " << vert_2[0] << " " << vert_2[1] << "\n"; 
    //     // std::cout << "dot = " << dot << " distance " << sqrt( pow(point[0] - closest_point[0], 2) + pow(point[1] - closest_point[1], 2)) << "\n";
    //     // std::cout << "point  = " << point[0] << "  " << point[1] << "  closest point = " << closest_point[0] << " " << closest_point[1] << "\n"; 
    //     sleep(3);
    // }
    return ans;
}

double dist_between_two_points(double x1, double y1, double x2, double y2){

    double dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    return dist;

}