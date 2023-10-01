#include "MyBugAlgorithm.h"
#include <vector>
#include <iomanip>

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;

    path.waypoints.push_back(problem.q_init); // starting at the initial location

    //int bug = 2;
    int bug = 1;

    float m, b, dx;

    dx = 0.5; // this is the distance I want to travel in x when following the mline 

    std::vector<float> line = equation_of_line(problem.q_init[0], problem.q_init[1], problem.q_goal[0], problem.q_goal[1]);

    m = line[0];
    b = line[1];
    int pos = 2;

    if(m == 1000 && b == 1001){
        pos = line[2];
    }
    
    float new_x, new_y, current_x, current_y, next_x, next_y, dy;
    int passed_goal;
    ////std::cout<<"before loop current x trunc  " << trunc(10 * path.waypoints.back()[0]) << " goal x trunc " << trunc(10 * problem.q_goal[0]) << "\n";
    ////std::cout<<"before loop current y trunc  " << trunc(10 * path.waypoints.back()[1]) << " goal x trunc " << trunc(10 * problem.q_goal[1]) << "\n";
    ////std::cout << "first condition " << (trunc(10 * path.waypoints.back()[0]) != trunc(10 * problem.q_goal[0])) << "\n";
    ////std::cout << "first condition " << (trunc(10 * path.waypoints.back()[1]) != trunc(10 * problem.q_goal[1])) << "\n";
    current_x = path.waypoints.back()[0];
    current_y = path.waypoints.back()[1];
    
    if (m == 1000 && b == 1001){
        next_x = current_x;
        dx = 0;
        if (pos == 1){
            dy = 0.5;
        }
        else if (pos == 0){
            dy = -0.5;
        }
        
        next_y = current_y + dy;
    }
    else{
        next_x = current_x + dx;
        next_y = m * next_x + b;
        dy = next_y - current_y;
    }

    if( bug == 2){
        while( trunc(10 * path.waypoints.back()[0]) != trunc(10 * problem.q_goal[0]) || trunc(10 * path.waypoints.back()[1]) != trunc(10 * problem.q_goal[1])){
            ////std::cout<<"in loop current x trunc  " << trunc(10 * path.waypoints.back()[0]) << " goal x trunc " << trunc(10 * problem.q_goal[0]) << "\n";
            ////std::cout<<"in loop current y trunc  " << trunc(10 * path.waypoints.back()[1]) << " goal x trunc " << trunc(10 * problem.q_goal[1]) << "\n";
            current_x = path.waypoints.back()[0];
            current_y = path.waypoints.back()[1];
            // next_x = current_x + dx;
            // next_y = m * next_x + b;
            // dy = next_y - current_y;
            if (m < 0){
                dx = -0.5;
                next_x = current_x + dx;
                next_y = m * next_x + b;
                dy = next_y - current_y;
            }
            else if (trunc(10 * m) == 0){
                if(current_x > problem.q_goal[0]){
                    dx = -0.5;
                    next_x = current_x + dx;
                    next_y = current_y;
                    dy = 0;
                }
                else if (current_x < problem.q_goal[0]){
                    dx = 0.5;
                    next_x = current_x + dx;
                    next_y = current_y;
                    dy = 0;
                }
            }
            else{
                next_x = current_x + dx;
                next_y = m * next_x + b;
                dy = next_y - current_y;
            }



            if(path.waypoints.size() > 5000){
                path.waypoints.push_back(Eigen::Vector2d(problem.q_goal[0], problem.q_goal[1]));
                break;
            }            

            passed_goal = orientation(current_x, current_y, next_x, next_y, problem.q_goal[0], problem.q_goal[1]);
            
            // if(passed_goal == 0){

            //     if (trunc(10 * (m * problem.q_goal[0] + b)) == trunc(10 * problem.q_goal[1]) && (current_x < problem.q_goal[0] && next_x > problem.q_goal[0]) && (current_y < problem.q_goal[1] && next_y > problem.q_goal[1])){
            //         break;
            //     }
                
            // }
             if(passed_goal == 0){
                bool test_pass_x = (trunc(10 * current_x) <= trunc(10 * problem.q_goal[0]) && trunc(10 *  next_x) >= trunc(10 * problem.q_goal[0])) || (trunc(10 * current_x) >= trunc(10 * problem.q_goal[0]) && trunc(10 *  next_x) <= trunc(10 * problem.q_goal[0]));
                bool test_pass_y = (trunc(10 *  current_y) <= trunc(10 *  problem.q_goal[1]) && trunc(10 *  next_y) >= trunc(10 *  problem.q_goal[1])) || (trunc(10 *  current_y) >= trunc(10 *  problem.q_goal[1]) && trunc(10 *  next_y) <= trunc(10 *  problem.q_goal[1]));
                if (test_pass_x && test_pass_y){
                    path.waypoints.push_back(Eigen::Vector2d(problem.q_goal[0], problem.q_goal[1]));
                    break;
                }
            }

            ////std::cout << "currently at point     " << current_x << "    " << current_y << "\n";
            ////std::cout<<"current x trunc  " << trunc(10 * path.waypoints.back()[0]) << " goal x trunc " << trunc(10 * problem.q_goal[0]) << "\n";
            ////std::cout<<"current y trunc  " << trunc(10 * path.waypoints.back()[1]) << " goal x trunc " << trunc(10 * problem.q_goal[1]) << "\n";

            std::vector<float> collision = Check_Collision(problem, current_x, current_y, next_x, next_y, path);

            if(collision[0] == 1){

                //std::cout << "First one the object I would hit is   " << collision[1] << "  the vertices describing the line I would hit are     " << collision[2] << "  " << collision[3] << "\n";
                //std::cout<<"last leave point I am passing in is--------------------"<<current_x <<"  "<<current_y << "\n";
                follow(problem, current_x, current_y, current_x, current_y, dx, dy, current_x, m, b, collision, path);
                //std::cout<<"I am outside the follow"<<"\n";

            }
            else{

                path.waypoints.push_back(Eigen::Vector2d(next_x, next_y));

            }
            //sleep(1);

        }
    }
    else if (bug == 1){
        while( trunc(10 * path.waypoints.back()[0]) != trunc(10 * problem.q_goal[0]) || trunc(10 * path.waypoints.back()[1]) != trunc(10 * problem.q_goal[1])){
            if(path.waypoints.size() > 5000){
                path.waypoints.push_back(Eigen::Vector2d(problem.q_goal[0], problem.q_goal[1]));
                break;
            }  
            
            current_x = path.waypoints.back()[0];
            current_y = path.waypoints.back()[1];

            if (m < 0){
                dx = -0.5;
                next_x = current_x + dx;
                next_y = m * next_x + b;
                dy = next_y - current_y;
            }
            else if (trunc(10 * m) == 0){
                if(current_x > problem.q_goal[0]){
                    dx = -0.5;
                    next_x = current_x + dx;
                    next_y = current_y;
                    dy = 0;
                }
                else if (current_x < problem.q_goal[0]){
                    dx = 0.5;
                    next_x = current_x + dx;
                    next_y = current_y;
                    dy = 0;
                }
            }
            else{
                next_x = current_x + dx;
                next_y = m * next_x + b;
                dy = next_y - current_y;
            }

            passed_goal = orientation(current_x, current_y, next_x, next_y, problem.q_goal[0], problem.q_goal[1]);
            
            if(passed_goal == 0){
                ////std::cout<<"we recognize that our line and the goal line are the same\n";
                ////std::cout << "test 1 = " << (trunc(10 * (m * problem.q_goal[0] + b)) == trunc(10 * problem.q_goal[1])) << " test 2 " << (current_x < problem.q_goal[0] && next_x > problem.q_goal[0]) << " test 3" <<(current_y < problem.q_goal[1] && next_y > problem.q_goal[1]) << "\n";
                bool test_pass_x = (trunc(10 * current_x) <= trunc(10 * problem.q_goal[0]) && trunc(10 *  next_x) >= trunc(10 * problem.q_goal[0])) || (trunc(10 * current_x) >= trunc(10 * problem.q_goal[0]) && trunc(10 *  next_x) <= trunc(10 * problem.q_goal[0]));
                bool test_pass_y = (trunc(10 *  current_y) <= trunc(10 *  problem.q_goal[1]) && trunc(10 *  next_y) >= trunc(10 *  problem.q_goal[1])) || (trunc(10 *  current_y) >= trunc(10 *  problem.q_goal[1]) && trunc(10 *  next_y) <= trunc(10 *  problem.q_goal[1]));
                //std::cout << "based new tests x " << test_pass_x << " and " << test_pass_y << "\n";
                // if (trunc(10 * (m * problem.q_goal[0] + b)) == trunc(10 * trunc(10 *  problem.q_goal[1])) && test_pass_x && test_pass_y){
                //     break;
                // }
                if (test_pass_x && test_pass_y){
                    path.waypoints.push_back(Eigen::Vector2d(problem.q_goal[0], problem.q_goal[1]));
                    break;
                }
            }

            ////std::cout << "currently at point     " << current_x << "    " << current_y << "\n";
            ////std::cout<<"current x trunc  " << trunc(10 * path.waypoints.back()[0]) << " goal x trunc " << trunc(10 * problem.q_goal[0]) << "\n";
            ////std::cout<<"current y trunc  " << trunc(10 * path.waypoints.back()[1]) << " goal x trunc " << trunc(10 * problem.q_goal[1]) << "\n";
            //sleep(1);

            std::vector<float> collision = Check_Collision(problem, current_x, current_y, next_x, next_y, path);

            if(collision[0] == 1){

                //std::cout << "First one the object I would hit is   " << collision[1] << "  the vertices describing the line I would hit are     " << collision[2] << "  " << collision[3] << "\n";
               ////std::cout<<"last leave point I am passing in is--------------------"<<current_x <<"  "<<current_y << "\n";
                std::vector<std::vector<float>> location_mem;
                location_mem.push_back(std::vector{current_x, current_y, dist_between_two_points(current_x, current_y, problem.q_goal[0], problem.q_goal[1])});
                int break_flag = 0;
                follow_bug1(problem, current_x, current_y, current_x, current_y, dx, dy, location_mem, m, b, collision, path, break_flag);
                //std::cout << " we have returned to the main loop to move toward the target and not follow\n";
                //recalculate m and b
                line = equation_of_line(path.waypoints.back()[0], path.waypoints.back()[1], problem.q_goal[0], problem.q_goal[1]);
                m = line[0];
                b = line[1];
                //std::cout << "equation of line we are trying to follow has m: " << m << " and b: " << b << "\n";
                //std::cout<<"I am outside the follow"<<"\n";

            }
            else{

                path.waypoints.push_back(Eigen::Vector2d(next_x, next_y));

            }
            //sleep(1);

        }

    }


    
    //path.waypoints.push_back(problem.q_goal);

    // calculate distance the robot has traveled
    float dist_traveled;
    for (int i = 0; i < path.waypoints.size() - 1 ; i++){
        // dist = dist_between_two_points(path.waypoints[i][0],path.waypoints[i][1], path.waypoints[i+1][0],path.waypoints[i+1][1]);
        ////std::cout << "point one is  " << path.waypoints[i][0] << " " << path.waypoints[i][1] << "\n";
        ////std::cout << "point two is  " << path.waypoints[i+1][0] << " " << path.waypoints[i+1][1] << "\n";
        ////std::cout << "dist between last two points  " << dist << "\n";
        dist_traveled += dist_between_two_points(path.waypoints[i][0],path.waypoints[i][1], path.waypoints[i+1][0],path.waypoints[i+1][1]);
    }
   std::cout << "total distance traveled is " << dist_traveled << "\n";
    return path;
}

std::vector<float> MyBugAlgorithm::Check_Collision(const amp::Problem2D& problem, float current_x, float current_y, float next_x, float next_y, amp::Path2D& path){

    std::vector<float> answer; // Convention: indx 0: will a collision happen on next time step? indx 1: what object would we be colliding with indx 2,3: what edge as defined by vertices would we collide with 
    float max_x, min_x, max_y, min_y;
    std::vector<int> obstacle_in_range; 
    std::vector<float> line = equation_of_line(current_x, current_y, next_x, next_y); 

    float m = line[0];
    float b = line[1];
    //std::cout << "TOTAL NUMBER OF OBSTACLES:  " << problem.obstacles.size() << "\n";
    for(int i = 0; i < problem.obstacles.size(); i++){

        max_x = problem.obstacles[i].verticesCCW()[0][0];
        min_x = problem.obstacles[i].verticesCCW()[0][0];
        max_y = problem.obstacles[i].verticesCCW()[0][1];
        min_y = problem.obstacles[i].verticesCCW()[0][1];

        for(int j = 0; j < problem.obstacles[i].verticesCCW().size(); j++){

            if(max_x < problem.obstacles[i].verticesCCW()[j][0]){
                max_x = problem.obstacles[i].verticesCCW()[j][0];
            }
            if(min_x > problem.obstacles[i].verticesCCW()[j][0]){
                min_x = problem.obstacles[i].verticesCCW()[j][0];
            }

            if(max_y < problem.obstacles[i].verticesCCW()[j][1]){
                max_y = problem.obstacles[i].verticesCCW()[j][1];
            }
            if(min_y > problem.obstacles[i].verticesCCW()[j][1]){
                min_y = problem.obstacles[i].verticesCCW()[j][1];
            }

        }

        if(next_x > max_x && current_x > max_x || current_y > max_y && next_y > max_y || next_x < min_x || next_y < min_y ){
            continue; 
        }
        else{
            obstacle_in_range.push_back(i);
        }

    }

    for(int i = 0; i < obstacle_in_range.size(); i++){

        for (int j = 0; j < problem.obstacles[obstacle_in_range[i]].verticesCCW().size(); j++){

            int next_vert_indx, vert_indx;

            float vert_x, vert_y, next_vert_x, next_vert_y, temp_vert_x, temp_vert_y;

            
            if( j == problem.obstacles[obstacle_in_range[i]].verticesCCW().size() - 1){
                vert_indx = j;
                next_vert_indx = 0;
            }
            //else if (problem.obstacles[obstacle_in_range[i]].verticesCCW()[j][0] == problem.obstacles[obstacle_in_range[i]].verticesCCW()[j+1][0])
            else{
                vert_indx = j;
                next_vert_indx = j+1;
            }

            vert_x = problem.obstacles[obstacle_in_range[i]].verticesCCW()[vert_indx][0];
            vert_y = problem.obstacles[obstacle_in_range[i]].verticesCCW()[vert_indx][1];
            next_vert_x = problem.obstacles[obstacle_in_range[i]].verticesCCW()[next_vert_indx][0];
            next_vert_y = problem.obstacles[obstacle_in_range[i]].verticesCCW()[next_vert_indx][1];
            
            int vert_x_orient = trunc(10 * vert_x);
            int vert_y_orient = trunc(10 * vert_y);
            int next_vert_x_orient = trunc(10 * next_vert_x);
            int next_vert_y_orient = trunc(10 * next_vert_y);
            int current_x_orient = trunc(10 * current_x);
            int current_y_orient = trunc(10 * current_y);
            int next_x_orient = trunc(10 * next_x);
            int next_y_orient = trunc(10 * next_y);

            //std::cout << "OBSTACLE IN RANGE: " << obstacle_in_range[i] << "\n";
            ////std::cout << "vertex 1 index is " << vert_indx << " vertex 2 index is " << next_vert_indx << "\n";
            ////std::cout << "Vertex one of obstacle in range: " << vert_x << "  " << vert_y << "\n";
            ////std::cout << "Vertex two of obstacle in range: " << next_vert_x << "  " << next_vert_y << "\n";

            //sleep(1);
            if(m == 1000 && b == 1001 && (next_x == vert_x || (next_y >= std::min(vert_y, next_vert_y) && next_y <= std::max(vert_y, next_vert_y))) ){
                if (vert_x == max_x){
                    answer = {1.0, (float)obstacle_in_range[i], (float)1, (float)2};
                }
                else if (vert_x == min_x){
                    answer = {1.0, (float)obstacle_in_range[i], (float)3, (float)0};
                }
            }
            else if (trunc(10. * (m * vert_x + b)) == trunc(10. * vert_y) || trunc(10. * (m * next_vert_x + b)) == trunc(10. * next_vert_y) && current_x < vert_x){
                answer = {1.0, (float)obstacle_in_range[i], (float)3, (float)0};
                return answer;
            }


            // if (trunc(10. * (m * vert_x + b)) == trunc(10. * vert_y) || trunc(10. * (m * next_vert_x + b)) == trunc(10. * next_vert_y) && current_x < vert_x){
            //     //std::cout << "I am on the m line\n";
            //     answer = {1.0, (float)obstacle_in_range[i], (float)3, (float)0};
            //     return answer;
            // }

            //std::cout << "failing in the m line check\n";
            int orient1 = orientation(vert_x_orient, vert_y_orient, next_vert_x_orient, next_vert_y_orient, current_x_orient, current_y_orient);
            //std::cout << "failing after orient1\n";
            int orient2 = orientation(vert_x_orient, vert_y_orient, next_vert_x_orient, next_vert_y_orient, next_x_orient, next_y_orient);
            //std::cout << "failing after orient2\n";
            int orient3 = orientation(current_x_orient, current_y_orient, next_x_orient, next_y_orient, vert_x_orient, vert_y_orient);
            //std::cout << "failing after orient3\n";
            int orient4 = orientation(current_x_orient, current_y_orient, next_x_orient, next_y_orient, next_vert_x_orient, next_vert_y_orient);
            //std::cout << "failing after orient4\n";
            //std::cout << "orientations : " << orient1 << " orient 2: " << orient2 << " orient 3: " << orient3 << " orient 4: " << orient4 << "\n";

            int next_on_obs = 0;

            if(next_x <= std::max(vert_x, next_vert_x) && next_x >= std::min(vert_x, next_vert_x) && next_y <= std::max(vert_y, next_vert_y) && next_y >= std::min(vert_y, next_vert_y)){
                next_on_obs = 1;
            }

            if ((orient1 != orient2 && orient3 != orient4) || (orient1 == 0 && next_on_obs == 1) || (orient2 == 0 && next_on_obs == 1) || (orient3 == 0 && next_on_obs == 1) || (orient4 == 0 && next_on_obs == 1)){
                //std::cout << "going to cross lines\n"; 
                answer = {1.0, (float)obstacle_in_range[i], (float)vert_indx, (float)next_vert_indx};
                return answer;            
            }
            // if (orient1 == 0 && next_on_obs == 1){
            //     //std::cout << "Orient 1\n";
            //     answer = {1.0, (float)obstacle_in_range[i], (float)vert_indx, (float)next_vert_indx};
            //     return answer;
            // }
            // else if (orient2 == 0 && next_on_obs == 1){
            //     //std::cout << "Orient 2\n";
            //     answer = {1.0, (float)obstacle_in_range[i], (float)vert_indx, (float)next_vert_indx};
            //     return answer;
            // }
            // else if (orient3 == 0 && next_on_obs == 1){
            //     //std::cout << "Orient 3\n";
            //     answer = {1.0, (float)obstacle_in_range[i], (float)vert_indx, (float)next_vert_indx};
            //     return answer;
            // }
            // else if (orient4 == 0 && next_on_obs == 1){
            //     //std::cout << "Orient 4\n";
            //     answer = {1.0, (float)obstacle_in_range[i], (float)vert_indx, (float)next_vert_indx};
            //     return answer;
            // }

        }

    }
    answer = {0,0,0,0};
    return answer;
}


int MyBugAlgorithm::orientation(int x1, int y1, int x2, int y2, int x3, int y3){

    float turn = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
    if (turn > 0){ //clockwise
        return 1;
    }
    else if (turn < 0){//counter clockwise 
        return 2;
    }
    else return 0; //showing they are colinear 

}


void MyBugAlgorithm::follow(const amp::Problem2D& problem, float current_x, float current_y, float initial_x, float initial_y, float incoming_dx, float incoming_dy, float last_leave_point, float m, float b, std::vector<float> info, amp::Path2D& path){
    
    std::vector<float> line;
    std::vector<float> follow_collision, next_hit_check;
    float dy, dx = 0;
    float next_x, next_y;
    
    //std::cout<<"goint to assign vertices to line of object to follow\n";
    float vertex1_x = problem.obstacles[info[1]].verticesCCW()[info[2]][0];
    //std::cout<<"vert1\n";
    float vertex1_y = problem.obstacles[info[1]].verticesCCW()[info[2]][1];
    //std::cout<<"vert2\n";
    float vertex2_x = problem.obstacles[info[1]].verticesCCW()[info[3]][0];
    //std::cout<<"vert3\n";
    float vertex2_y = problem.obstacles[info[1]].verticesCCW()[info[3]][1];
    //std::cout<<"vertices have been assigned\n";
    line = equation_of_line(vertex1_x, vertex1_y, vertex2_x, vertex2_y);
    ////std::cout << "I AM NOW FOLLOWING A NEW OBJECT LINE WITH THE FOLLOWING VERTICES " << "\n";
    ////std::cout << "vertix 1 locations x: " << vertex1_x << " y: " << vertex1_y << "\n";
    ////std::cout << "vertix 2 locations x: " << vertex2_x << " y: " << vertex2_y << "\n";
    

    if (line[0] == 1000 && line[1] == 1001 ){
        
        if(incoming_dx < 0){
            next_hit_check = Check_Collision(problem, current_x, current_y, current_x, current_y - 0.5, path);
            if(next_hit_check[0] == 1){
                dy = 0.5;
            }
            else if (next_hit_check[0] == 0 && current_x < vertex1_x && current_y < vertex1_y){
                dy = 0.5;
            }
            else{
                dy = -0.5;
            }
        }
        else if (incoming_dx > 0){
            next_hit_check = Check_Collision(problem, current_x, current_y, current_x, current_y + 0.5, path);
            if(next_hit_check[0] == 1){
                dy = -0.5;
            }
            else if (next_hit_check[0] == 0 && current_x > vertex2_x && current_y > vertex2_y){
                dy = -0.5;
            }
            else{
                dy = 0.5;
            }
        }
        dx = 0;
    }
    else if (line[0] == 0){
        if(incoming_dy < 0){
            next_hit_check = Check_Collision(problem, current_x, current_y, current_x + 0.5, current_y, path);
            if(next_hit_check[0] == 1){
                dx = -0.5;
            }
            else if (next_hit_check[0] == 0 && current_x > vertex2_x && current_y < vertex1_y){
                dx = -0.5;
            }
            else{
                dx = 0.5;
            }
        }
        else if (incoming_dy > 0){
            next_hit_check = Check_Collision(problem, current_x, current_y, current_x - 0.5, current_y, path);
            if(next_hit_check[0] == 1){
                dx = 0.5;
            }
            else if (next_hit_check[0] == 0 && current_x < vertex1_x && current_y > vertex2_y){
                dx = 0.5;
            }
            else{
                dx = -0.5;
            }
        }
        dy = 0;
    }
    else{
        //std::cout << "why am I here\n";
        line[0] = m;
        line[1] = b;
        // if(current_x >= std::max(vertex1_x, vertex2_x)){
        //     dx = -0.5;
        // }
        // else{
        //     dx = 0.5;
        // }
        next_x = current_x + 0.5; //incoming_dx;
        next_y = m * next_x + b;
        dy = next_y - current_y;
    }
    // if(trunc(m) == 0){
    //     std::cout << "need a case for this \n";
    // }

    while(true){ //movement loop
        
        if(path.waypoints.size() > 5000){
            path.waypoints.push_back(Eigen::Vector2d(problem.q_goal[0], problem.q_goal[1]));
            return;
        }   

        ////std::cout << "last leave point in follow loop " << last_leave_point << "\n";

        ////std::cout << "current position " << current_x << " " << current_y << "\n";
        // //std::cout << "next step will be at " << current_x + dx << " " << current_y + dy << "\n";
        ////std::cout << "current dx is  " << dx << " current dy is " << dy << "\n";

        bool test1 = trunc( 10* (current_x * m + b)) == trunc(10 * current_y);
        bool test2 = current_x != initial_x;
        bool test3 = current_x < problem.q_goal[0];
        bool test4 = trunc(10 * current_x) > trunc(10 * last_leave_point);
        bool test5 = current_x != initial_x || current_y != initial_y;
        bool test6 = (trunc(10 * current_x) == trunc(10 * problem.q_goal[0])) && trunc(10 * current_y) >= trunc(10 * std::max(vertex1_y, vertex2_y));
        
        path.waypoints.push_back(Eigen::Vector2d(current_x, current_y));
        //sleep(1)

        ///std::cout << "check which condition is not true to leave " <<  test1 << " " << test2 << " " << test3 << " " << test4 << "\n";
        
        if(m == 1000 && b == 1001 && test5 && test6){
            return;
        }
        // else if (m == 0 && trunc(10 * current_y) == trunc(10 * problem.q_goal[1]) && trunc(10 * current_x) >= trunc(10 * std::max(vertex1_x, vertex2_x)) && current_x > initial_x){
        //     return;
        // }
        else if(trunc( 10* (current_x * m + b)) == trunc(10 * current_y) && test5 && current_x <= problem.q_goal[0] && trunc(10 * current_x) > trunc(10 * last_leave_point)){
            //std::cout << "I have identified I am on the mline and no returning out of follow\n";
            return;
        }
        
        //std::cout << "minimum x for obstacle that I am following  " << std::min(vertex1_x, vertex2_x) << "\n";
        
        //sleep(1);
        follow_collision = Check_Collision(problem, current_x, current_y, current_x + dx, current_y + dy, path);


        if (follow_collision[0] == 1){
            //std::cout << "-----------------I am reporting a collision would happen on the next step-----------------------  " << "\n";
            follow(problem, current_x, current_y, current_x, current_y, dx, dy, last_leave_point, m, b, follow_collision, path);
            return;
        }

        //std::cout << "current position " << current_x << " " << current_y << "\n";
    
        //sleep(1);

        if(dy > 0 && current_y > std::max(vertex1_y, vertex2_y)){
            ////std::cout << "current y in break = " << current_y << " max y of vertices is " << std::max(vertex1_y, vertex2_y) << "\n";
            //std::cout << "BREAK 1 " << "\n";
            break;
        }
        else if (dy < 0 && current_y < std::min(vertex1_y, vertex2_y)){
           //std::cout << "BREAK 2 " << "\n";
            break;
        }
        else if (dx > 0 && current_x > std::max(vertex1_x, vertex2_x)){
           //std::cout << "BREAK 3 " << "\n";
            break;
        }
        else if (dx < 0 && current_x < std::min(vertex1_x, vertex2_x)){
           //std::cout << "BREAK 4 " << "\n";
            break;
        }
    
        current_x += dx;
        current_y += dy;    
    }

    //std::cout << "before the if statement vertex " << info[2] <<" and " <<info[3]<<"\n";
    if(info[2] == 0){
        if(dx > 0){
            info[2] = 1;
            info[3] = 2;
        }
        else{
            info[2] = 3;
            info[3] = 0;
        }
    }
    else if (info[2] == 1){
        if(dy > 0){
            info[2] = 2;
            info[3] = 3;
        }
        else{
            info[2] = 0;
            info[3] = 1;
        }
    }
    else if (info[2] == 2){
        //std::cout << "I now that vertex 1 is corner 2\n";
        if(dx > 0){
            //std::cout<<"I recognize dx bigger than 0\n";
            info[2] = 1;
            info[3] = 2;
            //std::cout << "I have set the vertices that I now want to follow to " << info[2] << " and "<<info[3] << "\n";
        }
        else{
            //std::cout<<" 1 \n";
            info[2] = 3;
            info[3] = 0;
        }
    }
    else if (info[2] == 3){
        if(dy < 0){
            //std::cout<<" 2 \n";
            info[2] = 0;
            info[3] = 1;
        }
        else{
            //std::cout<<" 3 \n";
            info[2] = 2;
            info[3] = 3;
        }
    }

    //std::cout << "next follow is around vertex " << info[2] <<" and " <<info[3]<<"\n";
    follow(problem, current_x, current_y, initial_x, initial_y, dx, dy, last_leave_point, m, b, info, path);

    return;

}


float MyBugAlgorithm::dist_between_two_points(float x1, float y1, float x2, float y2){

    float dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    return dist;

}


std::vector<float> MyBugAlgorithm::equation_of_line(float x1, float y1, float x2, float y2){
    std::vector<float> ans;
    if(x2 == x1){
        ans.insert(ans.begin(), 1000);
        ans.insert(ans.begin() + 1, 1001);
        if(y1 < y2){
            ans.insert(ans.begin() + 2, 1);
        }
        else if (y1 > y2){
            ans.insert(ans.begin() + 2, 0);
        }
    }
    else{
    ans.insert(ans.begin(), (y2 - y1) / (x2 - x1));
    ans.insert(ans.begin() + 1, y1 - ans[0] * x1);
    ans.insert(ans.begin() + 2, 3);
    }
    
    return ans;

}

void MyBugAlgorithm::follow_bug1(const amp::Problem2D& problem, float current_x, float current_y, float initial_x, float initial_y, float incoming_dx, float incoming_dy, std::vector<std::vector<float>>& location_mem, float & m, float & b, std::vector<float> info, amp::Path2D& path, int & break_flag){

    //idk if this needs to be here
    // if(trunc(10 * (current_x * m + b)) == trunc(10 * current_y) && current_x != initial_x && current_x < problem.q_goal[0] && trunc(10 * current_x) > trunc(10 * last_leave_point)){
    //     return;
    // }
    
    std::vector<float> line;
    std::vector<float> follow_collision, next_hit_check;

    // location_mem[0][0] = current_x;
    // location_mem[0][1] = current_y;
    // location_mem[0][2] = dist_between_two_points(current_x, current_y, problem.q_goal[0], problem.q_goal[1]);
    
    float dy, dx = 0;
    float next_x, next_y;
    
   //std::cout<<"goint to assign vertices to line of object to follow\n";
    float vertex1_x = problem.obstacles[info[1]].verticesCCW()[info[2]][0];
    //std::cout<<"vert1\n";
    float vertex1_y = problem.obstacles[info[1]].verticesCCW()[info[2]][1];
    //std::cout<<"vert2\n";
    float vertex2_x = problem.obstacles[info[1]].verticesCCW()[info[3]][0];
    //std::cout<<"vert3\n";
    float vertex2_y = problem.obstacles[info[1]].verticesCCW()[info[3]][1];
    //std::cout<<"vertices have been assigned\n";
    line = equation_of_line(vertex1_x, vertex1_y, vertex2_x, vertex2_y);
   //std::cout << "I AM NOW FOLLOWING A NEW OBJECT LINE WITH THE FOLLOWING VERTICES " << "\n";
   //std::cout << "vertix 1 locations x: " << vertex1_x << " y: " << vertex1_y << "\n";
   //std::cout << "vertix 2 locations x: " << vertex2_x << " y: " << vertex2_y << "\n";
   //std::cout << "incoming dx is : " << incoming_dx << " current position is  " << current_x << " and " << current_y << "\n";

    if (line[0] == 1000 && line[1] == 1001 ){
        
        if(incoming_dx < 0){
            next_hit_check = Check_Collision(problem, current_x, current_y, current_x, current_y - 0.5, path);
            if(next_hit_check[0] == 1){
                dy = 0.5;
            }
            else if (next_hit_check[0] == 0 && current_x <= vertex1_x && current_y <= vertex1_y){
                dy = 0.5;
            }
            else{
                dy = -0.5;
            }
        }
        else if (incoming_dx > 0){
            next_hit_check = Check_Collision(problem, current_x, current_y, current_x, current_y + 0.5, path);
            if(next_hit_check[0] == 1){
                dy = -0.5;
            }
            else if (next_hit_check[0] == 0 && current_x > vertex2_x && current_y > vertex2_y){
                dy = -0.5;
            }
            else{
                dy = 0.5;
            }
        }
        dx = 0;
    }
    else if (line[0] == 0){
        if(incoming_dy < 0){
            next_hit_check = Check_Collision(problem, current_x, current_y, current_x + 0.5, current_y, path);
            if(next_hit_check[0] == 1){
                dx = -0.5;
            }
            else if (next_hit_check[0] == 0 && current_x > vertex2_x && current_y < vertex1_y){
                dx = -0.5;
            }
            else{
                dx = 0.5;
            }
        }
        else if (incoming_dy > 0){
            next_hit_check = Check_Collision(problem, current_x, current_y, current_x - 0.5, current_y, path);
            if(next_hit_check[0] == 1){
                dx = 0.5;
            }
            else if (next_hit_check[0] == 0 && current_x < vertex1_x && current_y > vertex2_y){
                dx = 0.5;
            }
            else{
                dx = -0.5;
            }
        }
        dy = 0;
    }
    else{
        line[0] = m;
        line[1] = b;
        next_x = current_x + 0.5;
        next_y = m * next_x + b;
        dy = next_y - current_y;
    }

    while(true){ //movement loop

       //std::cout << "current position " << current_x << " " << current_y << "\n";
        //std::cout << "next step will be at " << current_x + dx << " " << current_y + dy << "\n";
        //std::cout << "current dx is  " << dx << " current dy is " << dy << "\n";
        //sleep(1);

        if(path.waypoints.size() > 5000){
            path.waypoints.push_back(Eigen::Vector2d(problem.q_goal[0], problem.q_goal[1]));
            return;
        }  
        
        // bool test1 = trunc( 10* (current_x * m + b)) == trunc(10 * current_y);
        // bool test2 = current_x != initial_x;
        // bool test3 = current_x < problem.q_goal[0];
        // //bool test4 = trunc(10 * current_x) > trunc(10 * last_leave_point);
        // bool test5 = current_x != initial_x || current_y != initial_y;
        
        path.waypoints.push_back(Eigen::Vector2d(current_x, current_y));
        //sleep(1);
        location_mem.push_back(std::vector{current_x, current_y, dist_between_two_points(current_x, current_y, problem.q_goal[0], problem.q_goal[1])});

       //std::cout << "check which condition is not true to leave " <<  (trunc(10 * current_x) == trunc(10 * initial_x)) << " " << (trunc(10 * current_y) == trunc(10 * initial_y)) << " " << (location_mem.size() > 2) << "\n";


        if(trunc(10 * current_x) == trunc(10 * initial_x) && trunc(10 * current_y) == trunc(10 * initial_y) && location_mem.size() > 2){

           //std::cout << "terminating the follow loop with break flag =  " << break_flag << "\n";
            float min_dist = location_mem[0][2];
            int indx_for_min = 0;

            for (int i = 0; i < location_mem.size(); i++){
                if(location_mem[i][2] < min_dist){
                    min_dist = location_mem[i][2];
                    indx_for_min = i;
                }
            }
            
           //std::cout << "point that minimizes distance " << location_mem[indx_for_min][0] << " and " << location_mem[indx_for_min][1] << "\n";
            //sleep(20);
            if(info[2] == 0){
                if(dx > 0){
                    info[2] = 1;
                    info[3] = 2;
                }
                else{
                    info[2] = 3;
                    info[3] = 0;
                }
            }
            else if (info[2] == 1){
                if(dy > 0){
                    info[2] = 2;
                    info[3] = 3;
                }
                else{
                    info[2] = 0;
                    info[3] = 1;
                }
            }
            else if (info[2] == 2){
                //std::cout << "I now that vertex 1 is corner 2\n";
                if(dx > 0){
                    //std::cout<<"I recognize dx bigger than 0\n";
                    info[2] = 1;
                    info[3] = 2;
                    //std::cout << "I have set the vertices that I now want to follow to " << info[2] << " and "<<info[3] << "\n";
                }
                else{
                    //std::cout<<" 1 \n";
                    info[2] = 3;
                    info[3] = 0;
                }
            }
            else if (info[2] == 3){
                if(dy < 0){
                    //std::cout<<" 2 \n";
                    info[2] = 0;
                    info[3] = 1;
                }
                else{
                    //std::cout<<" 3 \n";
                    info[2] = 2;
                    info[3] = 3;
                }
            }

            if(break_flag == 1){
                return;
            }
            else{
                break_flag = 1;
                follow_bug1(problem, current_x, current_y, location_mem[indx_for_min][0], location_mem[indx_for_min][1], dx, dy, location_mem, m, b, info, path, break_flag);
                return;
            }
        }
        
        //std::cout << "minimum x for obstacle that I am following  " << std::min(vertex1_x, vertex2_x) << "\n";
        
        //sleep(1);


        follow_collision = Check_Collision(problem, current_x, current_y, current_x + dx, current_y + dy, path);


        if (follow_collision[0] == 1){
            //std::cout << "-----------------I am reporting a collision would happen on the next step-----------------------  " << "\n";
            follow_bug1(problem, current_x, current_y, initial_x, initial_y, dx, dy, location_mem, m, b, follow_collision, path, break_flag);
            return;
        }

        //std::cout << "current position " << current_x << " " << current_y << "\n";
    
        //sleep(1);

        if(dy > 0 && current_y > std::max(vertex1_y, vertex2_y)){
            ////std::cout << "current y in break = " << current_y << " max y of vertices is " << std::max(vertex1_y, vertex2_y) << "\n";
            //std::cout << "BREAK 1 " << "\n";
            break;
        }
        else if (dy < 0 && current_y < std::min(vertex1_y, vertex2_y)){
           //std::cout << "BREAK 2 " << "\n";
            break;
        }
        else if (dx > 0 && current_x > std::max(vertex1_x, vertex2_x)){
           //std::cout << "BREAK 3 " << "\n";
            break;
        }
        else if (dx < 0 && current_x < std::min(vertex1_x, vertex2_x)){
           //std::cout << "BREAK 4 " << "\n";
            break;
        }
    
        current_x += dx;
        current_y += dy;    
    }

    //std::cout << "before the if statement vertex " << info[2] <<" and " <<info[3]<<"\n";
    if(info[2] == 0){
        if(dx > 0){
            info[2] = 1;
            info[3] = 2;
        }
        else{
            info[2] = 3;
            info[3] = 0;
        }
    }
    else if (info[2] == 1){
        if(dy > 0){
            info[2] = 2;
            info[3] = 3;
        }
        else{
            info[2] = 0;
            info[3] = 1;
        }
    }
    else if (info[2] == 2){
        //std::cout << "I now that vertex 1 is corner 2\n";
        if(dx > 0){
            //std::cout<<"I recognize dx bigger than 0\n";
            info[2] = 1;
            info[3] = 2;
            //std::cout << "I have set the vertices that I now want to follow to " << info[2] << " and "<<info[3] << "\n";
        }
        else{
            //std::cout<<" 1 \n";
            info[2] = 3;
            info[3] = 0;
        }
    }
    else if (info[2] == 3){
        if(dy < 0){
            //std::cout<<" 2 \n";
            info[2] = 0;
            info[3] = 1;
        }
        else{
            //std::cout<<" 3 \n";
            info[2] = 2;
            info[3] = 3;
        }
    }

    //std::cout << "next follow is around vertex " << info[2] <<" and " <<info[3]<<"\n";
    follow_bug1(problem, current_x, current_y, initial_x, initial_y, dx, dy, location_mem, m, b, info, path, break_flag);

    return;

}
    