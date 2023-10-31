#include "HelpfulClass.h"

Eigen::Vector2d MyLinkManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const{
    //std::cout << "JOINT LOCATION \n";
    std::vector<double> links = this->getLinkLengths();
    //std::cout << "JOINT LOCATION \n";
    Eigen::Vector2d base_loc = this->getBaseLocation();
    //std::cout << "GOT BASE LOCATION \n";
    
    double x_sum = base_loc[0];
    double y_sum = base_loc[1];
    double angle_sum = 0;
    //std::cout << "joint index " << joint_index << " size of state " << state.size() << " size of links " << links.size() << " size of base vec " << base_loc.size() << "\n";
    for (int i = 0; i < joint_index; i++){

        angle_sum = angle_sum + state[i];

        x_sum = x_sum + links[i] * cos(angle_sum);
        y_sum = y_sum + links[i] * sin(angle_sum);
        
    }   
    Eigen::Vector2d ans(x_sum,y_sum);

    return ans;

}

amp::ManipulatorState MyLinkManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
    //std::cout << "inside IK function\n";
    std::vector<double> links = this->getLinkLengths();
    Eigen::Vector2d base_loc = this->getBaseLocation();
    //std::cout << "got link lengths\n";
    double l1 = links[0];
    double l2 = links[1];
    double p3_x, p3_y;

    p3_x = end_effector_location[0];
    p3_y = end_effector_location[1];
    //std::cout << "end effector location I am trying to calculate for " << p3_x << " " << p3_y << "\n";
    double t2 = acos( (1/(2 * l1 * l2)) * ( (pow(p3_x,2) + pow(p3_y,2)) - (pow(l1,2) + pow(l2,2)) ) );

    double t1 = asin( (1/(pow(p3_x,2) + pow(p3_y,2))) * (p3_y * (l1 + l2 * cos(t2)) - p3_x * l2 * sqrt(1 - pow(cos(t2),2)) ) );

    if(t1 == 0 && base_loc[0] > end_effector_location[0]){
        t1 = 3.141592653589793238463;
    }
    
    //amp::ManipulatorState ans;
    //std::cout << "end effector location I calculated " << t1 << " " << t2 << "\n";
    Eigen::VectorXd ans(2); 
    ans << t1, t2;
    
    return ans;

}


int MyGridCSpace2D_ForLinks::orientation(double x1, double y1, double x2, double y2, double x3, double y3) {

    float turn = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
    if (turn > 0){ //clockwise
        return 1;
    }
    else if (turn < 0){//counter clockwise 
        return 2;
    }
    else return 0; //showing they are colinear 

}

bool MyGridCSpace2D_ForLinks::CollisionCheck(double x0, double x1){
    //std::cout << "angle x " << x0 << " angle y " << x1 << "\n";
    MyLinkManipulator2D link_obj(this->link_lengths);
    
    std::vector<double> l = link_obj.getLinkLengths();
    //std::cout << "link lengths in collision check are " << l[0] << " " << l[1] << "\n";
    //std::cout << "I AM GETTING TO RIGHT HERE\n";
    Eigen::VectorXd state(2);
    state << x0 * 3.141592653589793238463/180.0, x1 * 3.141592653589793238463/180.0;
   
    //uint32_t joint_index = 2;
    Eigen::Vector2d base = link_obj.getJointLocation(state, 0);
    Eigen::Vector2d point1 = link_obj.getJointLocation(state, 1);
    Eigen::Vector2d end_point = link_obj.getJointLocation(state, 2);

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
        //std::cout << "looking at obstacle " << i << "\n";
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
                    //std::cout << "we have detected link collision\n";
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

std::pair<std::size_t, std::size_t> MyGridCSpace2D_ForLinks::getCellFromPoint(double x0, double x1) const{
    std::pair<std::size_t, std::size_t> ans;
    return ans;

}


bool PointCollisionCheck(double x, double y, const amp::Problem2D& problem){
    
    double min_x, max_x, min_y, max_y;    

    for (int i = 0; i < problem.obstacles.size(); i++){
        //std::cout << "looking at obstacle " << i << "\n";
        min_x = problem.obstacles[i].verticesCCW()[0][0];
        max_x = problem.obstacles[i].verticesCCW()[0][0];
        min_y = problem.obstacles[i].verticesCCW()[0][1];
        max_y = problem.obstacles[i].verticesCCW()[0][1];
        
        for (int j = 0; j < problem.obstacles[i].verticesCCW().size(); j ++){

            if (problem.obstacles[i].verticesCCW()[j][0] < min_x){
                min_x = problem.obstacles[i].verticesCCW()[j][0];
            }
            else if (problem.obstacles[i].verticesCCW()[j][0] > max_x){
                max_x = problem.obstacles[i].verticesCCW()[j][0];
            }
            else if (problem.obstacles[i].verticesCCW()[j][1] < min_y){
                min_y = problem.obstacles[i].verticesCCW()[j][1];
            }
            else if (problem.obstacles[i].verticesCCW()[j][1] > max_y){
                max_y = problem.obstacles[i].verticesCCW()[j][1];
            }

        }
        
        if (x >= min_x && x <= max_x && y >= min_y && y <= max_y){ //this is where we need collision logic for end effector

            int cw = 0;
            int ccw = 0;

            for(int k = 0; k < problem.obstacles[i].verticesCCW().size(); k++){
                
                double x2, y2, cross, x1, y1;

                if (k == problem.obstacles[i].verticesCCW().size() - 1){

                    x2 = problem.obstacles[i].verticesCCW()[0][0];
                    y2 = problem.obstacles[i].verticesCCW()[0][1];
                }
                else{

                    x2 = problem.obstacles[i].verticesCCW()[k + 1][0];
                    y2 = problem.obstacles[i].verticesCCW()[k + 1][1];

                }

                x1 = problem.obstacles[i].verticesCCW()[k][0];
                y1 = problem.obstacles[i].verticesCCW()[k][1];

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

bool ConnectionCollisionCheck(Eigen::Vector2d p1, Eigen::Vector2d p2, const amp::Problem2D& problem){
    
    double min_x, max_x, min_y, max_y;

    double x_1 = p1[0];
    double y_1 = p1[1];
    double x_2 = p2[0];
    double y_2 = p2[1];

    for (int i = 0; i < problem.obstacles.size(); i++){
        //std::cout << "looking at obstacle " << i << "\n";
        min_x = problem.obstacles[i].verticesCCW()[0][0];
        max_x = problem.obstacles[i].verticesCCW()[0][0];
        min_y = problem.obstacles[i].verticesCCW()[0][1];
        max_y = problem.obstacles[i].verticesCCW()[0][1];
        
        for (int j = 0; j < problem.obstacles[i].verticesCCW().size(); j ++){

            if (problem.obstacles[i].verticesCCW()[j][0] < min_x){
                min_x = problem.obstacles[i].verticesCCW()[j][0];
            }
            else if (problem.obstacles[i].verticesCCW()[j][0] > max_x){
                max_x = problem.obstacles[i].verticesCCW()[j][0];
            }
            else if (problem.obstacles[i].verticesCCW()[j][1] < min_y){
                min_y = problem.obstacles[i].verticesCCW()[j][1];
            }
            else if (problem.obstacles[i].verticesCCW()[j][1] > max_y){
                max_y = problem.obstacles[i].verticesCCW()[j][1];
            }

        }
        
        if( std::min(x_1, x_2) < max_x && std::min(y_1, y_2) < max_y){
            for(int iter = 0; iter < problem.obstacles[i].verticesCCW().size(); iter++){
                double vertex_x = problem.obstacles[i].verticesCCW()[iter][0];
                double vertex_y = problem.obstacles[i].verticesCCW()[iter][1];
                double next_vertex_x, next_vertex_y;
                
                if(iter == problem.obstacles[i].verticesCCW().size() - 1){
                    next_vertex_x = problem.obstacles[i].verticesCCW()[0][0];
                    next_vertex_y = problem.obstacles[i].verticesCCW()[0][1];
                }
                else{
                    next_vertex_x = problem.obstacles[i].verticesCCW()[iter + 1][0];
                    next_vertex_y = problem.obstacles[i].verticesCCW()[iter + 1][1];
                }

                int orient1 = orientation(x_1, y_1, x_2, y_2, vertex_x, vertex_y);
                int orient2 = orientation(x_1, y_1, x_2, y_2, next_vertex_x, next_vertex_y);
                int orient3 = orientation(vertex_x, vertex_y, next_vertex_x, next_vertex_y, x_1, y_1);
                int orient4 = orientation(vertex_x, vertex_y, next_vertex_x, next_vertex_y, x_2, y_2);

                if (orient1 != orient2 && orient3 != orient4){
                    return true;            
                }                    
            }
        }
    }

    return false;
}

double DistanceBetweenNodes(Eigen::Vector2d n1, Eigen::Vector2d n2){
    return sqrt(pow(n2[0] - n1[0] , 2) + pow(n2[1] - n1[1] , 2));
}

int orientation(double x1, double y1, double x2, double y2, double x3, double y3) {

    float turn = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
    if (turn > 0){ //clockwise
        return 1;
    }
    else if (turn < 0){//counter clockwise 
        return 2;
    }
    else return 0; //showing they are colinear 

}


amp::AStar::GraphSearchResult NewMyAstar::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic){

    amp::AStar::GraphSearchResult ans; //= new GraphSearchResult();
    //problem.graph->print();

    std::vector<amp::Node> open_set = {problem.init_node};
    std::vector<amp::Node> closed_set = {};
    std::vector<double> g_score(problem.graph.get()->nodes().size(), std::numeric_limits<double>::infinity());
    
    std::vector<double> f_score(problem.graph.get()->nodes().size(), std::numeric_limits<double>::infinity());
    //std::cout << "size of fscore is " << problem.graph.get()->nodes().size() << "\n";
    std::vector<amp::Node> came_from(problem.graph.get()->nodes().size());
    f_score[problem.init_node] = heuristic.operator()(problem.init_node);
    g_score[problem.init_node] = 0;
    int number_of_iterations = 0;
    
    while(!open_set.empty()){
        //std::cout << "still in while loop\n";
        double lowest_f_score = f_score[open_set[0]];
        int next_node_index = 0;
        //std::cout << "got f score\n";

        for(int i = 0; i < open_set.size(); i++){
            //std::cout << "trying to get f score of node " << i << "\n";
            double challenge_f_score = f_score[open_set[i]];
            //std::cout << "able to get f score\n";

            if(challenge_f_score < lowest_f_score){
                lowest_f_score = challenge_f_score;
                next_node_index = i;
            }
        }

        amp::Node current_node = open_set[next_node_index];
        //std::cout << "current node  " << current_node << "  goal node  " << problem.goal_node << "\n";
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
            //std::cout << "number of iterations to get to goal is " << number_of_iterations << "\n";
            //std::cout << "success\n";
            return ans;
        }
        
        open_set.erase(open_set.begin() + next_node_index);

        std::vector<amp::Node> children_of_current_node = problem.graph.get()->children(current_node);
        std::vector<double> edge_weights_from_current_node = problem.graph.get()->outgoingEdges(current_node);            

        for(int i = 0; i < children_of_current_node.size(); i++){
            //std::cout << "we are at child " << i << "\n";
            double tentative_gscore = g_score[current_node] + edge_weights_from_current_node[i];
            //std::cout << "I am at node  " << current_node << "  child of current node  " << children_of_current_node[i] << " tentative g score is " << tentative_gscore << "  g score current node " << g_score[current_node]<< "  plus  " <<edge_weights_from_current_node[i]<< "\n";
            //std::cout << "gscore of child is valid " << g_score[children_of_current_node[i]] << "\n";
            if(tentative_gscore < g_score[children_of_current_node[i]]){
                //std::cout << "in if\n";
                came_from[children_of_current_node[i]] = current_node;
                //std::cout << "came from ok\n";
                g_score[children_of_current_node[i]] = tentative_gscore;
                //std::cout << "g score ok\n";
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
    //std::cout << "fail\n";
    std::list<amp::Node> fail_path = {problem.init_node};
    //std::cout << "fail path created\n";
    ans.success = false;
    //std::cout << "ans success asigned\n";
    ans.node_path = fail_path;
    //std::cout << "fail path set\n";
    ans.path_cost = 0;
    //std::cout << "returning from inside astar\n";

    return ans;
}

