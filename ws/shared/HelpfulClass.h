#include "AMPCore.h"

class MyLinkManipulator2D : public amp::LinkManipulator2D{

    public:

    MyLinkManipulator2D(const std::vector<double>& link_lengths)
    :  LinkManipulator2D(link_lengths){}

    MyLinkManipulator2D()
    :  LinkManipulator2D(){}

    Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const;
    amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const;

};

class MyGridCSpace2D_ForLinks : public amp::GridCSpace2D{

    public: 

    amp::Environment2D env;
    std::vector<double> link_lengths;

    MyGridCSpace2D_ForLinks(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
    : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max){}

    int orientation(double x1, double y1, double x2, double y2, double x3, double y3);
    bool CollisionCheck(double x0, double x1);
    virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const;

};

bool PointCollisionCheck(double x, double y, const amp::Problem2D& problem);
bool ConnectionCollisionCheck(Eigen::Vector2d p1, Eigen::Vector2d p2, const amp::Problem2D& problem);
double DistanceBetweenNodes(Eigen::Vector2d n1, Eigen::Vector2d n2);
int orientation(double x1, double y1, double x2, double y2, double x3, double y3);

class NewMyAstar : public amp::AStar{

    public:

    virtual amp::AStar::GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic);
};


std::vector<Eigen::Vector2d> SquareAgentCorners(Eigen::Vector2d center, double r);
Eigen::Vector2d PotentialPoint(double p_goal, const amp::MultiAgentProblem2D& problem, int robot);
bool PointCollisionCheckMultiAgent(double x, double y, const amp::MultiAgentProblem2D& problem);
Eigen::Vector2d NewCloserPoint(Eigen::Vector2d current_point, Eigen::Vector2d next_point, double step_size);
amp::Node FindClosestNode(std::vector<Eigen::Vector2d> next_positions, std::shared_ptr<amp::Graph<double> >& graph, std::map<amp::Node, std::vector<Eigen::Vector2d> >& node_to_coord, const amp::MultiAgentProblem2D& problem, std::vector<bool> robot_at_goal);
bool WillRobotsCollide(std::vector<Eigen::Vector2d> next_positions, const amp::MultiAgentProblem2D& problem);
amp::Node FindClosestNodeDecentralized(Eigen::Vector2d next_position, std::shared_ptr<amp::Graph<double> >& graph, std::map<amp::Node, Eigen::Vector2d>& node_to_coord);
bool WillRobotsCollideDecentralized(Eigen::Vector2d next_position, amp::MultiAgentPath2D& path, int current_agent, int time_step, double r);
bool NewConnectionCollisionCheck(Eigen::Vector2d p1, Eigen::Vector2d p2, const amp::MultiAgentProblem2D& problem);
void DoesRobotCollisionExist(amp::MultiAgentPath2D& path);