#include "AMPCore.h"

bool CollisionCheckGlider(float x, float y, float z, float L, float W, float H);
std::vector<float> IntegrateGlider(float x, float y, float z, float beta, float gamma, float v, float beta_d, float gamma_d, float Length, float Width, float Height, float timestep);
std::vector<float> PotentialPointGlider(float prob_goal, std::vector<float> goal, std::vector<std::vector<float> > limits);
float DistanceBetweenPointsGlider(float x, float y, float z, std::vector<float> point);
float DifferenceBetweenDirectionAndPoint(float beta, std::vector<float> next_point, std::vector<float> current_point);
amp::Node FindClosestNodeGlider(std::vector<float> next_positions, std::shared_ptr<amp::Graph<double> >& graph, std::map<amp::Node, std::vector<float> >& node_to_coord);
amp::Node FindClosestNodeGliderMovingGoal(std::vector<std::vector<float> > goal_vec, std::shared_ptr<amp::Graph<double> >& graph, std::map<amp::Node, std::vector<float> >& node_to_coord);

class GliderMyAstar : public amp::AStar{

    public:

    virtual amp::AStar::GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic);
};
