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