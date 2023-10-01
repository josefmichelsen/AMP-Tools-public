#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        virtual std::vector<float> Check_Collision(const amp::Problem2D& problem, float current_x, float current_y, float next_x, float next_y, amp::Path2D& path) override;

        virtual int orientation(int x1, int y1, int x2, int y2, int x3, int y3) override;

        virtual void follow(const amp::Problem2D& problem, float current_x, float current_y, float initial_x, float initial_y, float incoming_dx, float incoming_dy, float last_leave_point, float m, float b, std::vector<float> info, amp::Path2D& path) override;
        
        virtual float dist_between_two_points(float current_x, float current_y, float goal_x, float goal_y) override;

        virtual std::vector<float> equation_of_line(float x1, float y1, float x2, float y2) override;

        virtual void follow_bug1(const amp::Problem2D& problem, float current_x, float current_y, float initial_x, float initial_y, float incoming_dx, float incoming_dy, std::vector<std::vector<float>>& location_mem, float & m, float & b, std::vector<float> info, amp::Path2D& path, int & break_flag) override;
    
    private:
        // Add any member variables here...
};