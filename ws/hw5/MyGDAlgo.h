#pragma once

#include "AMPCore.h"

class MyGDAlgo : public amp::GDAlgorithm{
    public:
    
    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

};