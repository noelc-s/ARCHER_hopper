#pragma once

#include "../Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>

using namespace Hopper_t;

class PlannerInterface
{
public:
    virtual ~PlannerInterface() = default;

    virtual void update(vector_t &starting_loc, vector_t &ending_loc, scalar_t& time,
                std::atomic<bool> &running, bool &planner_initialized) = 0;
    virtual vector_t getPath(scalar_t time, scalar_t des_yaw) = 0;

    virtual bool isEstimateInitialized() = 0;
};

// Factory function for creating the implementation
std::unique_ptr<PlannerInterface> createPlannerInstance();
