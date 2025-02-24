#pragma once

#include "../Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>

using namespace Hopper_t;

struct OptiState {
    scalar_t x, y, z;
    scalar_t q_w, q_x, q_y, q_z;
    scalar_t x_dot, y_dot, z_dot;
};

class OTInterface
{
public:
    virtual ~OTInterface() = default;

};

// Factory function for creating the implementation
std::unique_ptr<OTInterface> createOTInstance();
