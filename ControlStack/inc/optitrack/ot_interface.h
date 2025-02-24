#pragma once

#include "../Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>

using namespace Hopper_t;

class OTInterface
{
public:
    virtual ~OTInterface() = default;

    virtual void getState(vector_t& state) = 0;

};

// Factory function for creating the implementation
std::unique_ptr<OTInterface> createOTInstance();
