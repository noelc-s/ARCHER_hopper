#pragma once

#include "../Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>
#include "../../inc/estimatedState.h"

using namespace Hopper_t;

class OTInterface
{
public:
    virtual ~OTInterface() = default;

};

// Factory function for creating the implementation
std::unique_ptr<OTInterface> createOTInstance(std::shared_ptr<EstimatedState> optiState);
