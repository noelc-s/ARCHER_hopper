#pragma once

#include "../Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>

using namespace Hopper_t;

class NNInterface
{
public:
    virtual ~NNInterface() = default;

    virtual void evaluateNetwork(const vector_t &input, vector_t& output) = 0;

};

// Factory function for creating the implementation
std::unique_ptr<NNInterface> createNNInstance(const std::string model_name);
