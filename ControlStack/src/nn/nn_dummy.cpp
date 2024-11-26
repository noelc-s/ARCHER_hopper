#include "../../inc/nn/nn_interface.h"

class NNDummy : public NNInterface
{
public:

    NNDummy() {}

    void evaluateNetwork(const vector_t& input, vector_t& ouptut) override
    {
        return;
    }
};

std::unique_ptr<NNInterface> createNNInstance(const std::string model_name)
{
    return std::make_unique<NNDummy>();
}