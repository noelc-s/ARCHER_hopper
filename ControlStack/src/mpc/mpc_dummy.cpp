#include "../../inc/mpc/mpc_interface.h"

class MPCDummy : public MPCInterface
{
public:

    MPCDummy() {}

    int solve(vector_t &sol, vector_3t &command, vector_2t &command_interp) override
    {
        return -1;
    }
};

std::unique_ptr<MPCInterface> createMPCInstance()
{
    return std::make_unique<MPCDummy>();
}