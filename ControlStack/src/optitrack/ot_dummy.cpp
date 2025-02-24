#include "../../inc/optitrack/ot_interface.h"

class OTDummy : public OTInterface
{
public:

    OTDummy(std::shared_ptr<EstimatedState> optiState) {}
};

std::unique_ptr<OTInterface> createOTInstance(std::shared_ptr<EstimatedState> optiState)
{
    return std::make_unique<OTDummy>(optiState);
}