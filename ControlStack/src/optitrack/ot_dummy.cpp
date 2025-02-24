#include "../../inc/optitrack/ot_interface.h"

class OTDummy : public OTInterface
{
public:

    OTDummy() {}

    void getState(vector_t& state) override
    {
        return;
    }
};

std::unique_ptr<OTInterface> createOTInstance()
{
    return std::make_unique<OTDummy>();
}