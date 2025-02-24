#include "../../inc/optitrack/ot_interface.h"

class OTDummy : public OTInterface
{
public:

    OTDummy() {}
};

std::unique_ptr<OTInterface> createOTInstance()
{
    return std::make_unique<OTDummy>();
}