#include "../../inc/Planner/Planner_interface.h"

class PlannerDummy : public PlannerInterface
{
public:

    PlannerDummy() {}

    void update(vector_t &starting_loc, vector_t &ending_loc, scalar_t& time, 
                std::atomic<bool> &running, bool &planner_initialized) override
    {
        return;
    }

    bool isEstimateInitialied() {
	    return false;
    }

    estimatedState getEstimatedState() {
	EstiamtedState est = {};
	return est;
    }

    vector_t getPath(scalar_t time, scalar_t des_yaw) {
        vector_t empty;
        return empty;
    }
};

std::unique_ptr<PlannerInterface> createPlannerInstance()
{
    return std::make_unique<PlannerDummy>();
}
