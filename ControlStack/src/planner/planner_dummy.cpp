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

    scalar_t getGraphDisc() {
        return 0.1;
    }
};

std::unique_ptr<PlannerInterface> createPlannerInstance(std::shared_ptr<vector_3t> goal_pose, 
                                                        std::shared_ptr<vector_3t> initial_pose,
                                                        std::shared_ptr<vector_3t> graph_center)
{
    return std::make_unique<PlannerDummy>();
}
