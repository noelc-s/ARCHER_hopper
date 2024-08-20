#include "../inc/Planner.h"

Planner::Planner(Obstacle O)
{

    Params params;
    MPC_Params mpc_params;
    Planner_Params planner_params;
    loadPlannerParams("../config/planner_params.yaml", params, mpc_params, planner_params);

    const int state_size = 4;
    const int input_size = 2;

    planner = std::make_unique<PathPlanner>(state_size, input_size, mpc_params, planner_params);
    planner->initialize(O);
}

void Planner::update(Obstacle &O, vector_t &starting_loc, vector_t &ending_loc, vector_t &planned_command, int &index, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m)
{
    while (running)
    {

        planner->cutGraph(O);

        std::vector<int> optimalInd;
        std::vector<vector_t> optimalPath;
        planner->findPath(starting_loc, ending_loc, optimalInd, optimalPath);

        vector_t sol;
        planner->refineWithMPC(sol, O, optimalInd, optimalPath, starting_loc, ending_loc);
        {
            std::lock_guard<std::mutex> lock(m);
            planned_command << sol.segment(0, 4 * planner->mpc_->mpc_params_.N);
        }
        index = 2;
    }
}