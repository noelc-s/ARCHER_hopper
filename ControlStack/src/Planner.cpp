#include "../inc/Planner.h"

Planner::Planner(ObstacleCollector O)
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

// Function to compute mean
double compute_mean(const std::deque<double>& window) {
    return std::accumulate(window.begin(), window.end(), 0.0) / window.size();
}

// Function to compute standard deviation
double compute_stddev(const std::deque<double>& window, double mean) {
    double variance = 0.0;
    for (double val : window) {
        variance += (val - mean) * (val - mean);
    }
    return std::sqrt(variance / window.size());
}

void Planner::update(ObstacleCollector &O, vector_t &starting_loc, vector_t &ending_loc, vector_t &planned_command, int &index, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m)
{
    Timer timer(false);
    std::ofstream graph_file = open_log_file("../stored_graph.m");
    std::ofstream output_file = open_log_file("../output.m");
    log(planner->points, graph_file, "Points");
    log(planner->edges, graph_file, "EdgeControlPoints");
    while (running)
    {
        timer.start();
        if (planner->params_.log_edges) {
            planner->cutGraph(O, output_file);
        } else {
            planner->cutGraph(O);
        }
        plannerTiming.cut = timer.time();

        std::vector<int> optimalInd;
        std::vector<vector_t> optimalPath;
        planner->findPath(O.obstacles, starting_loc, ending_loc, optimalInd, optimalPath);
        plannerTiming.findPath = timer.time();

        vector_t sol;
        planner->refineWithMPC(sol, O, optimalInd, optimalPath, starting_loc, ending_loc);
        plannerTiming.refinement = timer.time();
        {
            std::lock_guard<std::mutex> lock(m);
            if (!(sol.segment(0, 4 * planner->mpc_->mpc_params_.N).array().isNaN().any()))
                planned_command << sol.segment(0, 4 * planner->mpc_->mpc_params_.N);
        }

        cutTimingWindow.push_back(plannerTiming.cut);
        pathTimingWindow.push_back(plannerTiming.findPath);
        mpcTimingWindow.push_back(plannerTiming.refinement);

        if (cutTimingWindow.size() > window_size) {
            cutTimingWindow.pop_front();
            pathTimingWindow.pop_front();
            mpcTimingWindow.pop_front();
        }

        meanTiming.cut = compute_mean(cutTimingWindow);
        meanTiming.findPath = compute_mean(pathTimingWindow);
        meanTiming.refinement = compute_mean(mpcTimingWindow);

        stdTiming.cut = compute_stddev(cutTimingWindow, meanTiming.cut);
        stdTiming.findPath = compute_stddev(pathTimingWindow, meanTiming.findPath);
        stdTiming.refinement = compute_stddev(mpcTimingWindow, meanTiming.refinement);
        if (planner->params_.log_edges) {
            printf("Exiting because obstacle size is: %ld", O.obstacles.size());
            exit(0);
        }
    }
}