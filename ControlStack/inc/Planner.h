
#include "graph.h"
#include "obstacle.h"
#include "mpc.h"
#include "utils.h"
#include "pathPlanner.h"

class Planner {
public:

    struct PlannerTiming {
        double cut;
        double findPath;
        double refinement;
    } plannerTiming;

    PlannerTiming meanTiming;
    PlannerTiming stdTiming;

    Planner(ObstacleCollector O);

    std::unique_ptr<PathPlanner> planner;

    void update(ObstacleCollector &O, vector_t &starting_loc, vector_t &ending_loc, vector_t &planned_command, vector_t &graph_sol, int &index, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m);

    std::deque<double> cutTimingWindow;
    std::deque<double> pathTimingWindow;
    std::deque<double> mpcTimingWindow;
    const int window_size = 100;

};