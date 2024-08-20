
#include "graph.h"
#include "obstacle.h"
#include "mpc.h"
#include "utils.h"
#include "pathPlanner.h"

class Planner {
public:

    Planner(Obstacle O);

    std::unique_ptr<PathPlanner> planner;

    void update(Obstacle &O, vector_t &starting_loc, vector_t &ending_loc, vector_t &planned_command, int &index, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m);

};