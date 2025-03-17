#include "../../inc/planner/planner_interface.h"
#include "pathPlanner.h"

class PlannerWrapper : public PlannerInterface
{
public:
    struct PlannerTiming
    {
        double cut;
        double findPath;
        double refinement;
    } plannerTiming;

    PlannerTiming meanTiming;
    PlannerTiming stdTiming;

    std::unique_ptr<PathPlanner> planner;
    ObstacleCollector O_;

    std::deque<double> cutTimingWindow;
    std::deque<double> pathTimingWindow;
    std::deque<double> mpcTimingWindow;
    const int window_size = 100;

    vector_t planned_command_;
    scalar_t t_planner_last_ = 0;

    ObstacleCollector generateObstacle()
    {
        ObstacleCollector O = ObstacleCollector();

        Obstacle obs;
        obs.center.resize(2);
        obs.center.setZero();
        obs.v.resize(4, 2);
        obs.A.resize(4, 4);
        obs.b.resize(4);
        obs.Adjacency.resize(4, 4);
        obs.Adjacency << 1, 0, 0, 1,
            1, 1, 0, 0,
            0, 1, 1, 0,
            0, 0, 1, 1;

        vector_t obst(8);
        obst.setZero();
        obst << 0.5, -0.5,
            1.5, -0.5,
            1.5, 0.5,
            0.5, 0.5;

        obs.v << obst[0], obst[1],
                obst[2], obst[3],
                obst[4], obst[5],
                obst[6], obst[7];

        std::vector<Eigen::Vector2d> edgeVectors(4);
        std::vector<Eigen::Vector2d> normals(4);

        // Compute edge vectors
        // 0,1
        // 2,3
        // 4,5
        // 6,7
        edgeVectors[0] = Eigen::Vector2d(obst[6] - obst[0], obst[7] - obst[1]);
        edgeVectors[1] = Eigen::Vector2d(obst[0] - obst[2], obst[1] - obst[3]);
        edgeVectors[2] = Eigen::Vector2d(obst[2] - obst[4], obst[3] - obst[5]);
        edgeVectors[3] = Eigen::Vector2d(obst[4] - obst[6], obst[5] - obst[7]);
        edgeVectors[0].normalize();
        edgeVectors[1].normalize();
        edgeVectors[2].normalize();
        edgeVectors[3].normalize();

        // Construct A and b
        vector_t tmp(4);
        tmp.setZero();
        for (int i = 0; i < 4; ++i)
        {
            obs.A.row(i) << -edgeVectors[i].transpose(), 0, 0;
            obs.b(i) = -edgeVectors[i].transpose().dot(Eigen::Vector2d(obst[2 * i], obst[2 * i + 1]));
        }
        std::vector<Obstacle> obstacles;
        obstacles.push_back(obs);
        O.obstacles = obstacles;
        return O;
    }

    PlannerWrapper()
    {
        Params params;
        MPC_Params mpc_params;
        Planner_Params planner_params;
        loadPlannerParams("../config/planner_params.yaml", params, mpc_params, planner_params);

        planned_command_.resize(4 * mpc_params.N);
        planned_command_.setZero();

        const int state_size = 4;
        const int input_size = 2;

        planner = std::make_unique<PathPlanner>(state_size, input_size, mpc_params, planner_params);
        O_ = generateObstacle();
        planner->initialize(O_);
    }

    // Function to compute mean
    double compute_mean(const std::deque<double> &window)
    {
        return std::accumulate(window.begin(), window.end(), 0.0) / window.size();
    }

    // Function to compute standard deviation
    double compute_stddev(const std::deque<double> &window, double mean)
    {
        double variance = 0.0;
        for (double val : window)
        {
            variance += (val - mean) * (val - mean);
        }
        return std::sqrt(variance / window.size());
    }

    vector_t getPath(scalar_t time, scalar_t des_yaw) {
        vector_t x1_x2(8);
        int index = 1;
        x1_x2 << planned_command_.segment(4 * index,8);
        matrix_t mul = planner->Bez_*x1_x2;
        matrix_t controlPoints = Eigen::Map<matrix_t>(mul.data(),4,4).transpose();
        scalar_t bez_t = time - t_planner_last_;

        vector_t path_command(5);
        path_command << planner->B->b(bez_t, controlPoints).transpose(), des_yaw;
        return path_command;
    }

    void update(vector_t &starting_loc, vector_t &ending_loc, scalar_t& time, std::atomic<bool> &running, bool &planner_initialized) override
    {
        Timer timer(false);
        std::ofstream graph_file = open_log_file("../stored_graph.m");
        std::ofstream output_file = open_log_file("../output.m");
        log(planner->points, graph_file, "Points");
        log(planner->edges, graph_file, "EdgeControlPoints");

        std::condition_variable cv2;
        std::mutex m2;

        vector_t graph_sol;
        int max_graph_sol_length = planner->params_.max_graph_sol_length;
        graph_sol.resize(4 * max_graph_sol_length);

        // std::thread cutGraph(static_cast<void (PathPlanner::*)(ObstacleCollector&, std::ofstream&, double&, std::condition_variable&, std::mutex&)>(&PathPlanner::cutGraphLoop),
        //             planner.get(), std::ref(O), std::ref(output_file), std::ref(plannerTiming.cut), std::ref(cv2), std::ref(m2));
        // sleep(1);

        while (running)
        {
            timer.start();
            if (planner->params_.log_edges)
            {
                planner->cutGraph(O_, output_file, cv2, m2);
            }
            else
            {
                planner->cutGraph(O_, cv2, m2);
            }
            plannerTiming.cut = timer.time();

            std::vector<int> optimalInd;
            std::vector<vector_t> optimalPath;
            planner->findPath(O_.obstacles, starting_loc, ending_loc, optimalInd, optimalPath, cv2, m2);
            plannerTiming.findPath = timer.time();

            vector_t sol;
            planner->refineWithMPC(graph_sol, sol, O_, optimalInd, optimalPath, starting_loc, ending_loc);
            plannerTiming.refinement = timer.time();
            // {
            // std::lock_guard<std::mutex> lock(m);
            if (!(sol.segment(0, 4 * planner->mpc_->mpc_params_.N).array().isNaN().any()))
                planned_command_ << sol.segment(0, 4 * planner->mpc_->mpc_params_.N);
            // }

            cutTimingWindow.push_back(plannerTiming.cut);
            pathTimingWindow.push_back(plannerTiming.findPath);
            mpcTimingWindow.push_back(plannerTiming.refinement);

            if (cutTimingWindow.size() > window_size)
            {
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
            if (planner->params_.log_edges)
            {
                printf("Successfully logged edges.");
                exit(0);
            }
            planner_initialized = true;
            t_planner_last_ = time;
        }
    }
};

std::unique_ptr<PlannerInterface> createPlannerInstance()
{
    return std::make_unique<PlannerWrapper>();
}