#ifndef ROM_H
#define ROM_H
#include "../inc/Types.h"
#include "../inc/UserInput.h"
#include <cmath>
#include "../inc/Hopper.h"
#include "mujoco.h"
#include "../inc/Policy.h"


using namespace Hopper_t;


class Command{
public: 
    virtual ~Command() {};
    virtual void update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state) = 0;
    virtual matrix_t getCommand() = 0;
    virtual int getHorizon() const = 0;
    virtual int getStateDim() const = 0;
};

class V5Command : public Command{
public: 
    V5Command(const scalar_t x0, const scalar_t y0);
    int getHorizon() const override { return 1; }
    int getStateDim() const override { return 3; }
    const scalar_t x0_, y0_;
    vector_t command;
    void update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state);
    matrix_t getCommand() {return command;};
};

class SingleIntCommand : public Command {
public: 
    SingleIntCommand(const int horizon, const double dt, const double v_max, const scalar_t x0, const scalar_t y0);
    matrix_t command;
    const int horizon;
    int getHorizon() const override { return horizon; }
    int getStateDim() const override { return 2; }
    int getInputDim() const { return 2; }
    const double dt;
    const double v_max;
    void update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state);
    matrix_t getCommand() {return command.transpose();};
};

class DoubleIntCommand : public Command {
public: 
    DoubleIntCommand(const int horizon, const double dt, const double v_max, const double a_max);
    matrix_t command;
    const int horizon;
    int getHorizon() const override { return horizon; }
    int getStateDim() const override { return 4; }
    const double dt;
    const double v_max;
    const double a_max;
    void update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state);
    matrix_t getCommand() {return command.transpose();};
};

class PredCBFCommand : public Command {
public: 
    PredCBFCommand(
        const double horizon, const double dt, const double alpha, const double rho, const bool smooth_barrier, const double epsilon,
        const double k_r, const double v_max, const double pred_dt, const int iters, const double K, const double tol, const bool use_delta,
        const std::vector<double> rs, const std::vector<double> cxs, const std::vector<double> cys, vector_2t zd
    );
    const std::string gainYamlPath = "../config/gains.yaml";
    std::shared_ptr<Hopper> hopper_;
    RaibertPolicy raibert_policy_ = RaibertPolicy(gainYamlPath);
    matrix_t command_;
    const double horizon_;
    const double dt_;
    const double alpha_;
    const double rho_;
    double delta_;
    const bool use_delta_;
    const int iters_;
    const double K_;
    const double tol_;
    const bool smooth_barrier_;
    const double epsilon_;
    const double k_r_;
    const double v_max_;
    const double pred_dt_;
    const std::vector<double> rs_;
    const std::vector<double> cxs_;
    const std::vector<double> cys_;
    const vector_t zd_;
    const int num_obs_;
    int num_runs_;
    
    mjModel *m_;
    mjData *d_;
    void update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state);
    matrix_t getCommand() {return command_;};
    int getHorizon() const override { return 1; }
    int getStateDim() const override { return 2; }
    int getInputDim() const { return 2; }
    vector_3t h_Dh(vector_2t z);
    vector_2t vd(vector_2t z);
    vector_2t predictiveSafetyFilter(Hopper::State &state);
    double robustifiedRollout(Hopper::State &state);
    vector_2t robustifiedSafetyFilter(vector_2t z, vector_2t vd);
};
#endif