#ifndef ROM_H
#define ROM_H
#include "../inc/Types.h"
#include "../inc/UserInput.h"

#include "../inc/utils.h"
#include "../inc/Hopper.h"
#include "../inc/reduced_order_safety_filter.h"

using namespace Hopper_t;


class Command{
public: 
    virtual ~Command() {};
    virtual void update(UserInput *userInput, std::atomic<bool> &running, vector_t& des_vel, vector_t& safe_vel, scalar_t& safe_h, std::condition_variable &cv, std::mutex &m, Hopper::State &state) = 0;
    virtual matrix_t getCommand() = 0;
    virtual int getHorizon() const = 0;
    virtual int getStateDim() const = 0;
};

class V5Command : public Command{
public: 
    V5Command();
    int getHorizon() const override { return 1; }
    int getStateDim() const override { return 3; }
    vector_t command;
    void update(UserInput *userInput, std::atomic<bool> &running, vector_t& des_vel, vector_t& safe_vel, scalar_t& safe_h, std::condition_variable &cv, std::mutex &m, Hopper::State &state);
    matrix_t getCommand() {return command;};
};

class SingleIntCommand : public Command {
public: 
    SingleIntCommand(const int horizon, const double dt, const double v_max);
    matrix_t command;
    const int horizon;
    int getHorizon() const override { return horizon; }
    int getStateDim() const override { return 2; }
    int getInputDim() const { return 2; }
    const double dt;
    const double v_max;
    void update(UserInput *userInput, std::atomic<bool> &running, vector_t& des_vel, vector_t& safe_vel, scalar_t& safe_h, std::condition_variable &cv, std::mutex &m, Hopper::State &state);
    matrix_t getCommand() {return command.transpose();};
};

class SafeSingleInt : public Command {
public: 
    SafeSingleInt(const int horizon, const double dt, const double v_max, const std::vector<double> o_r, const std::vector<double> o_x, const std::vector<double> o_y);
    matrix_t command;
    const int horizon;
    int getHorizon() const override { return horizon; }
    int getStateDim() const override { return 2; }
    int getInputDim() const { return 2; }
    const double dt;
    const double v_max;
    std::vector<double> r0;
    std::vector<double> x0;
    std::vector<double> y0;

    // Construct a ReducedOrderSafetyFilter
    ReducedOrderSafetyFilter safety_filter;

    void update(UserInput *userInput, std::atomic<bool> &running, vector_t& des_vel, vector_t& safe_vel, scalar_t& safe_h, std::condition_variable &cv, std::mutex &m, Hopper::State &state);
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
    void update(UserInput *userInput, std::atomic<bool> &running, vector_t& des_vel, vector_t& safe_vel, scalar_t& safe_h, std::condition_variable &cv, std::mutex &m, Hopper::State &state);
    matrix_t getCommand() {return command.transpose();};
};
#endif