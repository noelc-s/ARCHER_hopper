#include "../inc/Types.h"
#include "../inc/UserInput.h"

using namespace Hopper_t;


class Command{
public: 
    virtual ~Command() {};
    virtual void update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m) = 0;
    virtual matrix_t getCommand() = 0;
    virtual int getHorizon() const = 0;
    virtual int getStateDim() const = 0;
};

class V3Command : public Command{
public: 
    V3Command();
    int getHorizon() const override { return 1; }
    int getStateDim() const override { return 3; }
    vector_3t command;
    void update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m);
    matrix_t getCommand() {return command;};
};

class SingleIntCommand : public Command {
public: 
    SingleIntCommand(const int horizon, const double dt, const double v_max);
    matrix_t command;
    const int horizon;
    int getHorizon() const override { return horizon; }
    int getStateDim() const override { return 2; }
    const double dt;
    const double v_max;
    void update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m);
    matrix_t getCommand() {return command;};
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
    void update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m);
    matrix_t getCommand() {return command;};
};