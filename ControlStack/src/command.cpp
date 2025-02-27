#include "../inc/command.h"
#include <thread>

std::unique_ptr<Command> createCommand(const Parameters& p) {
    if (p.rom_type == "single_int") {
        return std::make_unique<SingleIntCommand>(p.horizon, p.dt_replan, p.v_max, p.x0, p.y0);
    } else if (p.rom_type == "double_int") {
        return std::make_unique<DoubleIntCommand>(p.horizon, p.dt_replan, p.v_max, p.a_max);
    } else if (p.rom_type == "position") {
        return std::make_unique<V5Command>(p.x0, p.y0);
    } else {
        throw std::runtime_error("RoM type unrecognized");
    }
}

std::unique_ptr<Command> createCommand(const HardwareParameters& p) {
    if (p.rom_type == "single_int") {
        return std::make_unique<SingleIntCommand>(p.horizon, p.dt_replan, p.v_max, p.x0, p.y0);
    } else if (p.rom_type == "double_int") {
        return std::make_unique<DoubleIntCommand>(p.horizon, p.dt_replan, p.v_max, p.a_max);
    } else if (p.rom_type == "position") {
        return std::make_unique<V5Command>(p.x0, p.y0);
    } else {
        throw std::runtime_error("RoM type unrecognized");
    }
}

V5Command::V5Command(const scalar_t x0, const scalar_t y0) : x0_(x0), y0_(y0)
{
    command.resize(5);
    command.setZero();
}

void V5Command::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m)
{
    while (running)
    {
        {
            std::lock_guard<std::mutex> lock(m);
            command << x0_ + userInput->joystick_command(0), y0_ + userInput->joystick_command(1),0,0, userInput->joystick_command(2);
        }
    }
}

SingleIntCommand::SingleIntCommand(const int horizon, const double dt, const double v_max, const scalar_t x0, const scalar_t y0) : horizon(horizon), dt(dt), v_max(v_max)
{
    command.resize(getHorizon(), getStateDim() + getInputDim() + 1);
    command.setZero();
    for (int i = 0; i < command.rows(); i++) {
        command(i,0) = x0;
        command(i,1) = y0;
    }
    
}

void SingleIntCommand::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m)
{
    // TODO: update dynamics
    while (running)
    {
        auto start = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(m);
            command.block(0, 0, horizon - 1, getStateDim() + getInputDim()) = command.block(1, 0, horizon - 1, getStateDim() + getInputDim());
            command.block(horizon - 1, 0, 1, getStateDim()) = command.block(horizon - 1, 0, 1, getStateDim()) + v_max * userInput->joystick_command.segment(0, 2).transpose() * dt;
            command.block(horizon - 1, getStateDim(), 1, getInputDim()) = v_max * userInput->joystick_command.segment(0, 2).transpose();
            command(0, getStateDim() + getInputDim()) = userInput->joystick_command(2);

        }
        // std::cout << command(0, 0) << ',' << command(0, 1) << std::endl;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::chrono::milliseconds sleep_duration(static_cast<int>((dt - elapsed.count()) * 1000));
        if (sleep_duration.count() > 0)
        {
            std::this_thread::sleep_for(sleep_duration);
        }
    }
}

DoubleIntCommand::DoubleIntCommand(const int horizon, const double dt, const double v_max, const double a_max) : horizon(horizon), dt(dt), v_max(v_max), a_max(a_max)
{
    command.resize(getHorizon(), getStateDim() + 1);
    command.setZero();
}

void DoubleIntCommand::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m)
{
    while (running)
    {
        auto start = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(m);
            command.block(0, 0, horizon - 1, getStateDim()) = command.block(1, 0, horizon - 1, getStateDim());
            vector_4t z_next;
            z_next << command(horizon - 1, 0) + dt * command(horizon - 1, 2),
                command(horizon - 1, 1) + dt * command(horizon - 1, 3),
                std::min(std::max(command(horizon - 1, 2) + dt * a_max * userInput->joystick_command(0), -v_max), v_max),
                std::min(std::max(command(horizon - 1, 3) + dt * a_max * userInput->joystick_command(1), -v_max), v_max);
            command.block(horizon - 1, 0, 1, getStateDim()) = z_next.transpose();
            command(0, getStateDim()) = userInput->joystick_command(2);
        }
        // std::cout << command(0, 0) << ',' << command(0, 1) << ',' << command(0, 2) << ',' << command(0, 3) << std::endl;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::chrono::milliseconds sleep_duration(static_cast<int>((dt - elapsed.count()) * 1000));
        if (sleep_duration.count() > 0)
        {
            std::this_thread::sleep_for(sleep_duration);
        }
    }
}