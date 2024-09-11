#include "../inc/rom.h"
#include <thread>

V5Command::V5Command()
{
    command.resize(5);
    command.setZero();
}

void V5Command::update(UserInput *userInput, std::atomic<bool> &running, vector_t& des_vel, vector_t& safe_vel, scalar_t& safe_h, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
{
    while (running)
    {
        {
            std::lock_guard<std::mutex> lock(m);
            command << userInput->joystick_command.segment(0, 2),0,0, userInput->joystick_command(2);
        }
    }
}

SingleIntCommand::SingleIntCommand(const int horizon, const double dt, const double v_max) : horizon(horizon), dt(dt), v_max(v_max)
{
    command.resize(getHorizon(), getStateDim() + getInputDim() + 1);
    command.setZero();
}

void SingleIntCommand::update(UserInput *userInput, std::atomic<bool> &running, vector_t& des_vel, vector_t& safe_vel, scalar_t& safe_h, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
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

SafeSingleInt::SafeSingleInt(const int horizon, const double dt, const double v_max, const std::vector<double> o_r, const std::vector<double> o_x, const std::vector<double> o_y) : horizon(horizon), dt(dt), v_max(v_max)
{
    command.resize(getHorizon(), getStateDim() + getInputDim() + 1);
    command.setZero();

        // Params associated with CBF controller
    double alpha = 0.8;
    double epsilon = 20.0;
    double sigma = 100.0;

    // Params associated with desired controller: kd(x) = -Kp*(x - xd)
    double Kp = 0.4;
    Eigen::Vector2d xd(0.0, 0.0);

    // Params associated with obstacle: h(x) = norm(x - xo)^2 - ro^2
    r0 = o_r;
    x0 = o_x;
    y0 = o_y;

    // Params for maximum reduced-order input (clamp all controllers at umax)
    double umax = 0.5;

    // Fill up params
    safety_filter.alpha = alpha;
    safety_filter.epsilon = epsilon;
    safety_filter.Kp = Kp;
    
    safety_filter.sigma = sigma;
    safety_filter.umax = umax;
    safety_filter.xd = xd;
}

void SafeSingleInt::update(UserInput *userInput, std::atomic<bool> &running, vector_t& des_vel, vector_t& safe_vel, scalar_t& safe_h, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
{
        // TODO: update dynamics
    while (running)
    {
        auto start = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(m);
            command.block(0, 0, horizon - 1, getStateDim() + getInputDim()) = command.block(1, 0, horizon - 1, getStateDim() + getInputDim());

            vector_t input = v_max * userInput->joystick_command.segment(0, 2).transpose();  
            safety_filter.xd = input.transpose();    

            vector_t safe_input;
            scalar_t min_h = 1e4;
            scalar_t h = 0;
            for (int o = 0; o < r0.size(); o++) {
                safety_filter.ro = r0[o];
                safety_filter.xo << x0[o], y0[o];
                h = safety_filter.h(state.pos.segment(0,2));
                if (h < min_h) {
                    min_h = h;
                    safe_input = safety_filter.get_input(state.pos.segment(0,2));
                }
            }
            safe_h = min_h;

            des_vel = - safety_filter.Kp * (state.pos.segment(0,2) - safety_filter.xd);
            safe_vel = safe_input;



            command.block(horizon - 1, 0, 1, getStateDim()) = command.block(horizon - 1, 0, 1, getStateDim()) + safe_input.transpose() * dt;
            command.block(horizon - 1, getStateDim(), 1, getInputDim()) = safe_input.transpose();
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

void DoubleIntCommand::update(UserInput *userInput, std::atomic<bool> &running, vector_t& des_vel, vector_t& safe_vel, scalar_t& safe_h, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
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