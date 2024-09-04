#include "../inc/rom.h"
#include <thread>

V3Command::V3Command()
{
    command.setZero();
}

void V3Command::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
{
    while (running)
    {
        {
            std::lock_guard<std::mutex> lock(m);
            command = userInput->joystick_command.segment(0, 3);
        }
    }
}

SingleIntCommand::SingleIntCommand(const int horizon, const double dt, const double v_max) : horizon(horizon), dt(dt), v_max(v_max)
{
    command.resize(getHorizon(), getStateDim());
    command.setZero();
}

void SingleIntCommand::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
{
    // TODO: update dynamics
    while (running)
    {
        auto start = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(m);
            command.block(0, 0, horizon - 1, getStateDim()) = command.block(1, 0, horizon - 1, getStateDim());
            command.block(horizon - 1, 0, 1, getStateDim()) = command.block(horizon - 1, 0, 1, getStateDim()) + v_max * userInput->joystick_command.segment(0, 2).transpose() * dt;
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
    command.resize(getHorizon(), getStateDim());
    command.setZero();
}

void DoubleIntCommand::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
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

SingleIntTube::SingleIntTube(const int horizon, const double dt, const double v_max) : horizon(horizon), dt(dt), v_max(v_max)
{
    state_trajectory.resize(getHorizon() + 1, getStateDim());
    input_trajectory.resize(getHorizon(), getStateDim());
    command.resize(5, 1);
    command.setZero();
}

void SingleIntTube::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
{
    // First, initialize a UDP socket to talk to NLP solver
    setupTubeSocket(server_fd, new_socket, address, opt_socket, addrlen);
    float TX_state[4] = {0, 0, 0, 0};
    float RX_trajectory[getHorizon() * getStateDim() * 2 + 2];  // TODO: check size here

    bool conn = false;
    while (!conn) {
        read(new_socket, &conn, sizeof(conn));
    }
    
    
    // TODO: update dynamics
    while (running)
    {
        auto start = std::chrono::high_resolution_clock::now();
        // Send initial condition to tube solver (it is in command)
        TX_state[0] = static_cast<float>(command(0));
        TX_state[1] = static_cast<float>(command(1));
        {
            std::lock_guard<std::mutex> lock(m);
            TX_state[2] = static_cast<float>(state.pos[0]);
            TX_state[3] = static_cast<float>(state.pos[1]);
        }
        send(new_socket, &TX_state, sizeof(TX_state), 0);

        // Recieve trajectory from tube solver (update state_trajectory, input_trajectory)
        read(new_socket, &RX_trajectory, sizeof(RX_trajectory));

        // Update state, input trajectory
        for (int i = 0; i < (getHorizon() + 1) * getStateDim(); i ++) {
            state_trajectory(i / 2, i % 2) = RX_trajectory[i];
        }
        for (int i = 0; i < getHorizon() * getStateDim(); i ++) {
            input_trajectory(i / 2, i % 2) = RX_trajectory[(getHorizon() + 1) * getStateDim() + i];
        }

        // Update command
        {
            std::lock_guard<std::mutex> lock(m);
            command.block(0, 0, getStateDim(), 1) = state_trajectory.block(1, 0, 1, getStateDim()).transpose();
            command.block(getStateDim(), 0, getStateDim(), 1) = input_trajectory.block(0, 0, 1, getStateDim()).transpose();
        }
        // std::cout << command.transpose() << std::endl;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::chrono::milliseconds sleep_duration(static_cast<int>((dt - elapsed.count()) * 1000));
        std::cout << sleep_duration.count() << std::endl;
        if (sleep_duration.count() > 0)
        {
            std::this_thread::sleep_for(sleep_duration);
        }
    }
}