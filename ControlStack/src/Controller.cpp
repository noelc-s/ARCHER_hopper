#include "../inc/Controller.h"

// Driver code
int main(int argc, char **argv)
{
    setupSocket(server_fd, new_socket, address, opt_socket, addrlen);
    setupGains(gainYamlPath, mpc_p, p); // mpc_p,
    // std::shared_ptr<MPC> opt(new MPC(20, 4, mpc_p));
    std::shared_ptr<Hopper> hopper(new Hopper(gainYamlPath));

    // Data Logging
    fileHandle.open(dataLog);
    fileHandle << "t,contact,"
            << "x,y,z,"
            << "legpos,"
            << "vx,vy,vz,"
            << "legvel,"
            << "q_x,q_y,q_z,q_w,"
            << "qd_x,qd_y,qd_z,qd_w,"
            << "w_x,w_y,w_z,"
            << "tau_foot,tau1,tau2,tau3,"
            << "wheel_vel1,wheel_vel2,wheel_vel3,"
            << "des_cmd1,des_cmd2,des_cmd3,des_cmd4,des_cmd5" << std::endl;
    fileHandleDebug.open(predictionLog);
    fileHandleDebug << "t,x,y,z,q_x,q_y,q_z,q_w,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";

    // Initialize
    offsets << p.roll_offset, p.pitch_offset;
    omega_des.setZero();
    u_des.setZero();
    x_term.setZero();
    ind = 1;

    std::unique_ptr<Command> command;
    if (p.rom_type == "single_int")
    {
        command = std::make_unique<SingleIntCommand>(p.horizon, p.dt_replan, p.v_max, p.x0, p.y0);
    }
    else if (p.rom_type == "double_int")
    {
        command = std::make_unique<DoubleIntCommand>(p.horizon, p.dt_replan, p.v_max, p.a_max);
    }
    else if (p.rom_type == "position") {
        command = std::make_unique<V5Command>(p.x0, p.y0);
    }
    else
    {
        throw std::runtime_error("RoM type unrecognized");
    }
    // Instantiate a new policy.
    // std::shared_ptr<MPC> mpc = std::make_shared<MPC>(20,4,mpc_p, hopper);
    // MPCPolicy policy = MPCPolicy(gainYamlPath, hopper, mpc);
    RaibertPolicy policy = RaibertPolicy(gainYamlPath);
    // ZeroDynamicsPolicy policy = ZeroDynamicsPolicy("../../models/trained_model.onnx", gainYamlPath);
    // RLPolicy policy = RLPolicy("../../models/hopper_vel_0w94yf4r.onnx", gainYamlPath);
    // RLTrajPolicy policy = RLTrajPolicy(p.model_name, gainYamlPath, command->getHorizon(), command->getStateDim());

    scalar_t yaw;
    bool reset_pos = false;
    // Thread for user input
    std::thread getUserInput(&UserInput::getJoystickInput, &readUserInput, std::ref(offsets), std::ref(reset), std::ref(yaw), std::ref(reset_pos), std::ref(cv), std::ref(m));
    // std::thread getUserInput(&UserInput::getKeyboardInput, &readUserInput, std::ref(offsets), std::ref(reset), std::ref(yaw), std::ref(cv), std::ref(m));

    // Thread for updating reduced order model
    std::thread runRoM(&Command::update, command.get(), &readUserInput, std::ref(running), std::ref(cv), std::ref(m));
    desired_command = command->getCommand();

    // 4 torques, 7 terminal s SE(3) state, 2 command, 8 obstacle_corners, xy mpc sol
    float TX_torques[4 + 7 + 2] = {};
    // time, pos, quat, vel, omega, contact, leg_pos, leg_vel, wheel_vel
    scalar_t RX_state[20] = {};

    // sleep(10);

    for (;;)
    {
        auto ret = read(new_socket, &RX_state, sizeof(RX_state));
        t1 = std::chrono::high_resolution_clock::now();

        Map<vector_t> state(RX_state, 20);
        dt_elapsed = state(0) - t_last;
        dt_planner_elapsed = state(0) - t_planner_last;
        dt_print_elapsed = state(0) - t_print_last;

        hopper->updateState(state);

        ///////     SIMULATING DRIFT /////////////////
        // scalar_t yaw_drift = state(0) * p.yaw_drift;
        // quat_t yaw_drift_quat = Euler2Quaternion(0, 0, yaw_drift);
        // quat_t q_meas = plus(yaw_drift_quat, hopper->state_.quat);
        // scalar_t optitrack_yaw = extract_yaw(hopper->state_.quat);
        // hopper->state_.quat = q_meas;
        // IMU_quat = hopper->state_.quat;
        ///////////////////////////////////////////

        // Measure the initial absolute yaw (from optitrack)
        // static scalar_t initial_yaw = optitrack_yaw;
        // static quat_t initial_yaw_quat = Euler2Quaternion(0, 0, initial_yaw);

        // Remove the measured yaw to put us back in the global frame
        // scalar_t measured_yaw = extract_yaw(hopper->state_.quat);
        // quat_t measured_yaw_quat = Euler2Quaternion(0, 0, measured_yaw);
        // quat_t optitrack_yaw_quat = Euler2Quaternion(0, 0, optitrack_yaw);
        // quat_t yaw_corrected = plus(optitrack_yaw_quat, minus(hopper->state_.quat, measured_yaw_quat));
        // hopper->state_.quat = yaw_corrected;

        // Add roll pitch offset to body frame
        quat_t rollPitch = Euler2Quaternion(-offsets[0], -offsets[1], 0);
        hopper->state_.quat = plus(hopper->state_.quat, rollPitch);
        hopper->state_.v.segment(3, 3) = hopper->state_.quat._transformVector(hopper->state_.v.segment(3, 3)); // Turn local omega to global omega

        if (dt_elapsed > p.dt)
        {
            desired_command = command->getCommand();
            quat_des = policy.DesiredQuaternion(hopper->state_, desired_command.col(0));
            // Add initial yaw to desired signal
            // quat_des = plus(quat_des, initial_yaw_quat);
            omega_des = policy.DesiredOmega();
            u_des = policy.DesiredInputs(hopper->state_.wheel_vel, hopper->state_.contact);
            t_last = state(0);
        }

        hopper->computeTorque(quat_des, omega_des, 0.1, u_des);

        e = quat_des.inverse() * hopper->state_.quat;
        auto e_ = manif::SO3<scalar_t>(e);
        xi = e_.log();
        error << xi.coeffs();

        // Log data
        if (fileWrite)
        {
            // fileHandle << "t,contact,x,y,z,legpos,vx,vy,vz,legvel,q_x,q_y,q_z,q_w,qd_x,qd_y,qd_z,qd_w,
                            // w_1,w_2,w_3,tau_foot,tau1,tau2,tau3,wheel_vel1,wheel_vel2,wheel_vel3,graph_sol,mpc_sol" << std::endl;
            fileHandle << state[0] << "," << hopper->state_.contact
                       << "," << hopper->state_.pos.transpose().format(CSVFormat)
                       << "," << hopper->state_.leg_pos
                       << "," << hopper->state_.vel.transpose().format(CSVFormat)
                       << "," << hopper->state_.leg_vel
                       << "," << hopper->state_.quat.coeffs().transpose().format(CSVFormat)
                       << "," << quat_des.coeffs().transpose().format(CSVFormat)
                       << "," << hopper->state_.omega.transpose().format(CSVFormat)
                       << "," << hopper->torque.transpose().format(CSVFormat)
                       << "," << hopper->state_.wheel_vel.transpose().format(CSVFormat)
                       << "," << desired_command.col(0).transpose().format(CSVFormat);
            fileHandle << std::endl;
        }

        for (int i = 0; i < 4; i++)
        {
            TX_torques[i] = hopper->torque[i];
        }

        // for (int i = 4; i < 23; i++)
        // {
        //     TX_torques[i] = 0;
        // }

        if ((desired_command.rows() == 5) & (desired_command.cols() == 1))
        {
            TX_torques[11] = desired_command(0);
            TX_torques[12] = desired_command(1);
        }
        else
        {
            TX_torques[11] = desired_command(0, desired_command.rows() - 1);
            TX_torques[12] = desired_command(1, desired_command.rows() - 1);
        }

        send(new_socket, &TX_torques, sizeof(TX_torques), 0);
        if (ind == p.stop_index)
        {
            running = false;
            exit(2);
        }
        ind++;
    }
}
