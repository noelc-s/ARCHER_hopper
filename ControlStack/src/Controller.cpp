#include "../inc/Controller.h"


// Driver code
int main()
{
    setupSocket(server_fd, new_socket, address, opt_socket, addrlen);
    setupGains(gainYamlPath, mpc_p, p);
    std::shared_ptr<MPC> opt(new MPC(20, 4, mpc_p));
    std::shared_ptr<Hopper> hopper(new Hopper(gainYamlPath));
    
    // Data Logging
    fileHandle.open(dataLog);
    fileHandle << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc" << std::endl;
    fileHandleDebug.open(predictionLog);
    fileHandleDebug << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc" << std::endl;

   // Initialize 
    offsets << p.roll_offset, p.pitch_offset;
    omega_des.setZero();
    u_des.setZero();
    x_term.setZero();
    ind = 1;

    // Instantiate a new policy.
    std::shared_ptr<Integrator> integrator(new Integrator(0.01, matrix_t::Identity(21,21), matrix_t::Identity(4,4)));
    PMPPolicy policy = PMPPolicy(gainYamlPath, hopper, integrator);
    // MPCPolicy policy = MPCPolicy(gainYamlPath, hopper, opt);
    // RaibertPolicy policy = RaibertPolicy(gainYamlPath);
    // ZeroDynamicsPolicy policy = ZeroDynamicsPolicy("../../models/trained_model.onnx", gainYamlPath);

    // std::thread userInput(getUserInput, std::ref(command), std::ref(cv), std::ref(m));
    std::thread getUserInput(&UserInput::getKeyboardInput, &readUserInput, std::ref(command), std::ref(cv), std::ref(m));
    // std::thread getUserInput(&UserInput::getJoystickInput, &readUserInput, std::ref(offsets), std::ref(command), std::ref(dist), std::ref(cv), std::ref(m));


    for (;;)
    {
        read(new_socket, &RX_state, sizeof(RX_state));
        t1 = std::chrono::high_resolution_clock::now();

        Map<vector_t> state(RX_state, 20);
        dt_elapsed = state(0) - t_last;

        hopper->updateState(state);

        ///////   SIMULATING DRIFT /////////////////
        scalar_t yaw_drift = state(0) * p.yaw_drift;
        quat_t yaw_drift_quat = Euler2Quaternion(0, 0, yaw_drift);
        quat_t q_meas = plus(yaw_drift_quat, hopper->quat);
        scalar_t optitrack_yaw = extract_yaw(hopper->quat);
        hopper->quat = q_meas;
        IMU_quat = hopper->quat;
        ///////////////////////////////////////////

        // Measure the initial absolute yaw (from optitrack)
        static scalar_t initial_yaw = optitrack_yaw;
        static quat_t initial_yaw_quat = Euler2Quaternion(0, 0, initial_yaw);

        // Remove the measured yaw to put us back in the global frame
        scalar_t measured_yaw = extract_yaw(hopper->quat);
        quat_t measured_yaw_quat = Euler2Quaternion(0, 0, measured_yaw);
        quat_t optitrack_yaw_quat = Euler2Quaternion(0, 0, optitrack_yaw);
        quat_t yaw_corrected = plus(optitrack_yaw_quat, minus(hopper->quat, measured_yaw_quat));
        hopper->quat = yaw_corrected; 

        // Add roll pitch offset to body frame
        quat_t rollPitch = Euler2Quaternion(-offsets[0], -offsets[1], 0);
        hopper->quat = plus(hopper->quat, rollPitch);
        hopper->v.segment(3, 3) = hopper->quat._transformVector(hopper->v.segment(3, 3)); // Turn local omega to global omega

        quat_des = policy.DesiredQuaternion(state(1), state(2), command,
                                            state(8), state(9), dist(0), hopper->contact);

        // Add initial yaw to desired signal
        quat_des = plus(quat_des, initial_yaw_quat);
        omega_des = policy.DesiredOmega();
        u_des = policy.DesiredInputs(hopper->wheel_vel, hopper->contact);

        if (dt_elapsed > p.dt)
        {
            hopper->computeTorque(quat_des, omega_des, 0.1, u_des);
            t_last = state(0);
        }

        
        e = quat_des.inverse() * hopper->quat;
        auto e_ = manif::SO3<scalar_t>(e);
        xi = e_.log();
        error << xi.coeffs();

        // Log data
        if (fileWrite)
        {
            // fileHandle << state[0] << "," << hopper->contact 
            //             << "," << 1
            //             << "," << hopper->pos.transpose().format(CSVFormat)
            //             << "," << hopper->leg_pos
            //             << "," << hopper->vel.transpose().format(CSVFormat)
            //             << "," << hopper->leg_vel
            //             << "," << IMU_quat.coeffs().transpose().format(CSVFormat)
            //             << "," << hopper->quat.coeffs().transpose().format(CSVFormat)
            //             << "," << quat_des.coeffs().transpose().format(CSVFormat)
            //             << "," << hopper->omega.transpose().format(CSVFormat)
            //             << "," << hopper->torque.transpose().format(CSVFormat)
            //             << "," << error.transpose().format(CSVFormat)
            //             << "," << hopper->wheel_vel.transpose().format(CSVFormat) << std::endl;
            for (int i = 0; i < 100; i++) {
                fileHandle << policy.x_sol.block(0,i,21,1).transpose().format(CSVFormat) << ",";
            }
            fileHandle << std::endl;
        }

        for (int i = 0; i < 4; i++)
        {
            TX_torques[i] = hopper->torque[i];
        }

        for (int i = 4; i < 28; i++) {
            TX_torques[i] = 0;
        }
        TX_torques[11] = command(0);
        TX_torques[12] = command(1);

        // x_term << MPC::local2global(MPC::xik_to_qk(sol.segment(opt.nx * (opt.p.N - 1), 20), q0_local));
        // quat_term = Quaternion<scalar_t>(x_term(6), x_term(3), x_term(4), x_term(5));
        // pos_term << x_term(0), x_term(1), x_term(2);
        // TX_torques[4] = pos_term[0];
        // TX_torques[5] = pos_term[1];
        // TX_torques[6] = pos_term[2];
        // TX_torques[7] = quat_term.w();
        // TX_torques[8] = quat_term.x();
        // TX_torques[9] = quat_term.y();
        // TX_torques[10] = quat_term.z();
        for (int i = 0; i < 100; i ++) {
            TX_torques[13+3*i] = policy.x_sol(0,i)+i/100.;
            TX_torques[14+3*i] = policy.x_sol(1,i);
            TX_torques[15+3*i] = policy.x_sol(2,i);
        }
        // TX_torques[13] = policy.x_sol(0,floor(4. / 4 * (policy.num_iter-1)));
        // TX_torques[14] = policy.x_sol(1,floor(4. / 4 * (policy.num_iter-1)));
        // TX_torques[15] = policy.x_sol(2,floor(4. / 4 * (policy.num_iter-1)));
        // TX_torques[16] = policy.x_sol(0,floor(3. / 4 * (policy.num_iter-1)));
        // TX_torques[17] = policy.x_sol(1,floor(3. / 4 * (policy.num_iter-1)));
        // TX_torques[18] = policy.x_sol(2,floor(3. / 4 * (policy.num_iter-1)));
        // TX_torques[19] = policy.x_sol(0,floor(2. / 4 * (policy.num_iter-1)));
        // TX_torques[20] = policy.x_sol(1,floor(2. / 4 * (policy.num_iter-1)));
        // TX_torques[21] = policy.x_sol(2,floor(2. / 4 * (policy.num_iter-1)));
        // TX_torques[22] = policy.x_sol(0,floor(1. / 4 * (policy.num_iter-1)));
        // TX_torques[23] = policy.x_sol(1,floor(1. / 4 * (policy.num_iter-1)));
        // TX_torques[24] = policy.x_sol(2,floor(1. / 4 * (policy.num_iter-1)));
        // TX_torques[25] = policy.x_sol(0,0);
        // TX_torques[26] = policy.x_sol(1,1);
        // TX_torques[27] = policy.x_sol(2,2);

        send(new_socket, &TX_torques, sizeof(TX_torques), 0);
        if (ind == p.stop_index)
        {
            exit(2);
        }
        ind++;
    }
}
