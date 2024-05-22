#include "../inc/Controller.h"


// Driver code
int main()
{
    setupSocket(server_fd, new_socket, address, opt_socket, addrlen);
    setupGains("../config/gains.yaml", mpc_p, p);
    MPC opt = MPC(20, 4, mpc_p);
    vector_t sol(opt.nx * opt.p.N + opt.nu * (opt.p.N - 1));
    vector_t sol_g((opt.nx + 1) * opt.p.N + opt.nu * (opt.p.N - 1));

    Hopper hopper = Hopper();
    
    // Data Logging
    fileHandle.open(dataLog);
    fileHandle << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";
    fileHandleDebug.open(predictionLog);
    fileHandleDebug << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";

    // Initialize 
    sol.setZero();
    sol_g.setZero();
    omega_des.setZero();
    u_des.setZero();
    x_term.setZero();
    ind = 1;

    std::thread userInput(getUserInput, std::ref(command), std::ref(cv), std::ref(m));

    for (;;)
    {
        read(new_socket, &RX_state, sizeof(RX_state));
        t1 = std::chrono::high_resolution_clock::now();

        Map<vector_t> state(RX_state, 20);
        dt_elapsed = state(0) - t_last;
        dt_elapsed_MPC = state(0) - t_last_MPC;

        hopper.updateState(state);
        quat_t quat(hopper.q(6), hopper.q(3), hopper.q(4), hopper.q(5));
        hopper.v.segment(3, 3) = quat._transformVector(hopper.v.segment(3, 3)); // Turn local omega to global omega
        q0 << hopper.q, hopper.v;
        q0_local = MPC::global2local(q0);
        bool replan = dt_elapsed_MPC >= p.MPC_dt_replan;
        if (replan)
        {
            opt.solve(hopper, sol, command, command_interp);
            for (int i = 0; i < opt.p.N; i++)
            {
                sol_g.segment(i * (opt.nx + 1), opt.nx + 1) << MPC::local2global(MPC::xik_to_qk(sol.segment(i * opt.nx, opt.nx), q0_local));
            }
            sol_g.segment((opt.nx + 1) * opt.p.N, opt.nu * (opt.p.N - 1)) << sol.segment((opt.nx) * opt.p.N, opt.nu * (opt.p.N - 1));
            x_pred << MPC::local2global(MPC::xik_to_qk(sol.segment(0, 20), q0_local)), MPC::local2global(MPC::xik_to_qk(sol.segment(20, 20), q0_local));
            u_pred << sol.segment(opt.p.N * opt.nx, 4);
            t_last_MPC = state(0);

            t2 = std::chrono::high_resolution_clock::now();
            // std::cout <<"Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";

            x_term << MPC::local2global(MPC::xik_to_qk(sol.segment(opt.nx * (opt.p.N - 1), 20), q0_local));
        }

        // Compute continuous time solution to discrete time problem.
        // vector_t x_des(21);
        // hopper.css2dss(opt.Ac.block(0,0,opt.nx,opt.nx),opt.Bc.block(0,0,opt.nx,opt.nu),opt.Cc.block(0,0,opt.nx,1),state(0)-t_last_MPC,opt.Ad_,opt.Bd_,opt.Cd_);
        // x_des << MPC::local2global(MPC::Exp(opt.Ad_*sol.segment(0,20) + opt.Bd_*u_pred + opt.Cd_));
        // quat_des = Quaternion<scalar_t>(x_des(6), x_des(3), x_des(4), x_des(5));
        // omega_des << x_des(14), x_des(15),x_des(16);

        // Simply set the next waypoint as the setpoint of the low level.
        quat_des = Quaternion<scalar_t>(x_pred(6, 1), x_pred(3, 1), x_pred(4, 1), x_pred(5, 1));
        omega_des << x_pred(14, 1), x_pred(15, 1), x_pred(16, 1);
        u_des = u_pred;

        quat_term = Quaternion<scalar_t>(x_term(6), x_term(3), x_term(4), x_term(5));
        pos_term << x_term(0), x_term(1), x_term(2);

        if (dt_elapsed > p.dt)
        {
            hopper.computeTorque(quat_des, omega_des, 0.1, u_des);
            t_last = state(0);
        }

        // TODO: Right now, the impact map takes up a dt, but that makes the timing inconsistent. Fix that.
        // Log data
        if (fileWrite)
        {
            // Local
            // x_global << hopper.q, hopper.v;
            // x_local = MPC::global2local(x_global);
            // xi_local = MPC::Log(x_local);
            // v_global = hopper.v.segment(0, 6);
            // v_local = x_local.segment(11, 6);
            // fileHandle <<state[0] << "," << hopper.contact << "," << xi_local.transpose().format(CSVFormat) << "," << hopper.torque.transpose().format(CSVFormat) << "," << t_last_MPC << "," << sol.transpose().format(CSVFormat)<< "," << replan << std::endl;
            // Global
            fileHandle << state[0] << "," << hopper.contact << "," << hopper.q.transpose().format(CSVFormat) << ","
                       << hopper.v.transpose().format(CSVFormat) << "," << hopper.torque.transpose().format(CSVFormat) << ","
                       << t_last_MPC << "," << sol_g.transpose().format(CSVFormat) << "," << replan << ","
                       << opt.elapsed_time.transpose().format(CSVFormat) << "," << opt.d_bar.cast<int>().transpose().format(CSVFormat) << ","
                       << command.transpose().format(CSVFormat) << std::endl;
        }

        for (int i = 0; i < 4; i++)
        {
            TX_torques[i] = hopper.torque[i];
        }

        TX_torques[4] = pos_term[0];
        TX_torques[5] = pos_term[1];
        TX_torques[6] = pos_term[2];
        TX_torques[7] = quat_term.w();
        TX_torques[8] = quat_term.x();
        TX_torques[9] = quat_term.y();
        TX_torques[10] = quat_term.z();
        TX_torques[11] = command_interp(0);
        TX_torques[12] = command_interp(1);
        TX_torques[13] = sol(floor(4. / 4 * (mpc_p.N - 1)) * 20);
        TX_torques[14] = sol(floor(4. / 4 * (mpc_p.N - 1)) * 20 + 1);
        TX_torques[15] = sol(floor(3. / 4 * (mpc_p.N - 1)) * 20);
        TX_torques[16] = sol(floor(3. / 4 * (mpc_p.N - 1)) * 20 + 1);
        TX_torques[17] = sol(floor(2. / 4 * (mpc_p.N - 1)) * 20);
        TX_torques[18] = sol(floor(2. / 4 * (mpc_p.N - 1)) * 20 + 1);
        TX_torques[19] = sol(floor(1. / 4 * (mpc_p.N - 1)) * 20);
        TX_torques[20] = sol(floor(1. / 4 * (mpc_p.N - 1)) * 20 + 1);
        TX_torques[21] = opt.full_ref(0);
        TX_torques[22] = opt.full_ref(1);

        send(new_socket, &TX_torques, sizeof(TX_torques), 0);
        if (ind == p.stop_index)
        {
            exit(2);
        }
        ind++;
    }
}
