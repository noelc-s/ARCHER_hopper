#include "../inc/Controller_Hardware.h"

// Driver code
int main(int argc, char **argv)
{
  ESPstate.setZero();
  fileHandle.open(dataLog);
  fileHandle << "t,contact,x,y,z,legpos,vx,vy,vz,legvel,q_x,q_y,q_z,q_w,qd_x,qd_y,qd_z,qd_w,w_1,w_2,w_3,tau_foot,tau1,tau2,tau3,wheel_vel1,wheel_vel2,wheel_vel3,des_cmd,graph_sol,sol,obst" << std::endl;

  desstate[0] = 1;
  desstate[1] = 0;
  desstate[2] = 0;
  desstate[3] = 0;
  desstate[4] = 0;
  desstate[5] = 0;
  desstate[6] = 0;
  desstate[7] = 0;
  desstate[8] = 0;
  desstate[9] = 0;

  setupGainsHardware(gainYamlPath);

  std::shared_ptr<Hopper> hopper(new Hopper(gainYamlPath));
  std::unique_ptr<Command> command;
  if (p.rom_type == "single_int")
  {
    command = std::make_unique<SingleIntCommand>(p.horizon, p.dt_policy, p.v_max);
  }
  else if (p.rom_type == "double_int")
  {
    command = std::make_unique<DoubleIntCommand>(p.horizon, p.dt_policy, p.v_max, p.a_max);
  }
  else if (p.rom_type == "position")
  {
    command = std::make_unique<V5Command>();
  }
  else
  {
    throw std::runtime_error("RoM type unrecognized");
  }
  // Instantiate a new policy.
  // MPCPolicy policy = MPCPolicy(gainYamlPath, hopper, opt);
  RaibertPolicy policy = RaibertPolicy(gainYamlPath);
  // ZeroDynamicsPolicy policy = ZeroDynamicsPolicy("../../models/trained_model.onnx", gainYamlPath);
  // RLPolicy policy = RLPolicy("../../models/hopper_vel_0w94yf4r.onnx", gainYamlPath);
  // RLTrajPolicy policy = RLTrajPolicy(p.model_name, gainYamlPath, command->getHorizon(), command->getStateDim());

  // Thread for user input
  std::thread getUserInput2(&UserInput::getJoystickInput, &readUserInput, std::ref(offsets), std::ref(reset), std::ref(cv), std::ref(m));
  // std::thread getUserInput(&UserInput::getKeyboardInput, &readUserInput, std::ref(command), std::ref(cv), std::ref(m));
  std::thread getUserInput(&UserInput::cornerTraversal, &readUserInput, std::ref(offsets), std::ref(reset), std::ref(cv), std::ref(m));

  // Thread for updating reduced order model
  std::thread runRoM(&Command::update, command.get(), &readUserInput, std::ref(running), std::ref(cv), std::ref(m));
  desired_command = command->getCommand();

  ObstacleCollector O = ObstacleCollector();
  Planner planner(O);
  std::cout << "Number of Edges: " << planner.planner->edges.size() << std::endl;

  startRosNode(argc, argv);
  // Give ROS some time to initialize
  std::this_thread::sleep_for(std::chrono::seconds(2));

  vector_t planned_command, graph_sol;
  vector_t IC;
  vector_t EC;
  vector_t path_command;
  path_command.resize(5);
  int index = 1;
  IC.resize(4);
  EC.resize(4);
  planned_command.resize(4 * planner.planner->mpc_->mpc_params_.N);
  graph_sol.resize(4 * planner.planner->mpc_->mpc_params_.N);
  IC.setZero();
  EC.setZero();
  planned_command.setZero();
  graph_sol.setZero();

  std::thread runPlanner(&Planner::update, &planner, std::ref(O), std::ref(IC), std::ref(EC), std::ref(planned_command), std::ref(graph_sol), std::ref(index), std::ref(running), std::ref(cv), std::ref(m));

  const int max_num_obstacles = 20;

  int size = 11 + 2 + 8 * max_num_obstacles + 2 * planner.planner->mpc_->mpc_params_.N + 2 * planner.planner->mpc_->mpc_params_.N;
  scalar_t *TX_torques = new scalar_t[size](); // Dynamically allocate array
  scalar_t RX_state[1] = {0.0};
  std::thread runVis(&MujocoVis, std::ref(cv), std::ref(hopper->state_), TX_torques, RX_state, size);

  quat_des.setIdentity();
  omega_des.setZero();
  u_des.setZero();

  offsets << p.roll_offset, p.pitch_offset;
  std::cout << "Offsets (r, p): " << offsets.transpose() << std::endl;

  A_kf << 0.4234, 0.0000, -0.0000, 0.0042, 0, 0,
      0.0000, 0.4234, 0.0000, 0, 0.0042, 0,
      -0.0000, -0.0000, 0.4234, 0, 0, 0.0042,
      -30.9689, 0.0000, -0.0000, 1.0000, 0, 0,
      0.0000, -30.9689, 0.0000, 0, 1.0000, 0,
      -0.0000, -0.0000, -30.9689, 0, 0, 1.0000;
  B_kf << 0, 0.5766, -0.0000, 0.0000,
      0, -0.0000, 0.5766, -0.0000,
      0, 0.0000, 0.0000, 0.5766,
      0, 30.9689, -0.0000, 0.0000,
      0, -0.0000, 30.9689, -0.0000,
      0.0042, 0.0000, 0.0000, 30.9689;

  // ROS stuff
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/vrpn_client_node/hopper/pose", 200, chatterCallback);

  quat_t quat_opti = quat_t(OptiState.q_w, OptiState.q_x, OptiState.q_y, OptiState.q_z);
  while (quat_opti.norm() < 0.99)
  {
    ros::spinOnce();
    quat_opti = quat_t(OptiState.q_w, OptiState.q_x, OptiState.q_y, OptiState.q_z);
    // std::cout << "Waiting for optitrack quat" << std::endl;
    // std::cout << quat_opti.coeffs().transpose() << std::endl;
  };

  signal(SIGINT, signal_callback_handler);
  setupSocketHardware();
  std::thread thread_object(getStateFromEthernet, std::ref(reset), std::ref(cv), std::ref(m));
  sleep(1);
  vector_3t current_vel, previous_vel;
  std::chrono::high_resolution_clock::time_point last_t_state_log;

  tstart = std::chrono::high_resolution_clock::now();
  t_loop = tstart;
  t_lowlevel = tstart;
  t_policy = tstart;

  while (ros::ok())
  {
    ros::spinOnce();
    t_loop = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::nanoseconds>(t_loop - t_lowlevel).count() * 1e-9 > p.dt_lowlevel)
    {
      t_lowlevel = t_loop;

      {
        std::lock_guard<std::mutex> lck(state_mtx);
        quat_optitrack = quat_t(OptiState.q_w, OptiState.q_x, OptiState.q_y, OptiState.q_z);
        quat_optitrack.normalize();
        state << std::chrono::duration_cast<std::chrono::nanoseconds>(t_loop - tstart).count() * 1e-9,
            OptiState.x, OptiState.y, OptiState.z,
            quat_optitrack.w(), quat_optitrack.x(), quat_optitrack.y(), quat_optitrack.z(), // uncomment if you want optitrack as orientation
            // ESPstate(6), ESPstate(7), ESPstate(8), ESPstate(9), // IMU as orientation
            OptiState.x_dot, OptiState.y_dot, OptiState.z_dot,
            ESPstate(3), ESPstate(4), ESPstate(5),
            ESPstate(10), ESPstate(11), ESPstate(12), ESPstate(0), ESPstate(1), ESPstate(2); // TODO: Balancing make nice
      }
      hopper->updateState(state);
      contact = hopper->state_.contact;
      // quat_t IMU_quat = hopper->state_.quat;

      // // Measure the initial absolute yaw (from optitrack)
      // static scalar_t initial_yaw = extract_yaw(quat_optitrack);
      // static quat_t initial_yaw_quat = Policy::Euler2Quaternion(0, 0, initial_yaw);

      // // Remove the measured yaw to put us back in the global frame
      // scalar_t measured_yaw = extract_yaw(hopper->state_.quat);
      // scalar_t optitrack_yaw = extract_yaw(quat_optitrack);
      // quat_t measured_yaw_quat = Policy::Euler2Quaternion(0, 0, measured_yaw);
      // quat_t optitrack_yaw_quat = Policy::Euler2Quaternion(0, 0, optitrack_yaw);
      // quat_t yaw_corrected = plus(optitrack_yaw_quat, minus(hopper->state_.quat, measured_yaw_quat));
      // hopper->state_.quat = yaw_corrected;

      hopper->state_.quat = quat_optitrack;
      // Add roll pitch offset to body frame
      quat_t rollPitch = Policy::Euler2Quaternion(-offsets[0], -offsets[1], 0);
      hopper->state_.quat = plus(hopper->state_.quat, rollPitch);

      // Obstacle positions are in local frame of hopper
      vector_t sol(planned_command.size());
      std::vector<vector_t> obstacle_pos_zed;
      std::vector<Obstacle> obstacles;
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
      std::vector<float> boxes = getBoxPositions();
      for (size_t i = 0; i < max_num_obstacles * 8; i += 8) // max_num_obstacles obstacles max
      {
        vector_t obst(8);
        obst.setZero();
        if (i < boxes.size())
        {
          for (size_t j = 0; j < 8; j += 2)
          {
            obst[j] = boxes[i + j] + p.zed_x_offset;
          }
          for (size_t j = 1; j < 8; j += 2)
          {
            obst[j] = boxes[i + j] + p.zed_y_offset;
          }
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
          obstacles.push_back(obs);
        }
        else
        {
          obs.v.setZero();
          obs.A.setZero();
          obs.b << -1, -1, -1, -1;
        }

        obstacle_pos_zed.push_back(obst);
      }
      O.obstacles = obstacles;
      IC << hopper->state_.pos(0), hopper->state_.pos(1), hopper->state_.vel(0), hopper->state_.vel(1);
      EC << desired_command(0), desired_command(1), 0, 0;
      path_command << planned_command.segment(4 * index, 4), 0;
      sol << planned_command;

      if (std::chrono::duration_cast<std::chrono::nanoseconds>(t_loop - t_policy).count() * 1e-9 > p.dt_policy)
      {
        t_policy = t_loop;
        desired_command = command->getCommand();
        if (planner.planner->params_.use_planner)
        {
          quat_des = policy.DesiredQuaternion(hopper->state_, path_command);
        }
        else
        {
          quat_des = policy.DesiredQuaternion(hopper->state_, desired_command);
        }

        // Add initial yaw to desired signal
        // quat_des = plus(quat_des, initial_yaw_quat);
        omega_des = policy.DesiredOmega();
      }
      // TODO: this is just doing spindown. make this nice.
      u_des = policy.DesiredInputs(hopper->state_.wheel_vel, hopper->state_.contact);

      hopper->computeTorque(quat_des, omega_des, 0.1, u_des);
      for (int i = 0; i < 11; i++)
      {
        TX_torques[i] = hopper->state_.q[i];
      }
      TX_torques[11] = desired_command(0);
      TX_torques[12] = desired_command(1);
      for (int i = 0; i < obstacle_pos_zed.size(); i++)
      {
        TX_torques[13 + i * 8] = obstacle_pos_zed[i][0];
        TX_torques[14 + i * 8] = obstacle_pos_zed[i][1];
        TX_torques[15 + i * 8] = obstacle_pos_zed[i][2];
        TX_torques[16 + i * 8] = obstacle_pos_zed[i][3];
        TX_torques[17 + i * 8] = obstacle_pos_zed[i][4];
        TX_torques[18 + i * 8] = obstacle_pos_zed[i][5];
        TX_torques[19 + i * 8] = obstacle_pos_zed[i][6];
        TX_torques[20 + i * 8] = obstacle_pos_zed[i][7];
      }
      for (int i = 0; i < planner.planner->mpc_->mpc_params_.N; i++)
      {
        TX_torques[13 + 8 * obstacle_pos_zed.size() + 2 * i] = sol[4 * i];
        TX_torques[13 + 8 * obstacle_pos_zed.size() + 2 * i + 1] = sol[4 * i + 1];
      }
      for (int i = 0; i < planner.planner->mpc_->mpc_params_.N; i++)
      {
        TX_torques[13 + 8 * obstacle_pos_zed.size() + 2 * planner.planner->mpc_->mpc_params_.N + 2 * i] = graph_sol[4 * i];
        TX_torques[13 + 8 * obstacle_pos_zed.size() + 2 * planner.planner->mpc_->mpc_params_.N + 2 * i + 1] = graph_sol[4 * i + 1];
      }

      // e = quat_des.inverse() * hopper->state_.quat;
      // auto e_ = manif::SO3<scalar_t>(e);
      // xi = e_.log();
      // error << xi.coeffs();

      {
        std::lock_guard<std::mutex> lck(des_state_mtx);
        desstate[0] = quat_des.w();
        desstate[1] = quat_des.x();
        desstate[2] = quat_des.y();
        desstate[3] = quat_des.z();
        desstate[4] = omega_des(0);
        desstate[5] = omega_des(1);
        desstate[6] = omega_des(2);
        desstate[7] = hopper->torque[1];
        desstate[8] = hopper->torque[2];
        desstate[9] = hopper->torque[3];
        // desstate[0] = 1;
        // desstate[1] = 0;
        // desstate[2] = 0;
        // desstate[3] = 0;
        // desstate[4] = 0;
        // desstate[5] = 0;
        // desstate[6] = 0;
        // desstate[7] = 0;
        // desstate[8] = 0;
        // desstate[9] = 0;
      }

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
                   << "," << desired_command.transpose().format(CSVFormat)
                   << "," << graph_sol.transpose().format(CSVFormat)
                   << "," << sol.transpose().format(CSVFormat);
        for (auto o : obstacles)
        {
          fileHandle << "," << o.v.col(0).transpose().format(CSVFormat);
          fileHandle << "," << o.v.col(1).transpose().format(CSVFormat);
        }
        fileHandle << std::endl;
      }
    }
  }
}

// roslaunch vrpn_client_ros fast.launch server:=192.168.1.2
// roslauch /home/noelcs/repos/... server
// ifconfig enx5c857e36c21a 169.254.10.80
// marker offset -120 mm
