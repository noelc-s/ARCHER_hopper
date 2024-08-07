#include "../inc/Controller_Hardware.h"

// Driver code
int main(int argc, char **argv)
{

  ESPstate.setZero();
  fileHandle.open(dataLog);
  fileHandle << "t,contact,x,y,z,legpos,vx,vy,vz,legvel,q_x,q_y,q_z,q_w,qd_x,qd_y,qd_z,qd_w,w_1,w_2,w_3,tau_foot,tau1,tau2,tau3,wheel_vel1,wheel_vel2,wheel_vel3,rom_x,rom_y,rom_vx,rom_vy" << std::endl;

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
    command = std::make_unique<SingleIntCommand>(p.horizon, p.dt_replan, p.v_max);
  }
  else if (p.rom_type == "double_int")
  {
    command = std::make_unique<DoubleIntCommand>(p.horizon, p.dt_replan, p.v_max, p.a_max);
  }
  else
  {
    throw std::runtime_error("RoM type unrecognized");
  }
  RLTrajPolicy policy = RLTrajPolicy(p.model_name, gainYamlPath, command->getHorizon(), command->getStateDim());

  // Thread for user input
  std::thread getUserInput(&UserInput::getJoystickInput, &readUserInput, std::ref(offsets), std::ref(reset), std::ref(cv), std::ref(m));
  // std::thread getUserInput(&UserInput::getKeyboardInput, &readUserInput, std::ref(command), std::ref(cv), std::ref(m));

  // Thread for updating reduced order model
  std::thread runRoM(&Command::update, command.get(), &readUserInput, std::ref(running), std::ref(cv), std::ref(m));
  desired_command = command->getCommand();

  quat_des.setIdentity();
  omega_des.setZero();
  u_des.setZero();

  offsets << p.roll_offset, p.pitch_offset;

  // Kalman Filter gains
  contact = false;
  A_kf <<     0.4234,0.0000,-0.0000,0.0042,0,0,
    0.0000,0.4234,0.0000,0,0.0042,0,
   -0.0000,-0.0000,0.4234,0,0,0.0042,
  -30.9689,0.0000,-0.0000,1.0000 ,0,0,
    0.0000,-30.9689,0.0000,0,1.0000,0,
   -0.0000,-0.0000,-30.9689,0,0,1.0000;
  B_kf <<0,0.5766,-0.0000,0.0000,
         0,-0.0000,0.5766,-0.0000,
         0,0.0000,0.0000,0.5766,
         0,30.9689,-0.0000,0.0000,
         0,-0.0000,30.9689,-0.0000,
    0.0042,0.0000,0.0000,30.9689;

  // ROS stuff
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/vrpn_client_node/hopper/pose", 200, chatterCallback);

  quat_t quat_opti = quat_t(OptiState.q_w, OptiState.q_x, OptiState.q_y, OptiState.q_z);
  while (quat_opti.norm() < 0.99)
  {
    ros::spinOnce();
    quat_opti = quat_t(OptiState.q_w, OptiState.q_x, OptiState.q_y, OptiState.q_z);
    std::cout << "Waiting for optitrack quat" << std::endl;
    std::cout << quat_opti.coeffs().transpose() << std::endl;
  };

  signal(SIGINT, signal_callback_handler);
  setupSocketHardware();
  sleep(1);
  std::thread thread_object(getStateFromEthernet, std::ref(reset), std::ref(cv), std::ref(m));

  vector_3t current_vel, previous_vel;
  std::chrono::high_resolution_clock::time_point last_t_state_log;

  tstart = std::chrono::high_resolution_clock::now();
  t2 = tstart;

  while (ros::ok())
  {
    ros::spinOnce();

    {
      std::lock_guard<std::mutex> lck(state_mtx);
      quat_optitrack = quat_t(OptiState.q_w, OptiState.q_x, OptiState.q_y, OptiState.q_z);
      quat_optitrack.normalize();
      state << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - tstart).count() * 1e-3,
          OptiState.x, OptiState.y, OptiState.z,
          // quat_a.w(), quat_a.x(), quat_a.y(), quat_a.z(), // uncomment if you want optitrack as orientation
          ESPstate(6), ESPstate(7), ESPstate(8), ESPstate(9), // IMU as orientation
          OptiState.x_dot, OptiState.y_dot, OptiState.z_dot,
          ESPstate(3), ESPstate(4), ESPstate(5),
          ESPstate(10), ESPstate(11), ESPstate(12), ESPstate(0), ESPstate(1), ESPstate(2);
    }
    hopper->updateState(state);
    contact = hopper->state_.contact;
    quat_t IMU_quat = hopper->state_.quat;

    // Measure the initial absolute yaw (from optitrack)
    static scalar_t initial_yaw = extract_yaw(quat_optitrack);
    static quat_t initial_yaw_quat = Policy::Euler2Quaternion(0, 0, initial_yaw);

    // Remove the measured yaw to put us back in the global frame
    scalar_t measured_yaw = extract_yaw(hopper->state_.quat);
    scalar_t optitrack_yaw = extract_yaw(quat_optitrack);
    quat_t measured_yaw_quat = Policy::Euler2Quaternion(0, 0, measured_yaw);
    quat_t optitrack_yaw_quat = Policy::Euler2Quaternion(0, 0, optitrack_yaw);
    quat_t yaw_corrected = plus(optitrack_yaw_quat, minus(hopper->state_.quat, measured_yaw_quat));
    hopper->state_.quat = yaw_corrected;

    // Add roll pitch offset to body frame
    quat_t rollPitch = Policy::Euler2Quaternion(-offsets[0], -offsets[1], 0);
    hopper->state_.quat = plus(hopper->state_.quat, rollPitch);
    
    t1 = std::chrono::high_resolution_clock::now();
    scalar_t replan = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t2).count() * 1e-3;
    if (replan > p.dt)
    {
      desired_command = command->getCommand();
      quat_des = policy.DesiredQuaternion(hopper->state_, desired_command);
      
      // Add initial yaw to desired signal
      quat_des = plus(quat_des, initial_yaw_quat);
      omega_des = policy.DesiredOmega();
      u_des = policy.DesiredInputs(hopper->state_.wheel_vel, hopper->state_.contact);
      t2 = std::chrono::high_resolution_clock::now();
    }

    hopper->computeTorque(quat_des, omega_des, 0.1, u_des);
    for (int i = 1; i < 4; i++)
    {
      u_des(i - 1) = hopper->torque[i];
    }

    e = quat_des.inverse() * hopper->state_.quat;
    auto e_ = manif::SO3<scalar_t>(e);
    xi = e_.log();
    error << xi.coeffs();

    {
      std::lock_guard<std::mutex> lck(des_state_mtx);
      desstate[0] = quat_des.w();
      desstate[1] = quat_des.x();
      desstate[2] = quat_des.y();
      desstate[3] = quat_des.z();
      desstate[4] = omega_des(0);
      desstate[5] = omega_des(1);
      desstate[6] = omega_des(2);
      desstate[7] = u_des(0);
      desstate[8] = u_des(1);
      desstate[9] = u_des(2);
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
        fileHandle << state[0] << "," << hopper->state_.contact
                    // << "," << 1
                    << "," << hopper->state_.pos.transpose().format(CSVFormat)
                    << "," << hopper->state_.leg_pos
                    << "," << hopper->state_.vel.transpose().format(CSVFormat)
                    << "," << hopper->state_.leg_vel
                    // << "," << IMU_quat.coeffs().transpose().format(CSVFormat)
                    << "," << hopper->state_.quat.coeffs().transpose().format(CSVFormat)
                    << "," << quat_des.coeffs().transpose().format(CSVFormat)
                    << "," << hopper->state_.omega.transpose().format(CSVFormat)
                    << "," << hopper->torque.transpose().format(CSVFormat)
                    // << "," << error.transpose().format(CSVFormat)
                    << "," << hopper->state_.wheel_vel.transpose().format(CSVFormat)
                    << "," << command->getCommand().row(0).format(CSVFormat) << std::endl;
    }
  }
}

//roslaunch vrpn_client_ros fast.launch server:=169.254.10.83
//ifconfig enx5c857e36c21a 169.254.10.80