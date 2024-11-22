#include "../inc/rom.h"
#include <thread>
#include <fstream>

V5Command::V5Command(const scalar_t x0, const scalar_t y0) : x0_(x0), y0_(y0)
{
    command.resize(5);
    command.setZero();
}

void V5Command::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
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

void SingleIntCommand::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
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

PredCBFCommand::PredCBFCommand(
    const double horizon, const double dt, const double alpha, const double rho, const bool smooth_barrier, const double epsilon,
    const double k_r, const double v_max, const double pred_dt, const int iters, const double K, const double tol, const bool use_delta,
    const bool use_barrier, const std::vector<double> rs, const std::vector<double> cxs, const std::vector<double> cys, const vector_2t zd
) : horizon_(horizon), dt_(dt), alpha_(alpha), rho_(rho), smooth_barrier_(smooth_barrier), epsilon_(epsilon), iters_(iters), K_(K), tol_(tol), use_delta_(use_delta),
    use_barrier_(use_barrier), k_r_(k_r), v_max_(v_max), pred_dt_(pred_dt), rs_(rs), cxs_(cxs), cys_(cys), zd_(zd), num_obs_(rs.size())
{
    // Resize the command
    command_.resize(5, 1);
    command_.setZero();
    delta_ = 0;
    num_runs_ = 0;

    char path[] = "../rsc/";
    char xmlfile[] = "hopper.xml";
    // char xmlfile[] = "hopper_2.xml";  // CoM +2cm in x,y
    char xmlpath[100] = {};
    char datapath[100] = {};

    strcat(xmlpath, path);
    strcat(xmlpath, xmlfile);

    strcat(datapath, path);


    // load and compile model
    char error[1000] = "Could not load binary model";
    m_ = mj_loadXML(xmlpath, 0, error, 1000);
    if (!m_) {
        mju_error_s("Load model error: %s", error);
    }
    // make data
    m_->opt.timestep = 0.001;
    d_ = mj_makeData(m_);

    // Initialize Hopper and Raibert
    hopper_ = std::shared_ptr<Hopper>(new Hopper(gainYamlPath));


    // Process the size of obstacle arrays
    assert(cxs_.size() == num_obs_ && cys_.size() == num_obs_);
}

void PredCBFCommand::update(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
{
    // TODO: update dynamics
    while (running)
    {
        auto start = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(m);
            zd_ << userInput->joystick_command[0], userInput->joystick_command[1];
            // std::cout << zd_.transpose() << ";  " << userInput->joystick_command.transpose() << std::endl;


            vector_2t z;
            z << state.pos[0], state.pos[1];
            vector_2t v = robustifiedSafetyFilter(z, vd(z));

            // Compute command to send to hopper for tracking
            command_.block<2, 1>(0, 0) = z + v * pred_dt_;
            command_.block<2, 1>(2, 0) = v;
            command_(4, 0) = userInput->joystick_command(2);
            // std::cout << "\nz: "<<  z << "\nv: " << v << "\ndt: " << pred_dt_ << "\ncommand: " << command_ << std::endl;

        }
        // std::cout << command(0, 0) << ',' << command(0, 1) << std::endl;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::chrono::milliseconds sleep_duration(static_cast<int>((dt_ - elapsed.count()) * 1000));
        if (sleep_duration.count() > 0)
        {
            std::this_thread::sleep_for(sleep_duration);
        }
    }
}

void PredCBFCommand::update_delta(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
{
    // TODO: update dynamics
    Hopper::State copied_state;

    while (state.quat.coeffs().norm() < 0.99) {}
    std::cout << state.quat.coeffs() << std::endl;

    while (running)
    {
        auto start = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(m);
            // copy state
            copied_state = state;
        }
        predictiveSafetyFilter(copied_state);
    }
}

vector_3t PredCBFCommand::h_Dh(vector_2t z) {
    double h = 0;
    vector_2t dh;
    dh.setZero();
    double dh_denom = 0;
    for (int i = 0; i < num_obs_; i++) {
        double r = rs_[i];
        vector_2t c;
        c << cxs_[i], cys_[i];
        double d = (z - c).norm() - r;
        if (smooth_barrier_) {
            double exp_d = std::exp(-rho_ * d);
            h += exp_d;
            dh += (z - c) / (z  - c).norm() * exp_d;
            dh_denom += exp_d;
        } else if (d < h || i == 0) {
            h = d;
            dh = (z - c) / (z - c).norm();
        }
    }
    if (smooth_barrier_) {
        h = -1 / rho_ * std::log(h);
        dh = dh / dh_denom;
    }
    return vector_3t(h, dh(0), dh(1));
}

vector_2t PredCBFCommand::predictiveSafetyFilter(Hopper::State &state) {
    // return vd;
    // Loop over iterations of the algorithm
    double prev_delta = delta_;
    bool converged = false;
    for (int i = 0; i < iters_; i++) {
        // Roll out planner/tracker
        double h_bar;
        if (use_delta_) {
            h_bar = robustifiedRollout(state);
        } else {
            h_bar = 0;
        }

        // Update robustness term based on barrier violation
        delta_ = std::max(0., delta_ - K_ * h_bar);
        // std::cout << delta_ << std::endl;
        // Check for convergence
        if (abs(delta_ - prev_delta) < tol_) {
            converged = true;
            // std::cout << "Pred CBF Converged...     Delta: " << delta_ << std::endl;
            break;
        }
        prev_delta = delta_;  // Update previous delta for checking convergence
    }
    if (!converged) {
        // std::cout << "Pred CBF NOT Converged!!! Delta: " << delta_ << std::endl;
    }
    // Apply safety filter with robustness term
    vector_2t z;
    z << state.pos[0], state.pos[1];
    // std::cout << "rollouts" << std::endl;
    return robustifiedSafetyFilter(z, vd(z));
}

double PredCBFCommand::robustifiedRollout(Hopper::State &state) {
    std::string dataLog = "../data/data_inner" + std::to_string(num_runs_) + ".csv";
    num_runs_++;
    std::ofstream fileHandle;
    fileHandle.open(dataLog);
    fileHandle << "t,contact,x,y,z,legpos,vx,vy,vz,legvel,q_x,q_y,q_z,q_w,qd_x,qd_y,qd_z,qd_w,w_1,w_2,w_3,tau_foot,tau1,tau2,tau3,wheel_vel1,wheel_vel2,wheel_vel3,cmdx,cmdy,cmdvx,cmdvy,cmdyaw,h,Jh1,Jh2,vdx,vdy,vx,vy,delta" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    static quat_t initial_yaw_quat = Euler2Quaternion(0, 0, extract_yaw(state.quat));
    // Set up the initial condition
    d_->qpos[0] = state.pos[0];    // x
    d_->qpos[1] = state.pos[1];    // y
    d_->qpos[2] = state.pos[2];    // z
    d_->qpos[3] = state.quat.w();  // qw
    d_->qpos[4] = state.quat.x();  // qx
    d_->qpos[5] = state.quat.y();  // qy
    d_->qpos[6] = state.quat.z();  // qz
    d_->qpos[10] = state.leg_pos;  // foot pos, note we don't care about 7-9, flywheel positions.

    d_->qvel[0] = state.vel[0];    // vx
    d_->qvel[1] = state.vel[1];    // vy
    d_->qvel[2] = state.vel[2];    // vz
    d_->qvel[3] = state.omega[0];  // wx
    d_->qvel[4] = state.omega[1];  // wy
    d_->qvel[5] = state.omega[2];  // wz
    d_->qvel[9] = state.leg_vel;   // foot vel
    d_->qvel[6] = state.wheel_vel[0];   // foot vel
    d_->qvel[7] = state.wheel_vel[1];   // foot vel
    d_->qvel[8] = state.wheel_vel[2];   // foot vel

    vector_2t z, v;
    vector_t tmp_command;
    vector_3t h_dh;
    tmp_command.resize(5, 1);
    tmp_command.setZero();
    vector_3t omega_des;
    omega_des.setZero();
    vector_t u_des(4);
    u_des.setZero();

    vector_t tmp_state;
    tmp_state.resize(20);
    tmp_state.setZero();
    quat_t quat_des;

    double h_bar = INFINITY;
    double num_steps = horizon_ / m_->opt.timestep;
    for (int k = 0; k < num_steps; k++) {
        // Parse current state
        int ind = 0;
        tmp_state(0) = d_->time;
        ind++;
        for (int i = 0; i < 3; i++) {
            tmp_state(ind) = d_->qpos[i];
            ind++;
        }
        for (int i = 0; i < 4; i++) {
            tmp_state(ind) = d_->qpos[i + 3];
            ind++;
        }
        for (int i = 0; i < 6; i++) {
            tmp_state(ind) = d_->qvel[i];
            ind++;
        }
        // Threshold for registering contact
        scalar_t contact_threshold = -0.003;
        tmp_state(ind) = (d_->contact[0].dist < contact_threshold);
        ind++;
        tmp_state(ind) = d_->qpos[10];
        ind++;
        tmp_state(ind) = d_->qvel[9];
        ind++;
        tmp_state(ind) = d_->qvel[6];
        ind++;
        tmp_state(ind) = d_->qvel[7];
        ind++;
        tmp_state(ind) = d_->qvel[8];

        hopper_->updateState(tmp_state);

        // Update safety violation
        z << d_->qpos[0], d_->qpos[1];
        h_dh = h_Dh(z);
        if (use_barrier_) {
            vector_2t Dh = h_dh.segment<2>(1);
            vector_2t fz;
            fz << d_->qvel[0], d_->qvel[1];
            float viol = Dh.dot(fz.segment<2>(0)) + alpha_ * h_dh(0);
            if (viol < h_bar) {
                h_bar = viol;
            }
        } else {
            if (h_dh(0) < h_bar) {
                h_bar = h_dh(0);
            }
        }
        
        // Get robustified RoM action
        v = robustifiedSafetyFilter(z, vd(z));
        tmp_command.block<2, 1>(0, 0) = z + v * pred_dt_;
        tmp_command.block<2, 1>(2, 0) = v;

        // Compute desired quaternion TODO add raibert policy
        quat_des = raibert_policy_.DesiredQuaternion(hopper_->state_, tmp_command);
        quat_des = plus(quat_des, initial_yaw_quat);
        omega_des = raibert_policy_.DesiredOmega();
        u_des = raibert_policy_.DesiredInputs(hopper_->state_.wheel_vel, hopper_->state_.contact);
        hopper_->computeTorque(quat_des, omega_des, 0.1, u_des);

        // Update torques
        vector_3t g_x(1,1,1);
        if (d_->qvel[6] > 500) {
            g_x[0] = std::max(-100*(d_->qvel[6]-600),0.);
        } else if (d_->qvel[6] < -500) {
            g_x[0] = std::max(100*(d_->qvel[6]+600),0.);
        }
        if (d_->qvel[7] > 500) {
            g_x[1] = std::max(-100*(d_->qvel[7]-600),0.);
        } else if (d_->qvel[7] < -500) {
            g_x[1] = std::max(100*(d_->qvel[7]+600),0.);
        }
        if (d_->qvel[8] > 500) {
            g_x[2] = std::max(-100*(d_->qvel[8]-600),0.);
        } else if (d_->qvel[8] < -500) {
            g_x[2] = std::max(100*(d_->qvel[8]+600),0.);
        }
        d_->ctrl[0] = hopper_->torque[0];
        for (int i = 0; i < 3; i++) {
            d_->ctrl[i+1] = g_x[i]*hopper_->torque[i+1];
        }
        // Step Dynamics
        mj_step(m_, d_);
        fileHandle << d_->time << "," << hopper_->state_.contact
                       << "," << hopper_->state_.pos.transpose().format(CSVFormat)
                       << "," << hopper_->state_.leg_pos
                       << "," << hopper_->state_.vel.transpose().format(CSVFormat)
                       << "," << hopper_->state_.leg_vel
                       << "," << hopper_->state_.quat.coeffs().transpose().format(CSVFormat)
                       << "," << quat_des.coeffs().transpose().format(CSVFormat)
                       << "," << hopper_->state_.omega.transpose().format(CSVFormat)
                       << "," << hopper_->torque.transpose().format(CSVFormat)
                       << "," << hopper_->state_.wheel_vel.transpose().format(CSVFormat)
                       << "," << command_.col(0).transpose().format(CSVFormat)
                       << "," << h_Dh(z).transpose().format(CSVFormat)
                       << "," << vd(z).transpose().format(CSVFormat)
                       << "," << robustifiedSafetyFilter(z, vd(z)).transpose().format(CSVFormat)
                       << "," << delta_;
            fileHandle << std::endl;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    // std::cout << elapsed.count() << std::endl;
    // throw std::runtime_error("I'm stopping here...");
    return h_bar;
}

vector_2t PredCBFCommand::robustifiedSafetyFilter(vector_2t z, vector_2t vd) {
    // Compute h and its jacobian at z
    vector_3t h_dh = h_Dh(z);
    double h = h_dh(0);
    vector_2t Dh = h_dh.segment<2>(1);

    // Create compact notation for safety filter
    vector_2t b = Dh;                 // Lgh, but gz is identity for single int
    double a;
    if (use_delta_) {
        a = -alpha_ * h + delta_;  // -alpha * h - Lfh + delta, but f is zero for single int
    } else {
        a = -alpha_ * h;
    }

    vector_2t v;
    // Implement safety filter
    if (b.dot(vd) >= a) {
        v = vd;
    } else {
        v = vd + b / (b.dot(b)) * (a - b.dot(vd));
    }
    return v;
}

vector_2t PredCBFCommand::vd(vector_2t z) {
    vector_2t vd = -k_r_ * (z - zd_);
    if (vd.norm() > v_max_) {
        vd = vd / vd.norm() * v_max_;
    }
    return vd;
}

NNPredCBFCommand::NNPredCBFCommand(
    const double horizon, const double dt, const double alpha, const double rho, const bool smooth_barrier, const double epsilon,
    const double k_r, const double v_max, const double pred_dt, const int iters, const double K, const double tol, const bool use_delta,
    const bool use_barrier, const std::vector<double> rs, const std::vector<double> cxs, const std::vector<double> cys, const vector_2t zd,
    std::string delta_model) : PredCBFCommand(
        horizon, dt, alpha, rho, smooth_barrier, epsilon, k_r, v_max, pred_dt, iters, K, tol, use_delta, use_barrier, rs, cxs, cys, zd
    ), delta_model_(delta_model)
{

    // Initialize NN Things
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "example-model-explorer");
    Ort::SessionOptions session_options;
    session = std::make_unique<Ort::Session>(Ort::Session(env, delta_model_.c_str(), session_options));

    inputNodeName = session->GetInputNameAllocated(0, allocator).get();
    outputNodeName = session->GetOutputNameAllocated(0, allocator).get();

    inputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetInputTypeInfo(0));
    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    inputType = inputTensorInfo.GetElementType();
    inputDims = inputTensorInfo.GetShape();
    inputDims[0] = 1; // hard code batch size of 1 for evaluation

    outputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetOutputTypeInfo(0));
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();
    outputType = outputTensorInfo.GetElementType();
    outputDims = outputTensorInfo.GetShape();
    outputDims[0] = 1; // hard code batch size of 1 for evaluation

    inputTensorSize = vectorProduct(inputDims);
    outputTensorSize = vectorProduct(outputDims);
}

void NNPredCBFCommand::update_delta(UserInput *userInput, std::atomic<bool> &running, std::condition_variable &cv, std::mutex &m, Hopper::State &state)
{
    // TODO: update dynamics
    Hopper::State copied_state;

    while (state.quat.coeffs().norm() < 0.99) {}
    std::cout << state.quat.coeffs() << std::endl;

    vector_t nn_input(6);
    nn_input.setZero();

    while (running)
    {
        auto start = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(m);
            // copy state
            copied_state = state;
        }
        // TODO: Replace with NN call
        nn_input << copied_state.pos, copied_state.vel;
        EvaluateNetwork(nn_input);
        // predictiveSafetyFilter(copied_state);
    }
}

void NNPredCBFCommand::EvaluateNetwork(const vector_t& input) {
    std::vector<float> input_(6);
    input_[0] = input(0);
    input_[1] = input(1);
    input_[2] = input(2);
    input_[3] = input(3);
    input_[4] = input(4);
    input_[5] = input(5);

    std::vector<float> output_(1);

    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, const_cast<float *>(input_.data()), inputTensorSize,
        inputDims.data(), inputDims.size());

    Ort::Value outputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, output_.data(), outputTensorSize,
        outputDims.data(), outputDims.size());

    std::vector<const char *> inputNames{inputNodeName.c_str()};
    std::vector<const char *> outputNames{outputNodeName.c_str()};

    session->Run(Ort::RunOptions{}, inputNames.data(), &inputTensor, 1, outputNames.data(), &outputTensor, 1);

    delta_ = std::min(std::max(output_[0], 0.0f), 2.0f);
    // std::cout << "NN" << std::endl;
}