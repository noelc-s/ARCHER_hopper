#include "../../inc/planner/ros_subscriber.h"

FreePolytopeSubscriber::FreePolytopeSubscriber() : Node("free_polytopes_subscriber")
{
    subscription_ = this->create_subscription<local_mapper_interfaces::msg::PolytopeArray>(
        "free_polytopes", 1,
        std::bind(&FreePolytopeSubscriber::freePolytopeCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscriber node started, waiting for messages...");
}

ObstacleCollector FreePolytopeSubscriber::getFreePolytopePositions()
{
    std::lock_guard<std::mutex> lock(free_polytope_mutex_);
    return O_;
}

void FreePolytopeSubscriber::freePolytopeCallback(const local_mapper_interfaces::msg::PolytopeArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(free_polytope_mutex_);
    O_.obstacles.clear();

    Obstacle obs;
    obs.center.resize(2);
    obs.center.setZero();
    obs.occType = FREE;

    for (auto &polytope : msg->polytopes)
    {
        int num_pts = polytope.vertices.size() / 2;
        obs.v.resize(num_pts, 2);
        obs.A.resize(num_pts, 4);
        obs.A.setZero();
        obs.b.resize(num_pts);

        for (int i = 0; i < num_pts; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                obs.v(i, j) = polytope.vertices[i * 2 + j];
                obs.A(i, j) = polytope.normals[i * 2 + j];
            }
            obs.b(i) = polytope.b[i];
        }

        // std::cout << obs.v << std::endl << std::endl;

        // std::vector<Eigen::Vector2d> edgeVectors(num_pts);
        // std::vector<Eigen::Vector2d> normals(num_pts);

        // for (int i = 0; i < num_pts; i++) {
        //     edgeVectors[i] = Eigen::Vector2d(obs.v(i,0) - obs.v((i+1) % num_pts,0),
        //                                  obs.v(i,1) - obs.v((i+1) % num_pts,1));
        //     edgeVectors[i].normalize();
        // }
        // edgeVectors[0] = Eigen::Vector2d(obs.v(3,0) - obs.v(0,0), obs.v(3,1) - obs.v(0,1));
        // edgeVectors[1] = Eigen::Vector2d(obs.v(0,0) - obs.v(1,0), obs.v(0,1) - obs.v(1,1));
        // edgeVectors[2] = Eigen::Vector2d(obs.v(1,0) - obs.v(2,0), obs.v(1,1) - obs.v(2,1));
        // edgeVectors[3] = Eigen::Vector2d(obs.v(2,0) - obs.v(3,0), obs.v(2,1) - obs.v(3,1));
        // edgeVectors[0].normalize();
        // edgeVectors[1].normalize();
        // edgeVectors[2].normalize();
        // edgeVectors[3].normalize();

        // Construct A and b
        // for (int i = 0; i < num_pts; ++i)
        // {
        //     obs.A.row(i) << -edgeVectors[i].transpose(), 0, 0;
        //     obs.b(i) = -edgeVectors[i].transpose().dot(Eigen::Vector2d(obs.v(i,0), obs.v(i,1)));
        // }

        O_.obstacles.push_back(obs);
    }
}

// Function to start ROS node in a separate thread
void startRosNode(std::shared_ptr<FreePolytopeSubscriber> freePolySubscriber,
                  std::shared_ptr<EstimateSubscriber> estimateSubscriber,
                  std::shared_ptr<GoalPublisher> goalPublisher)
{
    std::thread([freePolySubscriber]()
    {
        rclcpp::spin(freePolySubscriber);
        rclcpp::shutdown(); 
    }).detach();

    std::thread([estimateSubscriber]()
    {
        rclcpp::spin(estimateSubscriber);
        rclcpp::shutdown(); 
    }).detach();

    std::thread([goalPublisher]()
    {
        rclcpp::spin(goalPublisher);
        rclcpp::shutdown(); 
    }).detach();
}

EstimatedState EstimateSubscriber::getEstimatedState() {
	return estimated_state_;
}

void EstimateSubscriber::Callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Get the hopper pose
    geometry_msgs::msg::TransformStamped hopper_pose =
        tf_buffer_.lookupTransform("odom", "hopper", rclcpp::Time(0));

    // std::cout << msg->header.stamp.nanosec << std::endl;

    realsense_vel << msg->twist.twist.linear.x,
                     msg->twist.twist.linear.y,
                     msg->twist.twist.linear.z;
    realsense_ang_vel << msg->twist.twist.angular.x,
                         msg->twist.twist.angular.y,
                         msg->twist.twist.angular.z;

    // Now, remove the initial yaw from everything
    // body quat
    quat_t body_q = Eigen::Quaternion<double>(hopper_pose.transform.rotation.w, hopper_pose.transform.rotation.x, hopper_pose.transform.rotation.y, hopper_pose.transform.rotation.z);

    static scalar_t initial_yaw = extract_yaw(
        body_q
    );
    static quat_t inv_yaw_quat = Euler2Quaternion(0,0,-initial_yaw);
    body_q = inv_yaw_quat * body_q;

    // cam quat
    // THE ROS MODULE IS DOING SOME TRANSFORMATIONS. WE ARE UNDOING THEM
    quat_t cam_q = Eigen::Quaternion<double>(msg->pose.pose.orientation.w, -msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, -msg->pose.pose.orientation.x);

    // Transform velocities into local frame
    realsense_vel = R_H_to_RS.transpose()*(cam_q.inverse() * realsense_vel);
    realsense_ang_vel = R_H_to_RS.transpose()*(cam_q.inverse() * realsense_ang_vel); // transform to local vel

    cam_q = inv_yaw_quat * cam_q;

    // global position
    vector_3t global_pos;
    global_pos << hopper_pose.transform.translation.x, hopper_pose.transform.translation.y, hopper_pose.transform.translation.z;
    global_pos = inv_yaw_quat * global_pos;
    static vector_3t pos_origin = global_pos;

    // camera position
    vector_3t cam_pos;
    cam_pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    cam_pos = inv_yaw_quat * cam_pos;

    // linear velocity
    vector_3t lin_body_vel = realsense_vel;

    // angular velocity
    vector_3t ang_body_vel = realsense_ang_vel;
    
    // Quat from camera
    estimated_state_.cam_q_w = cam_q.w();
    estimated_state_.cam_q_x = cam_q.x();
    estimated_state_.cam_q_y = cam_q.y();
    estimated_state_.cam_q_z = cam_q.z();
    // body quaternion, with initial yaw removed
    estimated_state_.q_w = body_q.w();
    estimated_state_.q_x = body_q.x();
    estimated_state_.q_y = body_q.y();
    estimated_state_.q_z = body_q.z();
    // global position (initial yaw removed)
    estimated_state_.x = global_pos(0) - pos_origin(0);          
    estimated_state_.y = global_pos(1) - pos_origin(1);
    estimated_state_.z = global_pos(2) - pos_origin(2);
    // camera position (global frame, initial yaw removed)
    estimated_state_.cam_x = cam_pos(0) - pos_origin(0);
    estimated_state_.cam_y = cam_pos(1) - pos_origin(1);
    estimated_state_.cam_z = cam_pos(2) - pos_origin(2);
    // linear velocity, body frame
    estimated_state_.x_dot = lin_body_vel(0);
    estimated_state_.y_dot = lin_body_vel(1);
    estimated_state_.z_dot = lin_body_vel(2);
    // angular velocity, body frame
    estimated_state_.omega_x = ang_body_vel(0);
    estimated_state_.omega_y = ang_body_vel(1);
    estimated_state_.omega_z = ang_body_vel(2);

    estimated_state_.initial_yaw = initial_yaw;

    initialized_ = true;

    // // Realsense position, velocity, angular velocity (in camera frame)
    // realsense_pos << msg->pose.pose.position.x,
    //                 msg->pose.pose.position.y,
    //                 msg->pose.pose.position.z;
    // realsense_vel << msg->twist.twist.linear.x,
    //                   msg->twist.twist.linear.y,
    //                    msg->twist.twist.linear.z;
    // realsense_ang_vel << msg->twist.twist.angular.x,
    //                      msg->twist.twist.angular.y,
    //                      msg->twist.twist.angular.z;

    // // realsense orientation
    // q = Eigen::Quaternion<double>(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    // // Transform velocities into local frame
    // realsense_vel = R_H_to_RS.transpose()*(q.inverse() * realsense_vel);
    // realsense_ang_vel = R_H_to_RS.transpose()*(q.inverse() * realsense_ang_vel); // transform to local vel

    // // Body orientation (correct for where identity quat is, and cam to body transform R)
    // quat_t body_q = quat_t(R_z_up) * q * quat_t(R_H_to_RS);

    // // Remove initial yaw (continuing to correct global identity)
    // static scalar_t initial_yaw = extract_yaw(body_q);
    // quat_t inv_yaw_quat = Euler2Quaternion(0,0,-initial_yaw);
    // body_q = inv_yaw_quat * body_q;

    // // Transform position into the global frame
    // vector_t camera_pos = inv_yaw_quat * quat_t(R_z_up) * realsense_pos;    // Camera pos with z up (and initial yaw removed)
    // static vector_t p0  = camera_pos + body_q * r_cam_to_body;              // Hopper initial position global frame
    // vector_t global_pos = camera_pos + body_q * r_cam_to_body - p0;         // Hopper current position global frame
    
    // estimated_state_.cam_q_w = q.w();                            // Quat from camera
    // estimated_state_.cam_q_x = q.x();
    // estimated_state_.cam_q_y = q.y();
    // estimated_state_.cam_q_z = q.z();
    // estimated_state_.q_w = body_q.w();                           // body quaternion, with initial yaw removed
    // estimated_state_.q_x = body_q.x();
    // estimated_state_.q_y = body_q.y();
    // estimated_state_.q_z = body_q.z();
    // estimated_state_.x = global_pos(0) - pos_origin(0);          // global position (initial yaw removed)
    // estimated_state_.y = global_pos(1) - pos_origin(1);
    // estimated_state_.z = global_pos(2) - pos_origin(2);
    // estimated_state_.cam_x = camera_pos(0) - pos_origin(0);      // camera position (global frame, initial yaw removed)
    // estimated_state_.cam_y = camera_pos(1) - pos_origin(1);
    // estimated_state_.cam_z = camera_pos(2) - pos_origin(2);
    // estimated_state_.x_dot = realsense_vel(0);                   // linear velocity, body frame
    // estimated_state_.y_dot = realsense_vel(1);
    // estimated_state_.z_dot = realsense_vel(2);
    // estimated_state_.omega_x = realsense_ang_vel(0);             // angular velocity, body frame
    // estimated_state_.omega_y = realsense_ang_vel(1);
    // estimated_state_.omega_z = realsense_ang_vel(2);

	// initialized_ = true;
}

EstimateSubscriber::EstimateSubscriber() : Node("estimate_subscriber"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "rs_t265/odom", 10,
        std::bind(&EstimateSubscriber::Callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscriber node started, waiting for messages...");


    // Lookup the transform from camera frame to body frame
    while (!tf_buffer_.canTransform("t265_frame", "hopper", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0))) {
	    std::cout <<  "Transform from t265_frame to hopper not available yet." << std::endl;
        sleep(0.001);
    }
    geometry_msgs::msg::TransformStamped cam_to_hopper =
    tf_buffer_.lookupTransform("t265_frame", "hopper", rclcpp::Time(0));
    while (!tf_buffer_.canTransform("hopper", "odom", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0))) {
	    std::cout <<  "Transform from hopper to odom not available yet." << std::endl;
        sleep(0.001);
    }

    Rc2h = tf2::Quaternion(cam_to_hopper.transform.rotation.x,
                         cam_to_hopper.transform.rotation.y,
                         cam_to_hopper.transform.rotation.z,
                         cam_to_hopper.transform.rotation.w);
    pc2h = tf2::Vector3(cam_to_hopper.transform.translation.x,
                      cam_to_hopper.transform.translation.y,
                      cam_to_hopper.transform.translation.z);




    // Magic numbers from solidworks macro
    R_RS_to_RS_aligned << 0.271397251061513,    0.863510292339979,  0.425080588993638,
                            0.962467418729722,   -0.243493249790936, -0.119864528489434,
                            1.01307850997046E-15, 0.441657120772634, -0.897183920760302;
    R_RS_aligned_to_H << 0, 1, 0,
                            0, 0, 1,
                            1, 0, 0;
    R_error <<0.9995,    0.0000,    0.0305,
                0.0006,    0.9998,   -0.0194,
                -0.0305,    0.0194,    0.9993;
    R_H_to_RS = R_error.transpose() * R_RS_to_RS_aligned * R_RS_aligned_to_H;

    R_z_up << 1,0,0,
                0,0,-1,
                0,1,0;
}



GoalPublisher::GoalPublisher(std::shared_ptr<vector_3t> goal_pos) : Node("goal_publisher"), goal_pos_(goal_pos) {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GoalPublisher::send_goal, this)
    );
}

void GoalPublisher::send_goal() {
    auto goal = geometry_msgs::msg::PoseStamped();
    goal.header.frame_id = "odom";
    goal.header.stamp = this->get_clock()->now();
    goal.pose.position.x = (*goal_pos_)(0);
    goal.pose.position.y = (*goal_pos_)(1);
    quat_t quat = Euler2Quaternion(0, 0, (*goal_pos_)(2));
    goal.pose.orientation.x = quat.x();
    goal.pose.orientation.y = quat.y();
    goal.pose.orientation.z = quat.z();
    goal.pose.orientation.w = quat.w();

    publisher_->publish(goal);
    // RCLCPP_INFO(this->get_logger(), "Goal sent: [%.2f, %.2f]", goal.pose.position.x, goal.pose.position.y);
}