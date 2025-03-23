#include "../../inc/planner/ros_subscriber.h"

FreePolytopeSubscriber::FreePolytopeSubscriber(std::shared_ptr<vector_3t> initial_pose) : Node("free_polytopes_subscriber"), initial_pose_(initial_pose)
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

    matrix_2t initial_rot;
    initial_rot << cos((*initial_pose_)(2)), -sin((*initial_pose_)(2)),
                    sin((*initial_pose_)(2)), cos((*initial_pose_)(2));

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
        std::cout << obs.v.col(0).transpose().format(CSVFormat) << ","
                  << obs.v.col(1).transpose().format(CSVFormat) << ","
                  << obs.A.col(0).transpose().format(CSVFormat) << ","
                  << obs.A.col(1).transpose().format(CSVFormat) << ","
                  << obs.b.transpose().format(CSVFormat) << ",";
        obs.v = obs.v * initial_rot;
        obs.v.col(0).array() -= (*initial_pose_)(0);
        obs.v.col(1).array() -= (*initial_pose_)(1);
        obs.A.block(0,0,num_pts,2) = obs.A.block(0,0,num_pts,2) * initial_rot;
        obs.b -= obs.A.block(0,0,num_pts,2)*((*initial_pose_).segment(0,2));
        std::cout << obs.v.col(0).transpose().format(CSVFormat) << ","
                  << obs.v.col(1).transpose().format(CSVFormat) << ","
                  << obs.A.col(0).transpose().format(CSVFormat) << ","
                  << obs.A.col(1).transpose().format(CSVFormat) << ","
                  << obs.b.transpose().format(CSVFormat) << std::endl;

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

    if (!initialized_) {
        (*initial_pose_)(2) = extract_yaw(body_q);
        initial_rot_ << cos((*initial_pose_)(2)),-sin((*initial_pose_)(2)),
                        sin((*initial_pose_)(2)),cos((*initial_pose_)(2));
    }
    static quat_t inv_yaw_quat = Euler2Quaternion(0,0,-(*initial_pose_)(2));
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
    if (!initialized_) {
        (*initial_pose_).segment(0, 2) = pos_origin.segment(0, 2);
    }

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

    initialized_ = true;
}

EstimateSubscriber::EstimateSubscriber(std::shared_ptr<vector_3t> initial_pose): 
                                       Node("estimate_subscriber"), tf_buffer_(this->get_clock()),
                                       tf_listener_(tf_buffer_), initial_pose_(initial_pose)
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
    R_error << 0.9995,    0.0000,    0.0305,
               0.0006,    0.9998,   -0.0194,
              -0.0305,    0.0194,    0.9993;
    R_H_to_RS = R_error.transpose() * R_RS_to_RS_aligned * R_RS_aligned_to_H;

    R_z_up << 1,0,0,
              0,0,-1,
              0,1,0;
}



GoalPublisher::GoalPublisher(std::shared_ptr<vector_3t> goal_pose, std::shared_ptr<vector_3t> initial_pose) :
                             Node("goal_publisher"), goal_pose_(goal_pose), initial_pose_(initial_pose) {
    goalPublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    pathPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/graph_solve", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GoalPublisher::send_goal, this)
    );
    initial_rot_ << cos((*initial_pose_)(2)), -sin((*initial_pose_)(2)),
                    sin((*initial_pose_)(2)), cos((*initial_pose_)(2));
}

void GoalPublisher::send_goal() {
    initial_rot_ << cos((*initial_pose_)(2)), -sin((*initial_pose_)(2)),
                    sin((*initial_pose_)(2)), cos((*initial_pose_)(2));

    auto goal = geometry_msgs::msg::PoseStamped();
    goal.header.frame_id = "odom";
    goal.header.stamp = this->get_clock()->now();
    vector_2t global_goal_pos = initial_rot_ * ((*goal_pose_).segment(0, 2) + (*initial_pose_).segment(0, 2));
    goal.pose.position.x = global_goal_pos(0);
    goal.pose.position.y = global_goal_pos(1);
    quat_t quat = Euler2Quaternion(0, 0, (*goal_pose_)(2) + (*initial_pose_)(2));
    goal.pose.orientation.x = quat.x();
    goal.pose.orientation.y = quat.y();
    goal.pose.orientation.z = quat.z();
    goal.pose.orientation.w = quat.w();

    goalPublisher_->publish(goal);
    // RCLCPP_INFO(this->get_logger(), "Goal sent: [%.2f, %.2f]", goal.pose.position.x, goal.pose.position.y);

    // Create the marker message
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "odom";  // Adjust frame as needed
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "graph_solve";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker properties
    marker.scale.x = 0.05;  // Point size
    // marker.scale.y = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Convert Eigen vector to marker points'
    for (int i  = 0; i < graph_sol_.size()/4; i++) {
        if (i > 0 && graph_sol_(i*4) == 0 && graph_sol_(i*4 + 1) == 0 && 
                    graph_sol_(i*4 + 2) == 0 && graph_sol_(i*4 + 3) == 0) {
            break;
        }
        geometry_msgs::msg::Point ros_point;
        vector_2t point = graph_sol_.segment(i*4, 2);
        point << initial_rot_ * (point + (*initial_pose_).segment(0, 2));
        ros_point.x = point(0);
        ros_point.y = point(1);
        marker.points.push_back(ros_point);
    }

    // Publish the marker
    pathPublisher_->publish(marker);
    // RCLCPP_INFO(this->get_logger(), "Published marker with %ld points", marker.points.size());
}

void GoalPublisher::setGraphSol(vector_t graph_sol) {
    graph_sol_ = graph_sol;
}
