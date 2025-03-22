#ifndef ROS_SUBSCRIBER_H
#define ROS_SUBSCRIBER_H

#include "local_mapper_interfaces/msg/polytope_array.hpp"
#include "pathPlanner.h"
#include "../estimatedState.h"
#include "../utils.h"

#include <thread>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

 
class FreePolytopeSubscriber : public rclcpp::Node
{
    public:
        FreePolytopeSubscriber();
        ObstacleCollector getFreePolytopePositions();

    private:
        rclcpp::Subscription<local_mapper_interfaces::msg::PolytopeArray>::SharedPtr subscription_;

        ObstacleCollector O_;
        std::mutex free_polytope_mutex_;

        void freePolytopeCallback(const local_mapper_interfaces::msg::PolytopeArray::SharedPtr msg);
};

class EstimateSubscriber : public rclcpp::Node
{
  public:
    EstimateSubscriber(std::shared_ptr<vector_3t> initial_pose);
    EstimatedState getEstimatedState();
    bool initialized_ = false;
    tf2::Quaternion Rc2h;
    tf2::Vector3 pc2h;
  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    void Callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    EstimatedState estimated_state_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<vector_3t> initial_pose_;
    matrix_2t initial_rot_;

    // vector_3t realsense_pos{0,0,0};
    vector_3t realsense_vel{0,0,0};
    vector_3t realsense_ang_vel{0,0,0};
    // vector_3t pos_origin{0,0,0};
    // Eigen::Quaternion<double> q;

    Eigen::Matrix<double, 3, 3> R_RS_to_RS_aligned;
    Eigen::Matrix<double, 3, 3> R_RS_aligned_to_H;
    Eigen::Matrix<double, 3, 3> R_H_to_RS;
    Eigen::Matrix<double, 3, 3> R_z_up;                
    Eigen::Matrix<double, 3, 3> R_error;
};

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher(std::shared_ptr<vector_3t> goal_pose, std::shared_ptr<vector_3t> initial_pose);
    vector_t graph_sol_;
    void setGraphSol(vector_t graph_sol);

private:
    std::shared_ptr<vector_3t> goal_pose_;
    std::shared_ptr<vector_3t> initial_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPublisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pathPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    matrix_2t initial_rot_;

    void send_goal();
};

void startRosNode(std::shared_ptr<FreePolytopeSubscriber> freePolySubscriber,
                std::shared_ptr<EstimateSubscriber> estimateSubscriber,
                std::shared_ptr<GoalPublisher> goalPublisher);

#endif // ROS_SUBSCRIBER_H
