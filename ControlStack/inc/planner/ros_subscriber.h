#ifndef ROS_SUBSCRIBER_H
#define ROS_SUBSCRIBER_H

#include "local_mapper_interfaces/msg/polytope_array.hpp"
#include "pathPlanner.h"
#include "../estimatedState.h"

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
 
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
    EstimateSubscriber();
    EstimatedState getEstimatedState();
    bool initialized_ = false;
  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    void Callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    EstimatedState estimated_state_;
};

void startRosNode(std::shared_ptr<FreePolytopeSubscriber> freePolySubscriber, std::shared_ptr<EstimateSubscriber> estimateSubscriber);

#endif // ROS_SUBSCRIBER_H
