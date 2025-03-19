#ifndef ROS_SUBSCRIBER_H
#define ROS_SUBSCRIBER_H

#include "local_mapper_interfaces/msg/polytope_array.hpp"
#include "pathPlanner.h"

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <vector>
 
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

void startRosNode(std::shared_ptr<FreePolytopeSubscriber> freePolySubscriber);

#endif // ROS_SUBSCRIBER_H