#include "../inc/ros_subscriber.hpp"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <thread>
#include <mutex>
#include <vector>

// Shared variables
std::vector<float> box_positions;
std::mutex box_positions_mutex;

// Callback function
void boxPositionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(box_positions_mutex);
    box_positions.clear();
    box_positions.insert(box_positions.end(), msg->data.begin(), msg->data.end());
    // ROS_INFO("Received box positions:");
    // for (size_t i = 0; i < msg->data.size(); i += 8)
    // {
    //     ROS_INFO("Box %lu: ", i / 8);
    //     for (size_t j = 0; j < 8; ++j)
    //     {
    //         ROS_INFO_STREAM(msg->data[i + j] << " ");
    //     }
    //     ROS_INFO("\n");
    // }
}

// ROS thread function
void rosThread(int argc, char **argv)
{
    ros::init(argc, argv, "box_position_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("box_positions", 1000, boxPositionCallback);
    ROS_INFO("Subscriber node started, waiting for messages...");
    ros::spin();
}

// Start ROS in a separate thread
void startRosNode(int argc, char **argv)
{
    std::thread(rosThread, argc, argv).detach();
}

// Access box positions
std::vector<float> getBoxPositions()
{
    std::lock_guard<std::mutex> lock(box_positions_mutex);
    return box_positions;
}
