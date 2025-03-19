#include "../../inc/planner/ros_subscriber.h"

FreePolytopeSubscriber::FreePolytopeSubscriber() : Node("free_polytopes_subscriber")
{
    subscription_ = this->create_subscription<local_mapper_interfaces::msg::PolytopeArray>(
        "free_polytopes", 10,
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
    obs.v.resize(4, 2);
    obs.A.resize(4, 4);
    obs.b.resize(4);
    obs.Adjacency.resize(4, 4);
    obs.Adjacency << 1, 0, 0, 1,
        1, 1, 0, 0,
        0, 1, 1, 0,
        0, 0, 1, 1;
    obs.occType = FREE;

    for (auto &polytope : msg->polytopes)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                obs.v(i, j) = polytope.vertices[i * 2 + j];
            }
        }

        std::cout << obs.v << std::endl << std::endl;

        std::vector<Eigen::Vector2d> edgeVectors(4);
        std::vector<Eigen::Vector2d> normals(4);

        edgeVectors[0] = Eigen::Vector2d(obs.v(3,0) - obs.v(0,0), obs.v(3,1) - obs.v(0,1));
        edgeVectors[1] = Eigen::Vector2d(obs.v(0,0) - obs.v(1,0), obs.v(0,1) - obs.v(1,1));
        edgeVectors[2] = Eigen::Vector2d(obs.v(1,0) - obs.v(2,0), obs.v(1,1) - obs.v(2,1));
        edgeVectors[3] = Eigen::Vector2d(obs.v(2,0) - obs.v(3,0), obs.v(2,1) - obs.v(3,1));
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
            obs.b(i) = -edgeVectors[i].transpose().dot(Eigen::Vector2d(obs.v(i,0), obs.v(i,1)));
        }

        O_.obstacles.push_back(obs);
    }
}

// Function to start ROS node in a separate thread
void startRosNode(std::shared_ptr<FreePolytopeSubscriber> freePolySubscriber)
{
    std::thread([freePolySubscriber]()
    {
        rclcpp::spin(freePolySubscriber);
        rclcpp::shutdown(); 
    }).detach();
}
