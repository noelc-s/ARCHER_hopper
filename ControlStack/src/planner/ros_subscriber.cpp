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
void startRosNode(std::shared_ptr<FreePolytopeSubscriber> freePolySubscriber)
{
    std::thread([freePolySubscriber]()
    {
        rclcpp::spin(freePolySubscriber);
        rclcpp::shutdown(); 
    }).detach();
}
