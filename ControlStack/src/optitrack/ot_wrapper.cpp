#include "../../inc/optitrack/ot_interface.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/String.h"
#include <thread>

class OTWrapper : public OTInterface
{
public:
    std::shared_ptr<EstimatedState> optiState_;
    ros::Subscriber sub;
    matrix_t A_kf;
    matrix_t B_kf;

    OTWrapper(std::shared_ptr<EstimatedState> optiState): optiState_(optiState) {
        // ROS stuff
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "listener");
        ros::NodeHandle n;

        A_kf.resize(6,6);
        B_kf.resize(6,4);
        A_kf << 0.4234, 0.0000, -0.0000, 0.0042, 0, 0,
                0.0000, 0.4234, 0.0000, 0, 0.0042, 0,
               -0.0000, -0.0000, 0.4234, 0, 0, 0.0042,
               -30.9689, 0.0000, -0.0000, 1.0000, 0, 0,
                0.0000, -30.9689, 0.0000, 0, 1.0000, 0,
               -0.0000, -0.0000, -30.9689, 0, 0, 1.0000;
        B_kf << 0, 0.5766, -0.0000, 0.0000,
                0, -0.0000, 0.5766, -0.0000,
                0, 0.0000, 0.0000, 0.5766,
                0, 30.9689, -0.0000, 0.0000,
                0, -0.0000, 30.9689, -0.0000,
                0.0042, 0.0000, 0.0000, 30.9689;

        sub = n.subscribe("/natnet_ros/hopper_outdoor/pose", 200, &OTWrapper::chatterCallback, this);

        quat_t quat_opti = quat_t(optiState_->q_w, optiState_->q_x, optiState_->q_y, optiState_->q_z);
        while (quat_opti.norm() < 0.99)
        {
            ros::spinOnce();
            quat_opti = quat_t(optiState_->q_w, optiState_->q_x, optiState_->q_y, optiState_->q_z);
        };
        std::cout << "Optitrack Connected" << std::endl;
        std::thread rosThreadSpin(&OTWrapper::rosThread, this);
        rosThreadSpin.detach();
    }

    void rosThread() {
        while(ros::ok()) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "exiting optitrack" << std::endl;
    }

    // void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    // {
    //     static std::chrono::high_resolution_clock::time_point t1;
    //     t1 = std::chrono::high_resolution_clock::now();
    //     static std::chrono::seconds oneSecond(1);
    //     static std::chrono::high_resolution_clock::time_point last_t_state_log = t1 - oneSecond;
    //     static bool init = false;
    //     static scalar_t dt;
    //     static vector_3t state_init(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    //     static vector_3t current_vel(0, 0, 0);
    //     static vector_3t filtered_current_vel(0, 0, 0);
    //     static bool first_contact = false;

    //     static vector_3t previous_vel(0, 0, 0);
    //     static vector_3t last_state(0., 0., msg->pose.position.z);
    //     static vector_6t est_state(6);
    //     if (!init)
    //     {
    //         est_state.setZero();
    //         est_state.segment(0, 3) << last_state;
    //         init = true;
    //     }

    //     static const scalar_t g = 9.81;

    //     optiState_->x = msg->pose.position.x;
    //     optiState_->y = msg->pose.position.y;
    //     optiState_->z = msg->pose.position.z;
    //     optiState_->q_w = msg->pose.orientation.w;
    //     optiState_->q_x = msg->pose.orientation.x;
    //     optiState_->q_y = msg->pose.orientation.y;
    //     optiState_->q_z = msg->pose.orientation.z;

    //     dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - last_t_state_log).count() * 1e-9;
    //     if (dt < 1e-3)
    //         return;

    //     current_vel << (optiState_->x - last_state(0)) / dt, (optiState_->y - last_state(1)) / dt, (optiState_->z - last_state(2)) / dt;
    //     filtered_current_vel << alpha * current_vel + (1 - alpha) * previous_vel;
    //     last_state << optiState_->x, optiState_->y, optiState_->z;

    //     if (contact)
    //         first_contact = true;

    //     // Kalman Filter
    //     vector_4t input(-g, optiState_->x, optiState_->y, optiState_->z);
    //     est_state << A_kf * est_state + B_kf * input;
    //     if (!first_contact || contact)
    //     {
    //         est_state(2) = optiState_->z;
    //         est_state(5) = filtered_current_vel(2);
    //     }

    //     optiState_->x = est_state(0);
    //     optiState_->y = est_state(1);
    //     optiState_->z = est_state(2);
    //     optiState_->x_dot = est_state(3);
    //     optiState_->y_dot = est_state(4);
    //     optiState_->z_dot = est_state(5);

    //     previous_vel << current_vel;
    //     last_t_state_log = t1;
    //     // optitrack_updated = true;
    // }

    void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        static std::chrono::high_resolution_clock::time_point t1;
        t1 = std::chrono::high_resolution_clock::now();
        static std::chrono::seconds oneSecond(1);
        static std::chrono::high_resolution_clock::time_point last_t_state_log = t1 - oneSecond;
        static scalar_t dt;
        static vector_3t current_vel(0, 0, 0);

        static vector_3t last_state(0., 0., msg->pose.position.z);

        optiState_->x = msg->pose.position.x;
        optiState_->y = msg->pose.position.y;
        optiState_->z = msg->pose.position.z;
        optiState_->q_w = msg->pose.orientation.w;
        optiState_->q_x = msg->pose.orientation.x;
        optiState_->q_y = msg->pose.orientation.y;
        optiState_->q_z = msg->pose.orientation.z;

        dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - last_t_state_log).count() * 1e-9;
        if (dt < 1e-3)
            return;

        current_vel << (optiState_->x - last_state(0)) / dt, (optiState_->y - last_state(1)) / dt, (optiState_->z - last_state(2)) / dt;
        last_state << optiState_->x, optiState_->y, optiState_->z;


        optiState_->x_dot = current_vel(0);
        optiState_->y_dot = current_vel(1);
        optiState_->z_dot = current_vel(2);

        last_t_state_log = t1;
        // optitrack_updated = true;
    }
};

std::unique_ptr<OTInterface> createOTInstance(std::shared_ptr<EstimatedState> optiState)
{
    return std::make_unique<OTWrapper>(optiState);
}