#include "../../inc/optitrack/ot_interface.h"

class OTWrapper : public OTInterface
{
public:
    OTWrapper() {
        // ROS stuff
        ros::init(argc, argv, "listener");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/vrpn_client_node/hopper/pose", 200, chatterCallback);

        quat_t quat_opti = quat_t(OptiState.q_w, OptiState.q_x, OptiState.q_y, OptiState.q_z);
        while (quat_opti.norm() < 0.99)
        {
            ros::spinOnce();
            quat_opti = quat_t(OptiState.q_w, OptiState.q_x, OptiState.q_y, OptiState.q_z);
        };
        std::cout << "Optitrack Connected" << std::endl;
    }

    void getState(vector_t &state) override
    {
    }

    void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        static std::chrono::high_resolution_clock::time_point t1;
        t1 = std::chrono::high_resolution_clock::now();
        static std::chrono::seconds oneSecond(1);
        static std::chrono::high_resolution_clock::time_point last_t_state_log = t1 - oneSecond;
        static bool init = false;
        static scalar_t dt;
        static vector_3t state_init(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z + p.frameOffset + p.markerOffset);
        static vector_3t current_vel(0, 0, 0);
        static vector_3t filtered_current_vel(0, 0, 0);
        static bool first_contact = false;

        static vector_3t previous_vel(0, 0, 0);
        static vector_3t last_state(0., 0., msg->pose.position.z + p.frameOffset + p.markerOffset);
        static vector_6t est_state(6);
        if (!init)
        {
            est_state.setZero();
            est_state.segment(0, 3) << last_state;
            init = true;
        }

        static const scalar_t g = 9.81;

        OptiState.x = msg->pose.position.x;
        OptiState.y = msg->pose.position.y;
        OptiState.z = msg->pose.position.z + p.frameOffset + p.markerOffset;
        OptiState.q_w = msg->pose.orientation.w;
        OptiState.q_x = msg->pose.orientation.x;
        OptiState.q_y = msg->pose.orientation.y;
        OptiState.q_z = msg->pose.orientation.z;

        dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - last_t_state_log).count() * 1e-9;
        if (dt < 1e-3)
            return;

        current_vel << (OptiState.x - last_state(0)) / dt, (OptiState.y - last_state(1)) / dt, (OptiState.z - last_state(2)) / dt;
        filtered_current_vel << alpha * current_vel + (1 - alpha) * previous_vel;
        last_state << OptiState.x, OptiState.y, OptiState.z;

        if (contact)
            first_contact = true;

        // Kalman Filter
        vector_4t input(-g, OptiState.x, OptiState.y, OptiState.z);
        est_state << A_kf * est_state + B_kf * input;
        if (!first_contact || contact)
        {
            est_state(2) = OptiState.z;
            est_state(5) = filtered_current_vel(2);
        }

        OptiState.x = est_state(0);
        OptiState.y = est_state(1);
        OptiState.z = est_state(2);
        OptiState.x_dot = est_state(3);
        OptiState.y_dot = est_state(4);
        OptiState.z_dot = est_state(5);

        previous_vel << current_vel;
        last_t_state_log = t1;
        // optitrack_updated = true;
    }
};

std::unique_ptr<OTInterface> createOTInstance()
{
    return std::make_unique<OTWrapper>();
}