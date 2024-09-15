#include <stdlib.h>
#include <iostream>
#include <stdbool.h> //for bool

#include "mujoco.h"
#include "GLFW/glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "../inc/Types.h"
#include "yaml-cpp/yaml.h"

#define PORT 8080
#define MAXLINE 1000
#define mjUSEDOUBLE

using namespace Hopper_t;
using namespace Eigen;

IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

///////////////////////////////////////////////////////////////////////////////////////
////////////////// All of this first stuff is just Mujoco example code ////////////////
///////////////////////////////////////////////////////////////////////////////////////

// simulation end time
char path[] = "../rsc/";
char xmlfile[] = "hopper.xml";

// MuJoCo data structures
mjModel *m = NULL;   // MuJoCo model
mjData *d = NULL;    // MuJoCo data
mjContact *c = NULL; // MuJoCo contact
mjvCamera cam;       // abstract camera
mjvOption opt;       // visualization options
mjvScene scn;        // abstract scene
mjrContext con;      // custom GPU context
mjfSensor sens;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void set_torque_control(const mjModel *m, int actuator_no, int flag)
{
    if (flag == 0)
        m->actuator_gainprm[10 * actuator_no + 0] = 0;
    else
        m->actuator_gainprm[10 * actuator_no + 0] = 1;
}
/******************************/

/******************************/
void set_position_servo(const mjModel *m, int actuator_no, double kp)
{
    m->actuator_gainprm[10 * actuator_no + 0] = kp;
    m->actuator_biasprm[10 * actuator_no + 1] = -kp;
}
/******************************/

/******************************/
void set_velocity_servo(const mjModel *m, int actuator_no, double kv)
{
    m->actuator_gainprm[10 * actuator_no + 0] = kv;
    m->actuator_biasprm[10 * actuator_no + 2] = -kv;
}

void init_controller(const mjModel *m, mjData *d)
{
}

//**********************COMMUNICATION PROTOCOLS**************
void mycontroller(const mjModel *m, mjData *d)
{
}

// Helper function to compute distance between two points
double distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// main function
int main(int argc, const char **argv)
{

    // activate software
    // mj_activate("mjkey.txt");

    char xmlpath[100] = {};
    char datapath[100] = {};

    strcat(xmlpath, path);
    strcat(xmlpath, xmlfile);

    strcat(datapath, path);

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if (argc < 2)
        m = mj_loadXML(xmlpath, 0, error, 1000);

    else if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    // GLFWwindow *window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    GLFWwindow *window = glfwCreateWindow(3840, 2160, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);              // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150); // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {89.608063, -11.588379, 2, 0.000000, 0.000000,
                         0.500000}; // view the left side (for ll, lh, left_side)
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    // install control callback
    mjcb_control = mycontroller;

    init_controller(m, d);

    /* initialize random seed: */
    srand(time(NULL));

    //////////////////////////////////////////////////////////////////////////////////////
    ////////////// This is where custom code deviates from Mujoco examples ///////////////
    //////////////////////////////////////////////////////////////////////////////////////

    // Setup the socket to communicate between the simulator and the controller
    int *new_socket = new int;
    int valread;
    struct sockaddr_in serv_addr;

    if ((*new_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
    if (connect(*new_socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }

    // Read the gain matrix from the yaml file
    YAML::Node config = YAML::LoadFile("../config/gains.yaml");
    std::vector<scalar_t> p0 = config["Simulator"]["p0"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> v0 = config["Simulator"]["v0"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> rpy0 = config["Simulator"]["rpy0"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> w0 = config["Simulator"]["w0"].as<std::vector<scalar_t>>();

    config = YAML::LoadFile("../config/planner_params.yaml");
    int N = config["MPC"]["N"].as<int>();
    int max_graph_sol_length = config["Planner"]["max_graph_sol_length"].as<int>();
    int max_num_obstacles = config["Planner"]["max_num_obstacles"].as<int>();

    // [receive - RX] Torques and horizon states: TODO: Fill in
    float RX_torques[11 + 2 + 8 * max_num_obstacles + 2 * N + 2 * max_graph_sol_length] = {0};
    // [to send - TX] States: time[1], pos[3], quat[4], vel[3], omega[3], contact[1], leg (pos,vel)[2], flywheel speed [3]
    scalar_t TX_state[1] = {1.0};

    // Set the initial condition [pos, orientation, vel, angular rate]
    d->qpos[0] = p0[0];
    d->qpos[1] = p0[1];
    d->qpos[2] = p0[2];
    d->qpos[4] = rpy0[0];
    d->qpos[5] = rpy0[1];
    d->qpos[6] = rpy0[2];
    d->qvel[0] = v0[0];
    d->qvel[1] = v0[1];
    d->qvel[2] = v0[2];
    d->qvel[3] = w0[0];
    d->qvel[4] = w0[1];
    d->qvel[5] = w0[2];

    //////////////////////////////// Standard Mujoco Setup //////////////////////////////////////
    // get framebuffer viewport
    mj_step(m, d); // populate state info
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    // update scene and render
    // cam.lookat[0] = d->qpos[0];
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
    c = d->contact;
    // Instantiate perturbation object
    mjvPerturb *pert = new mjvPerturb();
    pert->select = 1;
    pert->active = 1;
    opt.flags[mjVIS_PERTFORCE] = 1;
    int iter = 0;

    // use the first while condition if you want to simulate for a period.
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;

        while (d->time - simstart < 1 / 60.0)
        {
            // use this for sending actual states
            mjtNum *quat;
            mjtNum *pos;
            quat = d->xquat;
            pos = d->xpos;

            // send current states to the controller
            send(*new_socket, &TX_state, sizeof(TX_state), 0);
            read(*new_socket, &RX_torques, sizeof(RX_torques));

            // Take integrator step
            mj_step(m, d);
            iter++;
        }

        ///////////// Set the states of the red and yellow dots to what the MPC predicts ///////////////
        static int body_offset = 11;
        static int vel_offset = 10;

        for (int i = 0; i < 3; i++) {
            d->qpos[i] = RX_torques[i];    
            d->qvel[i] = 0;    
        }
        d->qpos[3] = RX_torques[6];
        d->qpos[4] = RX_torques[3];
        d->qpos[5] = RX_torques[4];
        d->qpos[6] = RX_torques[5];
        d->qpos[7] = 0;
        d->qpos[8] = 0;
        d->qpos[9] = 0;
        d->qpos[10] = 0;    
        
        d->qpos[2] += 0.1;

        d->qpos[body_offset + 0] = RX_torques[11];
        d->qpos[body_offset + 1] = RX_torques[12];
        // d->qpos[body_offset+2] = RX_torques[13];
        // d->qpos[body_offset+3] = RX_torques[14];
        // d->qpos[body_offset+2] = RX_torques[13];
        // d->qpos[25] = RX_torques[14];
        // d->qpos[26] = RX_torques[15];
        // d->qpos[27] = RX_torques[16];
        // d->qpos[28] = RX_torques[17];
        // d->qpos[29] = RX_torques[18];
        // d->qpos[30] = RX_torques[19];
        // d->qpos[31] = RX_torques[20];
        // d->qpos[32] = RX_torques[21];
        // d->qpos[33] = RX_torques[22];

        ////////////////////////////////// Standard Mujoco stuff below this //////////////////////////////
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        cam.lookat[0] = d->qpos[0];
        cam.lookat[1] = d->qpos[1];
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

        static double zero3[3] = {0};
        static double zero9[9] = {0};
        float color[4] = {0.3020,0.3490,0.8392, 1.0};
        for (int i = 0; i < N - 1; i++)
        {
            mjv_initGeom(&scn.geoms[scn.ngeom], mjGEOM_CAPSULE, zero3, zero3, zero9, color);
            mjv_makeConnector(
                &scn.geoms[scn.ngeom], mjGEOM_CAPSULE, .01,
                RX_torques[13 + 8 * max_num_obstacles + 2 * i], RX_torques[13 + 8 * max_num_obstacles + 2 * i + 1], 0, RX_torques[13 + 8 * max_num_obstacles + 2 + 2 * i], RX_torques[13 + 8 * max_num_obstacles + 2 + 2 * i + 1], 0);
            scn.ngeom += 1;
        }

        float graphcolor[4] = {0.1725,0.8706,0.8588, 1.0};
        for (int i = 0; i < max_graph_sol_length-1; i++) {
            mjv_initGeom(&scn.geoms[scn.ngeom], mjGEOM_CAPSULE, zero3, zero3, zero9, graphcolor);
            mjv_makeConnector(
                    &scn.geoms[scn.ngeom], mjGEOM_CAPSULE, .01, 
                    RX_torques[13+8*max_num_obstacles+2*N+2*i],RX_torques[13+8*max_num_obstacles+2*N+2*i+1],0,RX_torques[13+8*max_num_obstacles+2*N+2+2*i],RX_torques[13+8*max_num_obstacles+2*N+2+2*i+1],0);
            scn.ngeom += 1;
        }

        // d->qpos[body_offset+2] = RX_torques[13:20];
        for (int o = 0; o < max_num_obstacles - 1; o++)
        {
            float corner[8];
            memcpy(corner, RX_torques + 13 + 8 * o, 8 * sizeof(float));

            float box_color[4] = {0.8392,0.3490,0.3490, 1.0};
            // Compute the centroid (average of all corners)
            double pos[3] = {
                (corner[0] + corner[2] + corner[4] + corner[6]) / 4.0, // x-coordinate
                (corner[1] + corner[3] + corner[5] + corner[7]) / 4.0, // y-coordinate
                0.0                                                    // z-coordinate
            };

            // Compute width and height
            // Find all distances and select maximum for width and height
            double d1 = distance(corner[0], corner[1], corner[2], corner[3]);
            double d2 = distance(corner[2], corner[3], corner[4], corner[5]);
            double d3 = distance(corner[4], corner[5], corner[6], corner[7]);
            double d4 = distance(corner[6], corner[7], corner[0], corner[1]);

            std::vector<double> e = {d1, d2, d3, d4};
            double angle;
            double width = std::max({d1, d2, d3, d4});
            double height = std::min({d1, d2, d3, d4});
            int index = std::distance(e.begin(), std::max_element(e.begin(), e.end()));
            switch (index)
            {
            case 0:
                angle = std::atan2(corner[3] - corner[1], corner[2] - corner[0]);
                break;
            case 1:
                angle = std::atan2(corner[5] - corner[3], corner[4] - corner[2]);
                break;
            case 2:
                angle = std::atan2(corner[7] - corner[5], corner[6] - corner[4]);
                break;
            case 3:
                angle = std::atan2(corner[1] - corner[7], corner[0] - corner[6]);
                break;
            }

            // angle = std::atan2(corner[3] - corner[1], corner[2] - corner[0]);
            if (height > width)
                std::swap(width, height);

            // Rotation matrix
            double rot[9] = {
                std::cos(angle), -std::sin(angle), 0,
                std::sin(angle), std::cos(angle), 0,
                0, 0, 1};
            double size[3] = {width / 2.0, height / 2.0, 0.03}; // Depth, adjust as needed
            // std::cout << size[0] << ", " <<  size[1] << ", " <<  size[2] << std::endl;
            // std::cout << pos[0] << ", " <<  pos[1] << ", " <<  pos[2] << std::endl;
            // std::cout << rot[0] << ", " <<  rot[1] << ", " <<  rot[2] << ", " <<  rot[3] << std::endl;
            mjv_initGeom(&scn.geoms[scn.ngeom], mjGEOM_BOX, size, pos, rot, box_color);
            scn.ngeom += 1;
        }

        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    // mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}
