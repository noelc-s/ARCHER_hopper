#include<stdlib.h>
#include <iostream>
#include<stdbool.h> //for bool

#include "mujoco.h"
#include "GLFW/glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "../inc/UnitTest.h"
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include<netinet/in.h>
#include<unistd.h>
#include <fstream>

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

//simulation end time
char path[] = "../rsc/";
char xmlfile[] = "hopper.xml";

// MuJoCo data structures
mjModel *m = NULL;                  // MuJoCo model
mjData *d = NULL;                   // MuJoCo data
mjContact *c = NULL;                // MuJoCo contact
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
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
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos) {
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
void scroll(GLFWwindow *window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}


void set_torque_control(const mjModel *m, int actuator_no, int flag) {
    if (flag == 0)
        m->actuator_gainprm[10 * actuator_no + 0] = 0;
    else
        m->actuator_gainprm[10 * actuator_no + 0] = 1;
}
/******************************/


/******************************/
void set_position_servo(const mjModel *m, int actuator_no, double kp) {
    m->actuator_gainprm[10 * actuator_no + 0] = kp;
    m->actuator_biasprm[10 * actuator_no + 1] = -kp;
}
/******************************/

/******************************/
void set_velocity_servo(const mjModel *m, int actuator_no, double kv) {
    m->actuator_gainprm[10 * actuator_no + 0] = kv;
    m->actuator_biasprm[10 * actuator_no + 2] = -kv;
}


void init_controller(const mjModel *m, mjData *d) {

}

//**********************COMMUNICATION PROTOCOLS**************
void mycontroller(const mjModel *m, mjData *d) {

}

template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    std::getline(indata, line);
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

// main function
int main(int argc, const char **argv) {

    std::string csvFile = "/home/noelcs/repos/ZeroDynamicsPolicies/HJBSampling/tests/hopper_gpu.csv";
    MatrixXd csvData = load_csv<MatrixXd>(csvFile);

    // activate software
    //mj_activate("mjkey.txt");

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
    GLFWwindow *window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {89.608063, -11.588379, 2, 0.000000, 0.000000,
                         0.500000}; //view the left side (for ll, lh, left_side)
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

    // Read the gain matrix from the yaml file
    scalar_t speed = 1.0;
    scalar_t pauseBeforeStart = 1.0;

    //////////////////////////////// Standard Mujoco Setup //////////////////////////////////////
    // get framebuffer viewport
    mj_step(m, d); // populate state info
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    // update scene and render
    //cam.lookat[0] = d->qpos[0];
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
    sleep(pauseBeforeStart);
    c = d->contact;
    // Instantiate perturbation object
    mjvPerturb* pert = new mjvPerturb();
    pert->select = 1;
    pert->active = 1;
    opt.flags[mjVIS_PERTFORCE] = 1;
    int iter = 0;
    
    

    // use the first while condition if you want to simulate for a period.
    while (!glfwWindowShouldClose(window)) {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        static int row = 0;
	
        while (d->time - simstart < speed / 60.0) {
            //use this for sending actual states
            mjtNum *quat;
            mjtNum *pos;
            quat = d->xquat;
            pos = d->xpos;

            // Take integrator step
            mj_step(m, d);
            iter++;
        } 
        
        std::cout << csvData(row,0) << "," <<
        		csvData(row,1) << "," <<
        		csvData(row,2) << "," <<
        		csvData(row,3) << std::endl;
        
        Quaternionf q;
	q = AngleAxisf(csvData(row,1), Vector3f::UnitX())
	    * AngleAxisf(csvData(row,2), Vector3f::UnitY())
	    * AngleAxisf(csvData(row,3), Vector3f::UnitZ());
        

	///////////// Set the states of the red and yellow dots to what the MPC predicts ///////////////
	    // Set the initial condition [pos, orientation, vel, angular rate]
	    d->qpos[0] = 0;
	    d->qpos[1] = 0;
	    d->qpos[2] = 0.5;
    	    d->qpos[3] = q.w();
	    d->qpos[4] = q.x();
	    d->qpos[5] = q.y();
	    d->qpos[6] = q.z();
	    d->qvel[0] = 0;
	    d->qvel[1] = 0;
	    d->qvel[2] = 0;
	    d->qvel[3] = 0;
	    d->qvel[4] = 0;
	    d->qvel[5] = 0;
	
	while(d->time > csvData(row,0))
		row = row+1;

	////////////////////////////////// Standard Mujoco stuff below this //////////////////////////////
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        cam.lookat[0] = d->qpos[0];
        cam.lookat[1] = d->qpos[1];
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
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
    //mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}
