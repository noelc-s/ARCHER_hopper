#pragma once

#include <Python.h>

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
#include<netinet/in.h>
#include<unistd.h>

#include "../inc/Types.h"
#include "yaml-cpp/yaml.h"

#define MAXLINE 1000
#define mjUSEDOUBLE

using namespace Hopper_t;
using namespace Eigen;

// enum SimulatorState {
//   RUNNING = 1,
//   STOPPED = 0,
//   RESET = -1
// };

class S {
public:
    virtual ~S() { }
    virtual void run() = 0;
};

void call_run_sim(S *s) {
    s->run();
}

class Simulator : public S {
    public:

        Simulator();
        // SimulatorState simulatorState_ = RUNNING;

        bool visualize_;
        void setVisualization(bool visualize); // set glfw

        void run() override; //explicitly override S::run() and use Simulator::run()
        void killSimulation();
    
    bool kill = false;

    private:
        void run_with_visualization();
        void run_without_visualization();

};

void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow *window, int button, int act, int mods);
void mouse_move(GLFWwindow *window, double xpos, double ypos);
void scroll(GLFWwindow *window, double xoffset, double yoffset);
void set_torque_control(const mjModel *m, int actuator_no, int flag);
void set_position_servo(const mjModel *m, int actuator_no, double kp);
void set_velocity_servo(const mjModel *m, int actuator_no, double kv);
void init_controller(const mjModel *m, mjData *d);
void mycontroller(const mjModel *m, mjData *d);

void setupSocket_receiveIC(scalar_t *init_conds, int n);

