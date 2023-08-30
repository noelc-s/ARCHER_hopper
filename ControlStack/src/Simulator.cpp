#include "Simulator.h"

class GilManager
{
public:
   GilManager()
   {
      mThreadState = PyEval_SaveThread();
   }

   ~GilManager()
   {
      if (mThreadState)
         PyEval_RestoreThread(mThreadState);
   }

   GilManager(const GilManager&) = delete;
   GilManager& operator=(const GilManager&) = delete;
private:
   PyThreadState* mThreadState;
};

//////////////////////////////////////////////////////////////////////////////////////

const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
char path[] = "../rsc/";
char xmlfile[] = "hopper.xml";
mjModel *m = NULL;                  // MuJoCo model
mjData *d = NULL;                   // MuJoCo data
mjContact *c = NULL;                // MuJoCo contact
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
mjfSensor sens;
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
mjtNum position_history = 0;
mjtNum previous_time = 0;
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

///////////////////////////////////////////////////////////////////////////////////////
////////////////// All of this first stuff is just Mujoco example code ////////////////
///////////////////////////////////////////////////////////////////////////////////////

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


///////////////////////////////////////////////////////////////////////////////////////////

// setup socket to receive intial condition from Controller
void setupSocket_receiveIC(scalar_t *init_conds, int n) {

    #define PORT_ 8081
    int client_IC;
    int status;
    struct sockaddr_in address_receiver;

    scalar_t init_cond[n];

    if ((client_IC = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
    }

    address_receiver.sin_family = AF_INET;
    address_receiver.sin_port = htons(PORT_);

    if (inet_pton(AF_INET, "127.0.0.3", &address_receiver.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
    }

    if ((status = connect(client_IC, (struct sockaddr *)&address_receiver, sizeof(address_receiver))) < 0) {
        printf("\nConnection Failed: socket IC \n");
    }

    read(client_IC, &init_cond, sizeof(init_cond));

    // std::cout << "Simulator Initial Conditions: \n";
    for (int i=0; i<n; i++) {
        init_conds[i] = init_cond[i];
        // std::cout << init_conds[i] << std::endl;
    }

    close(client_IC);
}

///////////////////////////////////////////////////////////////////////////////////////////

Simulator::Simulator() {
    // can do something for the sim here
}

// main function
void Simulator::run() {

    GilManager g; ///

    // activate software
    mj_activate("mjkey.txt");

    char xmlpath[100] = {};
    char datapath[100] = {};

    strcat(xmlpath, path);
    strcat(xmlpath, xmlfile);

    strcat(datapath, path);


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
     m = mj_loadXML(xmlpath, 0, error, 1000);
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

    // Set the initial condition [pos, orientation, vel, angular rate]
    scalar_t init_conds[12];
    setupSocket_receiveIC(init_conds, 12);

    d->qpos[0] = init_conds[0];
    d->qpos[1] = init_conds[1];
    d->qpos[2] = init_conds[2];
    d->qpos[3] = init_conds[3];
    d->qpos[4] = init_conds[4];
    d->qpos[5] = init_conds[5];

    d->qvel[0] = init_conds[6];
    d->qvel[1] = init_conds[7];
    d->qvel[2] = init_conds[8];
    d->qvel[3] = init_conds[9];
    d->qvel[4] = init_conds[10];
    d->qvel[5] = init_conds[11];

    // Read the gain matrix from the yaml file
    YAML::Node config = YAML::LoadFile("../config/gains.yaml");

    scalar_t pauseBeforeStart = config["Simulator"]["pauseBeforeStart"].as<scalar_t>();
    scalar_t speed = config["Simulator"]["speed"].as<scalar_t>();
    std::vector<scalar_t> pert_force_x = config["Simulator"]["pert_force_x"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> pert_force_y = config["Simulator"]["pert_force_y"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> pert_start = config["Simulator"]["pert_start"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> pert_end = config["Simulator"]["pert_end"].as<std::vector<scalar_t>>();
    scalar_t simend = config["Simulator"]["simEnd"].as<scalar_t>();

    // Setup the socket to communicate between the simulator and the controller
    int *new_socket = new int;
    int valread;
    struct sockaddr_in serv_addr;
    std::unique_ptr<uint16_t> port;
    // if (argc < 2) {
      port.reset(new uint16_t(8080));
    // } else {
    //   port.reset(new uint16_t(static_cast<uint16_t>(std::stoul(argv[1]))));
    // }

    // [receive - RX] Torques and horizon states: TODO: Fill in
    scalar_t RX_torques[26] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    // [to send - TX] States: time[1], pos[3], quat[4], vel[3], omega[3], contact[1], leg (pos,vel)[2], flywheel speed [3]
    scalar_t TX_state[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    if ((*new_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        // return -1;
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(*port);
    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        // return -1;
    }
    if (connect(*new_socket, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        printf("\nConnection Failed \n");
        // return -1;
    }

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

    scalar_t sim_flag = 1;

    // use the first while condition if you want to simulate for a period.
    while (!glfwWindowShouldClose(window)) {

      if (sim_flag < -0.5) {
	d->time = 0;

        d->qpos[0] = RX_torques[0];
        d->qpos[1] = RX_torques[1];
        d->qpos[2] = RX_torques[2];
        d->qpos[4] = RX_torques[3];
        d->qpos[5] = RX_torques[4];
        d->qpos[6] = RX_torques[5];
        d->qvel[0] = RX_torques[6];
        d->qvel[1] = RX_torques[7];
        d->qvel[2] = RX_torques[8];
        d->qvel[3] = RX_torques[9];
        d->qvel[4] = RX_torques[10];
        d->qvel[5] = RX_torques[11];

       }

        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
	
        while (d->time - simstart < speed / 60.0) {
            //use this for sending actual states
            mjtNum *quat;
            mjtNum *pos;
            quat = d->xquat;
            pos = d->xpos;
	    d->xfrc_applied[6] = 0;
	    d->xfrc_applied[7] = 0;
	    for (int i = 0; i < pert_force_x.size(); i++) {
		if (iter>pert_start[i] && iter < pert_end[i]) {
		  d->xfrc_applied[6] += pert_force_x[i];
		  d->xfrc_applied[7] += pert_force_y[i];
		}
	    }

            // Pack the state message:
            // time[1], pos[3], quat[4], vel[3], omega[3], contact[1], leg (pos,vel)[2], flywheel speed [3]
            int ind = 0;
            TX_state[0] = d->time;
            ind++;
            for (int i = 0; i < 3; i++) {
                TX_state[ind] = pos[i + 3];
                ind++;
            }
            for (int i = 0; i < 4; i++) {
                TX_state[ind] = quat[i + 4];
                ind++;
            }
            for (int i = 0; i < 6; i++) {
                TX_state[ind] = d->qvel[i];
                ind++;
            }

	    // Threshold for registering contact
            scalar_t contact_threshold = -0.003;
            TX_state[ind] = (c[0].dist < contact_threshold);
            ind++;

            TX_state[ind] = d->qpos[10];
            ind++;
            TX_state[ind] = d->qvel[9];
            ind++;
            TX_state[ind] = d->qvel[6];
            ind++;
            TX_state[ind] = d->qvel[7];
            ind++;
            TX_state[ind] = d->qvel[8];
            ind++;

            //send current states to the controller
	    do {
              send(*new_socket, &TX_state, sizeof(TX_state), 0);
              read(*new_socket, &RX_torques, sizeof(RX_torques));

              //override the communication based on the received toruqe comands from ctrl
              d->ctrl = RX_torques;

	      d->xfrc_applied[6] = RX_torques[23];
	      d->xfrc_applied[7] = RX_torques[24];

	      sim_flag = RX_torques[25];
	    } while (sim_flag < 0.1 && sim_flag > -0.1);

            // Take integrator step
            mj_step(m, d);
	    iter++;
        } 

	///////////// Set the states of the red and yellow dots to what the MPC predicts ///////////////
	static int body_offset = 11;
	static int vel_offset = 10;
	d->qpos[body_offset+0] = RX_torques[4];
 	d->qpos[body_offset+1] = RX_torques[5];
	d->qpos[body_offset+2] = RX_torques[6];
	d->qvel[vel_offset+0] = 0;
	d->qvel[vel_offset+1] = 0;
	d->qvel[vel_offset+2] = 0;
	d->qpos[body_offset+3] = RX_torques[7];
	d->qpos[body_offset+4] = RX_torques[8];
	d->qpos[body_offset+5] = RX_torques[9];
	d->qpos[body_offset+6] = RX_torques[10];

	d->qpos[22] = RX_torques[11];
	d->qpos[23] = RX_torques[12];
	d->qpos[24] = RX_torques[13];
	d->qpos[25] = RX_torques[14];
	d->qpos[26] = RX_torques[15];
	d->qpos[27] = RX_torques[16];
	d->qpos[28] = RX_torques[17];
	d->qpos[29] = RX_torques[18];
	d->qpos[30] = RX_torques[19];
	d->qpos[31] = RX_torques[20];
	d->qpos[32] = RX_torques[21];
	d->qpos[33] = RX_torques[22];


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
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
}
