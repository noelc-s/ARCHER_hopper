#ifndef Flywheel_h
#define Flywheel_h

#include "Arduino.h"
#include "Archer_Config.h"
#include "ArduinoEigen.h"
#include "TripENC.h"
#include "ELMO_CANt4.h"
#include <SD.h>

using namespace Eigen;

using vector_3t = Eigen::Matrix<float, 3, 1>;
using vector_4t = Eigen::Matrix<float, 4, 1>;
using matrix_t = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using matrix_3t = Eigen::Matrix<float, 3, 3>;
using quat_t = Eigen::Quaternion<float>;


//////////////////////////////////////////////////////////////////////////
////////////////// TODO: VERIFY THESE MAGIC NUMBERS //////////////////////
///////////////////// they are really important! /////////////////////////
//////////////////////////////////////////////////////////////////////////

//use for the counication with the wheel motors
//convert torques to amps with torque / 0.083 = currents [A]
//for a range of -1.6Nm to 1.6 Nm
float torque_to_current = 1.0 / 0.083;
#define MAX_CURRENT 12   //  15
#define MIN_CURRENT -12  // -15

//////////////////////////////////////////////////////////////////////////


#define TIMEOUT_INTERVAL 100  // ms to timeout
#define TEST_ / TEENSY

// State variables
float x_d[7];
float state[13]; //this is only for the first debugging run
char oneAdded[6];
quat_t quat_a, quat_d;
vector_3t omega_d, tau_ff;
float state_d[10];

// Elmo variables
TripENC tENC(trip_CS1, trip_CS2, trip_CS3);
Archer::ELMO_CANt4 elmo;

// SD CARD Variables
const byte NUMBER_OF_RECORDS = 6;
char parameterArray[NUMBER_OF_RECORDS][30];
char aRecord[30];
byte recordNum;
byte charNum;

// SD card location of gain config
String gain_config = "gain_config.txt";
File gainFile;

// For rotation about x and y (this is what Noel and I did the other day)
// The proportional and derivative gain should both be the same since the dynamics are coupled
double kp_y;
double kp_rp;
double kd_y;
double kd_rp;

// We inserted this bias because the true CG is a little of center
double r_offset;
double p_offset;


//=================================================================

/*
 * Fix yaw singularity
 * Add foot and WiFi back
 * Test MPC with longer horizon
 * 
 * Notes:
 * Yaw and roll pitch gain should not be different, coupled in Lie Algebra
 * 
 */

//use volatile if we need to use threading for our robot
volatile float dR = 0;
volatile float dP = 0;
volatile float dY = 0;
volatile float x1 = 0;
volatile float v1 = 0;
volatile float x2 = 0;
volatile float v2 = 0;
volatile float x3 = 0;
volatile float v3 = 0;
volatile float q0 = 1;
volatile float q1 = 0;
volatile float q2 = 0;
volatile float q3 = 0;

quat_t quat_init;
quat_t quat_init_inverse;
volatile bool initialized = false;
bool time_initialized = false;

//Archer::Koios *koios;

int nF;
bool rt = 0;

boolean newData = false;

volatile bool ESP_connected = false;

//===========SPEEDING UP BABY
char additional_read_buffer[3000];  //this values are out of nowhere
char additional_write_buffer[3000];


unsigned long last_ESP_message;
unsigned long current_ESP_message;
char receivedCharsESP[46];
float foot_state[3];

//=================SETUP=============
bool exit_state = false;
File data;
String dFile = "log.txt";

// Threads
Threads::Mutex state_mtx;
//Threads::Mutex foot_state_mtx;
Threads::Mutex serial_mtx;

void delayLoop(uint32_t T1, uint32_t L);
void BiaThread();
void ESPthread();
void imuThread();
matrix_3t cross(vector_3t q);
void getTorque(float* state, quat_t quat_d, vector_3t omega_d, vector_3t tau_ff, quat_t quat_a, vector_3t &torque);
void exitProgram();
void parseRecord(byte index);
void writeData(File data, uint32_t Tc1, quat_t quat_a, quat_t quat_d, vector_3t omega_a, vector_3t omega_d, float* foot_state, vector_3t current);

#endif
