#include <Archer_Config.h>
#include <TripENC.h>
#include <ELMO_CANt4.h>
#include <Koios.h>
#include <SPI.h>
//#include <SD.h>
#include <TeensyThreads.h>
#include <ArduinoEigen.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_/TEENSY

using namespace Archer;
using namespace Eigen;

//==================CONSTANTS
TripENC tENC(trip_CS1, trip_CS2, trip_CS3);
ELMO_CANt4 elmo;

float x_d[7];
#define torque_to_current 1.0/0.083

#define MAX_CURRENT 12  //  15
#define MIN_CURRENT -12 // -15

#define TIMEOUT_INTERVAL 100 // ms to timeout

using vector_3t = Eigen::Matrix<float, 3, 1>;
using vector_4t = Eigen::Matrix<float, 4, 1>;
using matrix_t = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using matrix_3t = Eigen::Matrix<float, 3, 3>;
using quat_t = Eigen::Quaternion<float>;

// For rotation about x and y (this is what Noel and I did the other day)
// The proportional and derivative gain should both be the same sine the dynamics are coupled
#define kp_y 100.0      
#define kp_rp 100.0
#define kd_y 2.0
#define kd_rp 2.0

// We insrted this bias because the true CG is a little of center
#define r_offset 0.005
#define p_offset 0.023

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

Koios *koios;

int nF;
bool rt = 0;

boolean newData = false;
//this is only for the first debugging run
float state[13];

//===========SPEEDING UP BABY
char additional_read_buffer[3000]; //this values are out of nowhere
char additional_write_buffer[3000];


unsigned long last_ESP_message;
unsigned long current_ESP_message;
char receivedCharsESP[46];
float foot_state[3];

//=================SETUP=============

bool exit_state = false;
File data;
String dFile = "log.txt";

void setup() {
  //====================WIFI==============
  delay(100); //give time to open and print
  Serial.begin(115200); //this is for the monitor
  Serial7.begin(115200); //baud rates must be the same
  while (!Serial7) {
    ;
  }
  delay(100);

  //start a diode to be sure all is working
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //  //================Koios=============
  koios = new Koios(tENC, elmo);
  koios->initKoios1(1);
  data = SD.open(dFile.c_str(),FILE_WRITE);

//#ifndef TEST_TEENSY
  // initKoios2
  delay(250);
  koios->STO(1);
  koios->waitSwitch(1); //manual switch on robot
  koios->setSigB(1);
  delay(250);
  rt = koios->motorsOn();
  delay(5000);
  koios->resetStates();
  koios->setLEDs("0100");
  koios->waitSwitch(0);
  koios->setSigB(0);
  koios-> setLogo('A');
  koios->flashG(2);
//#endif

  rt = threads.setSliceMicros(25);
//#ifndef TEST_TEENSY
  threads.addThread(imuThread);
  threads.addThread(BiaThread);
//#endif
  threads.addThread(ESPthread);
  delay(5000);
  foot_state[0] = 0;
  foot_state[1] = 0;
  foot_state[2] = 0;
//#ifndef TEST_TEENSY
  koios->setLEDs("0001");
  Serial7.clear();
  koios->setLogo('R');
  delay(5000);
//#endif
}

//============FUNCTIONS==========
void delayLoop(uint32_t T1, uint32_t L) {
  uint32_t T2 = micros();
  if ((T2 - T1) < L) {
    uint32_t a = L + T1 - T2;
    threads.delay_us(a);
  }
}

volatile bool ESP_connected = false;

Threads::Mutex state_mtx;
Threads::Mutex foot_state_mtx;
Threads::Mutex serial_mtx;

//ThreadWrap(Serial, SerialXtra);
//#define Serial ThreadClone(SerialXtra)
//ThreadWrap(B_PORT, SerialXtra2);
//#define B_PORT ThreadClone(SerialXtra2)
//ThreadWrap(IMU_PORT, SerialXtra3);
//#define IMU_PORT ThreadClone(SerialXtra3)

void BiaThread() {
  while (1) {
//    #ifdef TEST_TEENSY
//    { Threads::Scope scope(foot_state_mtx);
//    foot_state[0] = 3;
//    foot_state[1] = 4;
//    foot_state[2] = 5;
//    }
//    #else
    int index = 0;
    char receivedCharsBia[11];
//    {
//    Threads::Scope scope(serial_mtx);
    if (B_PORT.available() > 0) {
      while (index < 11) {
        { 
        if (B_PORT.available() > 0) {
          receivedCharsBia[index] = B_PORT.read();
          index++;
        }
        }
      }
    }
//    }
    char oneAddedBia[2];
    memcpy(oneAddedBia, receivedCharsBia + 9, 2 * sizeof(char));
    for (int i = 0; i < 2; i++) {
      for (int j = 1; j < 8; j++) {
        if (oneAddedBia[i] & (1 << (8 - j))) {
          receivedCharsBia[i * 7 + (j - 1)] = 0;
        }
      }
    }
    { Threads::Scope scope(foot_state_mtx);
    foot_state[0] = (float) receivedCharsBia[0];
    memcpy(foot_state+1, receivedCharsBia + 1, 2 * 4);
    }
//    #endif
    threads.delay_us(500);
  }
}

void ESPthread() {
  bool read_data = true;
  while (1) {
    if (initialized) {
      if (read_data) {
        char receivedCharsTeensy[13 * sizeof(float) + 8 + 1];
        { Threads::Scope scope(state_mtx);
          memcpy(receivedCharsTeensy, state, 52);
        }
  
        for (int i = 0; i < 8; i++) {
          byte oneAdded = 0b00000001;
          for (int j = 1; j < 8; j++) {
            if (receivedCharsTeensy[i * 7 + (j - 1)] == 0b00000000) {
              receivedCharsTeensy[i * 7 + (j - 1)] = 0b00000001;
              oneAdded += (1 << (8 - j));
            }
          }
          memcpy(&receivedCharsTeensy[52 + i], &oneAdded, 1);
        }
        receivedCharsTeensy[60] = 0b0;
  
        Serial7.print(receivedCharsTeensy);
        Serial7.flush();
        read_data = false;
      }

      int index = 0;
      if (Serial7.available() > 0) {
        while (index < 46) {
          if (Serial7.available() > 0) {
            receivedCharsESP[index] = Serial7.read();
            index++;
          }
          
        }
        if (!time_initialized) {
          last_ESP_message = millis();
          time_initialized = true;
        } else {
          last_ESP_message = millis();
        }
        ESP_connected = true;
        read_data = true;
        threads.delay_us(100);
      } else {
        threads.yield();
      }
      
    } else {
      threads.delay_us(2000);
    }
  }
}

void imuThread() {
  while (1) {
    if (IMU_PORT.available() > 0) {
      uint8_t byt;
      uint8_t b0;
      uint8_t b1;
      uint8_t b2;
      uint8_t b3;
//      { Threads::Scope scope(serial_mtx);
      byt = IMU_PORT.read();
//      }
      if (byt == 0xFA) {
//        threads.delay_us(350);
        delayMicroseconds(350);
//        { Threads::Scope scope(serial_mtx);
        for (int i = 0; i < 3; i++) {
          IMU_PORT.read();
        }
//        }
        //=========================================================
        union {
          long y;
          float z;
        } Q0_;
//        { Threads::Scope scope(serial_mtx);
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
//        }
        uint8_t bQ0[4] = {b0, b1, b2, b3};
        long x = (long)bQ0[3] << 24 | (long)bQ0[2] << 16 | bQ0[1] << 8 | bQ0[0];
        Q0_.y = x;
        { Threads::Scope scope(state_mtx);
        q0 = Q0_.z;
        }
        //=========================================================
        union {
          long y;
          float z;
        } Q1_;
//        { Threads::Scope scope(serial_mtx);
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
//        }
        uint8_t bQ1[4] = {b0, b1, b2, b3};
        x = (long)bQ1[3] << 24 | (long)bQ1[2] << 16 | bQ1[1] << 8 | bQ1[0];
        Q1_.y = x;
        { Threads::Scope scope(state_mtx);
        q1 = Q1_.z;
        }
        union {
          long y;
          float z;
        } Q2_;
//        { Threads::Scope scope(serial_mtx);
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
//        }
        uint8_t bQ2[4] = {b0, b1, b2, b3};
        x = (long)bQ2[3] << 24 | (long)bQ2[2] << 16 | bQ2[1] << 8 | bQ2[0];
        Q2_.y = x;
        { Threads::Scope scope(state_mtx);
        q2 = Q2_.z;
        }
        union {
          long y;
          float z;
        } Q3_;
//        { Threads::Scope scope(serial_mtx);
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
//        }
        uint8_t bQ3[4] = {b0, b1, b2, b3};
        x = (long)bQ3[3] << 24 | (long)bQ3[2] << 16 | bQ3[1] << 8 | bQ3[0];
        Q3_.y = x;
        { Threads::Scope scope(state_mtx);
        q3 = Q3_.z;
        }
        union {
          long y;
          float z;
        } RR;
//        { Threads::Scope scope(serial_mtx);
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
//        }
        uint8_t bR[4] = {b0, b1, b2, b3};
        x = (long)bR[3] << 24 | (long)bR[2] << 16 | bR[1] << 8 | bR[0];
        RR.y = x;
        { Threads::Scope scope(state_mtx);
        dR = RR.z;
        }
        union {
          long y;
          float z;
        } PP;
//        { Threads::Scope scope(serial_mtx);
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
//        }
        uint8_t bP[4] = {b0, b1, b2, b3};
        x = (long)bP[3] << 24 | (long)bP[2] << 16 | bP[1] << 8 | bP[0];
        PP.y = x;
        { Threads::Scope scope(state_mtx);
        dP = PP.z;
        }
        union {
          long y;
          float z;
        } YY;
//        { Threads::Scope scope(serial_mtx);
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
//        }
        uint8_t bY[4] = {b0, b1, b2, b3};
        x = (long)bY[3] << 24 | (long)bY[2] << 16 | bY[1] << 8 | bY[0];
        YY.y = x;
        { Threads::Scope scope(state_mtx);
        dY = YY.z;
        }
//        { Threads::Scope scope(serial_mtx);
        IMU_PORT.read();
        IMU_PORT.read();
//        }
        nF++;
      }
    }
    threads.delay_us(100);
  }
}

matrix_3t cross(vector_3t q) {
  matrix_3t c;
  c << 0, -q(2), q(1),
  q(2), 0, -q(0),
  -q(1), q(0), 0;
  return c;
}

char oneAdded[6];
quat_t quat_a;

void getTorque(float* state, quat_t quat_d, vector_3t omega_d, vector_3t tau_ff, quat_t quat_a, vector_3t &torque) {
  vector_3t delta_omega;
  vector_3t omega_a;
  vector_4t quat_d_vec;
  vector_4t quat_a_vec;

  quat_t quat_actuator = quat_t(0.8806, 0.3646, -0.2795, 0.1160);
  omega_a << state[3], state[4], state[5];

  quat_d_vec << quat_d.w(), quat_d.x(), quat_d.y(), quat_d.z();
  quat_a_vec << quat_a.w(), quat_a.x(), quat_a.y(), quat_a.z();

  vector_3t delta_quat;
  delta_quat << quat_a_vec[0] * quat_d_vec.segment(1, 3) - quat_d_vec[0] * quat_a_vec.segment(1, 3) -
             cross(quat_a_vec.segment(1, 3)) * quat_d_vec.segment(1, 3);
  matrix_3t Kp, Kd;
//        Serial.print(delta_quat[0]); Serial.print(", ");
//      Serial.print(delta_quat[1]); Serial.print(", ");
//      Serial.print(delta_quat[2]); Serial.print(",     ");
  Kp.setZero();
  Kd.setZero();
  Kp.diagonal() << kp_rp, kp_rp, kp_y;
  Kd.diagonal() << kd_rp, kd_rp, kd_y;

  vector_3t tau_fb;
  delta_omega = omega_a - omega_d;
  tau_fb = -quat_actuator.inverse()._transformVector(Kp * delta_quat) - quat_actuator.inverse()._transformVector(-Kd * delta_omega);

  torque << (tau_fb + tau_ff)*torque_to_current;
//
//  Serial.print(tau_fb[0]); Serial.print(", ");
//      Serial.print(tau_fb[1]); Serial.print(", ");
//      Serial.print(tau_fb[2]); Serial.print(",     ");
}

//==========================LOOP=============

void exitProgram() {
  elmo.cmdTC(0.0, IDX_K1);
  elmo.cmdTC(0.0, IDX_K2);
  elmo.cmdTC(0.0, IDX_K3);
  koios->motorsOff(0);
  koios->setSigB(1);
  data.close();
  threads.delay(100);
  koios->setLEDs("1000");
  koios->setLogo('R');
  while (1) {};
}

void loop() {
  static float Q0 = 0;
#ifdef TEST_TEENSY
  Q0 = 1;
#endif
  static float Q1 = 0;
  static float Q2 = 0;
  static float Q3 = 0;
  static float DY = 0;
  static float DP = 0;
  static float DR = 0;

  static int reset_cmd = 0;

  if (Serial.available() > 0) {
    while (Serial.available() > 0)
      Serial.read();
    reset_cmd = 1;
  };

  vector_4t state_tmp;

  while (!initialized) {
    rt = abs(q0) < 2 && abs(q1)<2 && abs(q2)<2 && abs(q3)<2 && abs(dY) < 1e5 && abs(dP) < 1e5 && abs(dR) < 1e5;
//    koios->checkFrame(q0, q1, q2, q3, dY, dP, dR);
    //based on imu upadate states
    if (rt == 1) {
      Q0 = q0;
      Q1 = q1;
      Q2 = q2;
      Q3 = q3;
      DY = dY;
      DP = dP;
      DR = dR;
    }
    koios->updateStates(x1, v1, x2, v2, x3, v3);
    // Add step to get leg length from Bia here over serial
    { Threads::Scope scope(state_mtx);
      Threads::Scope scope2(foot_state_mtx);
      state[0] = v1;
      state[1] = v3;
      state[2] = v2;
      state[3] = DR;
      state[4] = DP;
      state[5] = DY;
      state[6] = Q0;
      state[7] = Q1;
      state[8] = Q2;
      state[9] = Q3;
      state[10] = foot_state[0];
      state[11] = foot_state[1];
      state[12] = foot_state[2];
    }
    state_tmp << (float)state[9], (float)state[6], (float)state[7], (float)state[8];
    if (state_tmp.norm() > 0.95 && state_tmp.norm() < 1.05) {

      quat_a = quat_t((float)state[9], (float)state[6], (float)state[7], (float)state[8]); // assuming q_w is last.

           // roll (x-axis rotation)
      float sinr_cosp = 2 * (quat_a.w() * quat_a.x() + quat_a.y() * quat_a.z());
      float cosr_cosp = 1 - 2 * (quat_a.x() * quat_a.x() + quat_a.y() * quat_a.y());
      float roll = std::atan2(sinr_cosp, cosr_cosp);

      // pitch (y-axis rotation)
      float sinp = 2 * (quat_a.w() * quat_a.y() - quat_a.z() * quat_a.x());
      float pitch;
      if (std::abs(sinp) >= 1)
          pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
          pitch = std::asin(sinp);

      // yaw (z-axis rotation)
      float siny_cosp = 2 * (quat_a.w() * quat_a.z() + quat_a.x() * quat_a.y());
      float cosy_cosp = 1 - 2 * (quat_a.y() * quat_a.y() + quat_a.z() * quat_a.z());
      float yaw = std::atan2(siny_cosp, cosy_cosp);

    

      float cy = cos(-yaw * 0.5);
      float sy = sin(-yaw * 0.5);
      float cp = cos((p_offset) * 0.5);
      float sp = sin((p_offset) * 0.5);
      float cr = cos((r_offset) * 0.5);
      float sr = sin((r_offset) * 0.5);

      quat_t q;
      q.w() = cr * cp * cy + sr * sp * sy;
      q.x() = sr * cp * cy - cr * sp * sy;
      q.y() = cr * sp * cy + sr * cp * sy;
      q.z() = cr * cp * sy - sr * sp * cy;

      quat_t q_flip;
      q_flip.w() = 0;
      q_flip.x() = 1;
      q_flip.y() = 0;
      q_flip.z() = 0;

      q = q_flip.inverse()*q;
      
//      quat_init = quat_t((float)state[9], (float)state[6], (float)state[7], (float)state[8]);
      quat_init  = q;

//      if (quat_init.w() > 0) {
//        quat_init.w() = -quat_init.w();
//        quat_init.x() = -quat_init.x();
//        quat_init.y() = -quat_init.y();
//        quat_init.z() = -quat_init.z();
//      }
//      
    Serial.println(yaw); Serial.println("-------------------");
      quat_init_inverse = quat_init.inverse();
      initialized = true;
      koios->setLogo('G');
    }
    { Threads::Scope scope(state_mtx);
      quat_a = quat_init_inverse * quat_a;
  
      state[6] = quat_a.w();
      state[7] = quat_a.x();
      state[8] = quat_a.y();
      state[9] = quat_a.z();
    }
  }

  while (!ESP_connected) {threads.delay_us(100);}

  //check if imu data is not corrupted
//  int rt2 = koios->checkFrame(q0, q1, q2, q3, dY, dP, dR);
  bool rt2 = abs(q0) < 2 && abs(q1)<2 && abs(q2)<2 && abs(q3)<2 && abs(dY) < 1e5 && abs(dP) < 1e5 && abs(dR) < 1e5;
  //  Serial.println(rt2);
  //based on imu upadate states
  if (rt2 == 1) {
    Q0 = q0;
    Q1 = q1;
    Q2 = q2;
    Q3 = q3;
    DY = dY;
    DP = dP;
    DR = dR;
  }
  koios->updateStates(x1, v1, x2, v2, x3, v3);
  //  int new_contact = koios->getIntFromB();
  //  if (new_contact != -1) {
  //    contact = new_contact;
  //  }
  //  Serial.println(contact);
  // Add step to get leg length from Bia here over serial
  { Threads::Scope scope(state_mtx);
    Threads::Scope scope2(foot_state_mtx);
    state[0] = v1;
    state[1] = v3;
    state[2] = v2;
    state[3] = DR;
    state[4] = DP;
    state[5] = DY;
    state[6] = Q0;
    state[7] = Q1;
    state[8] = Q2;
    state[9] = Q3;
    state[10] = foot_state[0];
    state[11] = foot_state[1];
    state[12] = foot_state[2];

    state_tmp << state[9], state[6], state[7], state[8];
    if (state_tmp.norm() > 1.05 || state_tmp.norm() < 0.95) {
      Serial.print("Exiting because state norm was: ");
      Serial.println(state_tmp.norm());
      exitProgram();
    }      
      // -1 on w so that the double cover is resolved.
      quat_a = quat_t((float)state[9], (float)state[6], (float)state[7], (float)state[8]); // assuming q_w is last.
      quat_a = quat_init_inverse * quat_a;
      
//      Serial.print(quat_a.w()); Serial.print(", ");
//      Serial.print(quat_a.x()); Serial.print(", ");
//      Serial.print(quat_a.y()); Serial.print(", ");
//      Serial.print(quat_a.z()); Serial.print(",                    ");
//      Serial.println();
      
      

      state[6] = quat_a.w();
      state[7] = quat_a.x();
      state[8] = quat_a.y();
      state[9] = quat_a.z();
  }

  memcpy(oneAdded, receivedCharsESP + 40, 6 * sizeof(char));
  for (int i = 0; i < 6; i++) {
    for (int j = 1; j < 8; j++) {
      if (oneAdded[i] & (1 << (8 - j))) {
        receivedCharsESP[i * 7 + (j - 1)] = 0;
      }
    }
  }

  float state_d[10];
  memcpy(state_d, receivedCharsESP, 10 * 4);

  quat_t quat_d = quat_t(state_d[0], state_d[1], state_d[2], state_d[3]);
  vector_3t omega_d = vector_3t(state_d[4], state_d[5], state_d[6]);
  vector_3t tau_ff = vector_3t(state_d[7], state_d[8], state_d[9]);
  vector_3t omega_a = vector_3t(state[3], state[4], state[5]);
//  quat_t quat_d = quat_t(1,0,0,0);
//  vector_3t omega_d = vector_3t(0,0,0);
//  vector_3t tau_ff = vector_3t(0,0,0);

  //use for the counication with the wheel motors
  //convert torques to amps with torque / 0.083 = currents [A]
  //for a range of -1.6Nm to 1.6 Nm
  vector_3t current;
  if (initialized) {
    getTorque(state, quat_d, omega_d, tau_ff, quat_a, current);
  } else {
    current[0] = 0;
    current[1] = 0;
    current[2] = 0;
  }

  ////////////////////Safety///////////////////////
  for (int i = 0; i < 3; i++) {
    if (current[i] > MAX_CURRENT) {
      current[i] = MAX_CURRENT;
    } else if (current[i] < MIN_CURRENT) {
      current[i] = MIN_CURRENT;
    }
  }

  if (reset_cmd) {
    elmo.cmdTC(-0.1 * v1, IDX_K1);
    elmo.cmdTC(-0.1 * v3, IDX_K2);
    elmo.cmdTC(-0.1 * v2, IDX_K3);
    if (abs(v1) < 0.01 && abs(v2) < 0.01 && abs(v3) < 0.01) {
      reset_cmd = 0;
    }
  } else {
          // fix current to zero 
//        current[0] = 0;
//        current[1] = 0;
//        current[2] = 0;
        elmo.cmdTC(current[0],IDX_K1);
        elmo.cmdTC(current[1],IDX_K2);
        elmo.cmdTC(current[2],IDX_K3);
  }

//  vector_3t omega_a = vector_3t(state[3], state[4], state[5]);

  uint32_t Tc1 = micros();
  Serial.println(Tc1);
  data.print(Tc1);              data.print(",");
  data.print(quat_a.w(),4);     data.print(",");
  data.print(quat_a.x(),4);     data.print(",");
  data.print(quat_a.y(),4);     data.print(",");
  data.print(quat_a.z(),4);     data.print(",");
  data.print(quat_d.w(),4);     data.print(",");
  data.print(quat_d.x(),4);     data.print(",");
  data.print(quat_d.y(),4);     data.print(",");
  data.print(quat_d.z(),4);     data.print(",");
  data.print(omega_a[0],4);     data.print(",");
  data.print(omega_a[1],4);     data.print(",");
  data.print(omega_a[2],4);     data.print(",");
  data.print(omega_d[0],4);     data.print(",");
  data.print(omega_d[1],4);     data.print(",");
  data.print(omega_d[2],4);     data.print(",");
  data.print(foot_state[0],4);  data.print(",");
  data.print(foot_state[1],4);  data.print(",");
  data.print(foot_state[2],4);  data.print(",");
  data.print(current[0],4);     data.print(",");
  data.print(current[1],4);     data.print(",");
  data.print(current[2],4);     data.println(); 


    if (time_initialized) {
      current_ESP_message = millis();
      if (current_ESP_message - last_ESP_message > TIMEOUT_INTERVAL) {
        Serial.print("Exiting because ESP message took: ");
        Serial.println(current_ESP_message - last_ESP_message);
        exitProgram();
      }
    }
   
  //  Serial.println(Tc1-Tc0);

  // send u4 current command to leg over serial RX/TX between teensy boards
  delay(1);
}
