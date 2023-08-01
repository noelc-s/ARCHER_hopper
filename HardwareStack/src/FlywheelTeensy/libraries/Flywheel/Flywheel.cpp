#include "Arduino.h"
#include "Archer_Config.h"
#include "ArduinoEigen.h"
#include "TripENC.h"
#include "ELMO_CANt4.h"
#include <SD.h>
#include <TeensyThreads.h>

using vector_3t = Eigen::Matrix<float, 3, 1>;
using vector_4t = Eigen::Matrix<float, 4, 1>;
using matrix_t = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using matrix_3t = Eigen::Matrix<float, 3, 3>;
using quat_t = Eigen::Quaternion<float>;


// Global variables are bad. fix this in next update
extern float foot_state[3];
extern bool initialized;
extern Threads::Mutex state_mtx;
extern float state[13];
extern char receivedCharsESP[46];
extern bool time_initialized;
extern unsigned long last_ESP_message;
extern bool ESP_connected;
extern volatile float dR, dP, dY, d1, v1, x2, v2, x2, v3, q0, q1, q2, q3;
extern int nF;
extern float torque_to_current;
extern double kp_y, kp_rp, kd_y, kd_rp, r_offset, p_offset;
extern double wifi_on, foot_on, flywheel_on, debug_on;
extern Archer::ELMO_CANt4 elmo;
extern char aRecord[30];

void delayLoop(uint32_t T1, uint32_t L) {
  uint32_t T2 = micros();
  if ((T2 - T1) < L) {
    uint32_t a = L + T1 - T2;
    threads.delay_us(a);
  }
}

void BiaThread() {
  while (1) {
    int index = 0;
    char receivedCharsBia[11];
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
    char oneAddedBia[2];
    memcpy(oneAddedBia, receivedCharsBia + 9, 2 * sizeof(char));
    for (int i = 0; i < 2; i++) {
      for (int j = 1; j < 8; j++) {
        if (oneAddedBia[i] & (1 << (8 - j))) {
          receivedCharsBia[i * 7 + (j - 1)] = 0;
        }
      }
    }
    foot_state[0] = (float) receivedCharsBia[0];
    memcpy(foot_state+1, receivedCharsBia + 1, 2 * 4);
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
      byt = IMU_PORT.read();
      if (byt == 0xFA) {
        delayMicroseconds(350);
        for (int i = 0; i < 3; i++) {
          IMU_PORT.read();
        }
        //=========================================================
        union {
          long y;
          float z;
        } Q0_;
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
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
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
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
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
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
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
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
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
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
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
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
        b0 = IMU_PORT.read();
        b1 = IMU_PORT.read();
        b2 = IMU_PORT.read();
        b3 = IMU_PORT.read();
        uint8_t bY[4] = {b0, b1, b2, b3};
        x = (long)bY[3] << 24 | (long)bY[2] << 16 | bY[1] << 8 | bY[0];
        YY.y = x;
        { Threads::Scope scope(state_mtx);
        dY = YY.z;
        }
        IMU_PORT.read();
        IMU_PORT.read();
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
  Kp.setZero();
  Kd.setZero();
  Kp.diagonal() << kp_rp, kp_rp, kp_y;
  Kd.diagonal() << kd_rp, kd_rp, kd_y;

  Serial.println("========================================================");
  Serial.print("qd.w = "); Serial.print(quat_d.w(),3);  Serial.print(", "); 
  Serial.print("qd.x = "); Serial.print(quat_d.x(),3);  Serial.print(", ");    
  Serial.print("qd.y = "); Serial.print(quat_d.y(),3);  Serial.print(", ");     
  Serial.print("qd.z = "); Serial.println(quat_d.z(),3);   
  Serial.print("qa.w = "); Serial.print(quat_a.w(),3);  Serial.print(", ");    
  Serial.print("qa.x = "); Serial.print(quat_a.x(),3);  Serial.print(", ");   
  Serial.print("qa.y = "); Serial.print(quat_a.y(),3);  Serial.print(", ");   
  Serial.print("qa.z = "); Serial.println(quat_a.z(),3);
  Serial.print("dd.x = "); Serial.print(delta_quat[0],3);  Serial.print(", ");    
  Serial.print("dd.y = "); Serial.print(delta_quat[1],3);  Serial.print(", ");   
  Serial.print("dd.z = "); Serial.println(delta_quat[2],3);   

  vector_3t tau_fb;
  delta_omega = omega_a - omega_d;
  tau_fb = -quat_actuator.inverse()._transformVector(Kp * delta_quat) - quat_actuator.inverse()._transformVector(-Kd * delta_omega);

  torque << (tau_fb + tau_ff)*torque_to_current;
}

void writeData(File data, uint32_t Tc1, quat_t quat_a, quat_t quat_d, vector_3t omega_a, vector_3t omega_d, float* foot_state, vector_3t current) {
  
  //Serial.println(Tc1);
  data.print(Tc1);
  data.print(",");
  data.print(quat_a.w(), 4);
  data.print(",");
  data.print(quat_a.x(), 4);
  data.print(",");
  data.print(quat_a.y(), 4);
  data.print(",");
  data.print(quat_a.z(), 4);
  data.print(",");
  data.print(quat_d.w(), 4);
  data.print(",");
  data.print(quat_d.x(), 4);
  data.print(",");
  data.print(quat_d.y(), 4);
  data.print(",");
  data.print(quat_d.z(), 4);
  data.print(",");
  data.print(omega_a[0], 4);
  data.print(",");
  data.print(omega_a[1], 4);
  data.print(",");
  data.print(omega_a[2], 4);
  data.print(",");
  data.print(omega_d[0], 4);
  data.print(",");
  data.print(omega_d[1], 4);
  data.print(",");
  data.print(omega_d[2], 4);
  data.print(",");
  data.print(foot_state[0], 4);
  data.print(",");
  data.print(foot_state[1], 4);
  data.print(",");
  data.print(foot_state[2], 4);
  data.print(",");
  data.print(current[0], 4);
  data.print(",");
  data.print(current[1], 4);
  data.print(",");
  data.print(current[2], 4);
  data.println();
}
