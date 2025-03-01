#include <Archer_Config.h>
#include <DualENC.h>
#include <ELMO_CANt4.h>
#include <ControlBia.h>
#include <Bia.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <ArduinoEigen.h>

using namespace Archer;
using namespace Eigen;

#define MAX_HOP_TIMEOUT 1000000000  // 1000000
#define PULLEYMOTOR 1
#define FOOT 2

DualENC dENC(doub_CS1, doub_CS2);
ELMO_CANt4 elmo;
ControlBia cBia(1.0);
Bia bia(dENC, elmo, cBia);

uint32_t T1, Th;
float tau_max = 10;
int numHops = 5000;
volatile uint32_t T0, nF, pF, rt;
volatile float rb, wb, xf, vf, u;
float d0 = 0.015;  // spring deflection // 0.15
float b0 = 0.75;   // deflection to consider impact
// float u0 = 0.0; // offset torque
float u0 = -15.5;  // offset torque
int h = 0;
float rb0, v0;
volatile bool initialized;

volatile char contact = 0;
char footStateToKoios[1 + 2 * 4 + 2 + 1];  // contact, foot_state, bitAdded, newline
float x_foot = 0;
float xdot_foot = 0;
float x_meters = 0;
float v_meters = 0;
float theta_pulley = 0;
float thetadot_pulley = 0;

void findZero() {
  // Serial.println("Finding Zero");
  float theta_pulley, thetadot_pulley;
  float x_foot, xdot_foot;
  float x_foot_compression_distance = 0.02;
  float x_foot_uncompression_distance = 0.01;
  float u = 0.0;
  // float u_total = 0.0;
  int fsm = 0;

  // float kd = 1;
  // float thetadot_des = 0;

  uint32_t Ts0 = micros();
  uint32_t dTs;
  while (fsm < 1) {
    u = -0.5;
    rt = elmo.sendTC(u, 4);
    dTs = micros() - Ts0;
    if (dTs > 1000000) {
      fsm = 1;
    }
  }

  bia.updateState(PULLEYMOTOR, theta_pulley, thetadot_pulley);
  bia.setRB0(theta_pulley);

  // Serial.println("Pulling in");
  while (fsm < 2) {
    u = u - 0.0005;
    // Serial.print("U: ");
    // Serial.println(u);
    if (abs(u) > 10) {
      // Serial.println("Error. Required too much torque in initialization. Exiting.");
      bia.exitProgram();
    }

    // float error = thetadot_pulley - thetadot_des;
    // u_total = u - kd * error;

    bia.updateState(PULLEYMOTOR, theta_pulley, thetadot_pulley);
    bia.updateState(FOOT, x_foot, xdot_foot);
    bia.sendSafeTorque(theta_pulley, u);
    if (x_foot > x_foot_compression_distance) {
      fsm = 2;
      cBia.logZero(theta_pulley, 1);
    }
    Serial.print(x_foot); Serial.print("; ");
    Serial.print(xdot_foot); Serial.println(";        ");
  }
  // Serial.println("Deflection Registered.");
  // Serial.println("Releasing");
  while (fsm < 3) {
    u = u + 0.0005;
    if (u > 6) {
      // Serial.println("Error. Torque went above 1. Exiting.");
      bia.exitProgram();
    }
    bia.updateState(PULLEYMOTOR, theta_pulley, thetadot_pulley);
    bia.updateState(FOOT, x_foot, xdot_foot);
    bia.sendSafeTorque(theta_pulley, u);
    if (x_foot < x_foot_uncompression_distance) {
      fsm = 3;
      cBia.logZero(theta_pulley, 2);
    }
    Serial.print(x_foot); Serial.print("; ");
    Serial.print(xdot_foot); Serial.println(";        ");
  }
  // Serial.println("Done.");
}

void setup() {
  Serial.begin(115200);  //this is for the monitor
  Serial.println("Starting");
  delay(500);

  //initBia1
  bia.setLEDs("0100");
  delay(3000);
  rt = bia.initComm(1);
  // delay(1000);
  if (rt > 0) {
    bia.setLEDs("0010");
  } else {
    bia.flashR(10);
  }
  delay(10);

  cBia.setTx(tau_max);
  v0 = 100;
  Th = 2000000 * numHops + 1000000;

  Serial.println("Starting Bia2");
  delay(250);
  bia.STO(1);
  bia.waitSigK(1);
  bia.setLEDs("1111");
  delay(250);
  elmo.motorOn(IDX_BIA);
  delay(3000);
  bia.resetState(1);
  bia.resetState(2);
  Serial.println("Finding Zero");
  findZero();
  bia.setLEDs("0100");
  bia.waitSigK(0);
  bia.reverseSig(1);
  bia.setSigK(0);
  delay(50);
  bia.setLEDs("0001");
  delay(70);

  cBia.getRB0(rb0);
  bia.setRB0(rb0);
  bia.resetState(1);
  bia.resetState(2);
  T0 = micros();
  T1 = T0;
  bia.setLEDs("0001");

  rt = threads.setSliceMicros(50);
  contact = 0;
  threads.addThread(KoiosCommThread);
}

std::mutex state_mtx;

void KoiosCommThread() {
  while (1) {
    {
      std::lock_guard<std::mutex> lck(state_mtx);
      footStateToKoios[0] = contact;
      x_meters = x_foot / 1000;
      v_meters = xdot_foot / 1000;
      memcpy(footStateToKoios + 1, &x_meters, 4);
      memcpy(footStateToKoios + 5, &v_meters, 4);
    }
    // Serial.print((float)footStateToKoios[0]); Serial.print(";  ");
    // Serial.print(x_foot); Serial.print("; ");
    // Serial.print(xdot_foot); Serial.println(";        ");
    // Serial.print(theta_pulley); Serial.print("; ");
    // Serial.print(thetadot_pulley); Serial.println(";  ");

    for (int i = 0; i < 2; i++) {
      byte oneAdded = 0b00000001;
      for (int j = 1; j < 8; j++) {
        if (footStateToKoios[i * 7 + (j - 1)] == 0b00000000) {
          footStateToKoios[i * 7 + (j - 1)] = 0b00000001;
          oneAdded += (1 << (8 - j));
        }
      }
      memcpy(&footStateToKoios[9 + i], &oneAdded, 1);
    }
    footStateToKoios[11] = 0b0;

    K_PORT.write(footStateToKoios);
    K_PORT.flush();
    threads.delay_us(1000);
  }
}

void loop() {

  // threads.delay(100);
  //   // Update states:
  // bia.updateState(PULLEYMOTOR,theta_pulley, thetadot_pulley); // theta_pulley and thetadot_pulley are motor angle and vel
  // {std::lock_guard<std::mutex> lck(state_mtx);
  // bia.updateState(2,x_foot, xdot_foot); // x_foot and xdot_foot are spring deflection in mm
  // }

  compPhase();  //
  {
    std::lock_guard<std::mutex> lck(state_mtx);
    contact = 1;
  }
  releasePhase();  // control to rb = 0, end at xf = 0
  {
    std::lock_guard<std::mutex> lck(state_mtx);
    contact = 0;
  }
}

void compPhase() {
  float xfs;
  uint32_t Tc0 = micros();
  uint32_t Ts0, dTs;
  int fsm = 0;
  Serial.println("------------Comp Phase---------------");

  bia.updateState(PULLEYMOTOR, theta_pulley, thetadot_pulley);  // theta_pulley and thetadot_pulley are motor angle and vel
  float theta_pulley_0 = theta_pulley;

  while (fsm < 4) {
    contact = 0;
    // WIFI ESTOP:
    if (bia.checkSigK() == 1) {
      bia.exitProgram();
    }
    // Update states:
    bia.updateState(PULLEYMOTOR, theta_pulley, thetadot_pulley);  // theta_pulley and thetadot_pulley are motor angle and vel
    {
      std::lock_guard<std::mutex> lck(state_mtx);
      bia.updateState(FOOT, x_foot, xdot_foot);  // x_foot and xdot_foot are spring deflection in mm
    }

    float kp = 0.4;
    float kd = 0.04;
    float x_star = 30;
    u = -kp * (x_foot - x_star) - kd * xdot_foot;
    //    Serial.println(x_foot);
    if (theta_pulley - theta_pulley_0 >= 1.0) {
      Serial.println("Exiting because t-t_0 deflection was too large");
      bia.exitProgram();
    }
    // rt = elmo.sendTC(-u + u0, 4);
    rt = bia.sendSafeTorque(theta_pulley, -u + u0);



    if (fsm == 0) {     // Waiting for compression
      if (x_foot > 8.0) {  // Check for enough deflection
        fsm = 1;
      }
    }
    if (fsm == 1) {         // waiting to slow down
      if (abs(xdot_foot) < 20) {  // Check for slow movement
        fsm = 2;
        Ts0 = micros();
      }
    }
    if (fsm == 2) {  // waiting for settling
      dTs = micros() - Ts0;
      if (abs(xdot_foot) > 20) {  // Check if it sped up
        fsm = 1;
      } else if (dTs > 50000) {  // Check if settle time reached (50000)
        fsm = 3;
        xfs = x_foot;
      }
    }
    if (fsm == 3) {  // Check if impact occured
      if ((x_foot - xfs) > 0.75) {
        fsm = 4;
      }
    }

    Serial.print(x_foot); Serial.print("; ");
    Serial.print(xdot_foot); Serial.print(";        ");
    Serial.print(theta_pulley); Serial.print("; ");
    Serial.print(thetadot_pulley); Serial.println(";        ");
  }
}

void releasePhase() {
  uint32_t Tr0 = micros();
  float up, ud;
  int fsm = 0;
  Serial.println("------------Release Phase---------------");
  while (fsm < 1) {
    // WIFI ESTOP:
    if (bia.checkSigK() == 1) {
      bia.exitProgram();
    }
    {
      std::lock_guard<std::mutex> lck(state_mtx);
      bia.testPDb(theta_pulley, thetadot_pulley, x_foot, xdot_foot, up, ud);
    }
    rb = theta_pulley;
    wb = thetadot_pulley;
    vf = xdot_foot;
    u = up + ud;
    if (x_foot < 0.5) {
      fsm = 1;
    }
    delayLoop(T1, 1000);
    T1 = micros();
    nF++;
  }
}

void delayLoop(uint32_t T1, uint32_t dT) {
  uint32_t T2 = micros();
  if ((T2 - T1) < dT) {
    uint32_t a = dT + T1 - T2;
    threads.delay_us(a);
  }
}
