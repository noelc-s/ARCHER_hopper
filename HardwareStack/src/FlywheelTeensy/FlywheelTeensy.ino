/*
  Arduino Code to be uploaded to the flywheel (Koios board)
  To ensure that everything works, do:
  File > Preferences > Settings > Sketchbook location:
  <path-to-hardware>/HardwareStack/src/FlywheelTeensy
*/

#include <Archer_Config.h>
#include <TripENC.h>
#include <ELMO_CANt4.h>
#include <Koios.h>
#include <SPI.h>
#include <SD.h>
#include <TeensyThreads.h>
#include <ArduinoEigen.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <QNEthernet.h>   // Library for Ethernet Communication

using namespace Archer;
using namespace Eigen;
using namespace qindesign::network;     // Namespace for QNEthernet

//==================CONSTANTS
TripENC tENC(trip_CS1, trip_CS2, trip_CS3);
ELMO_CANt4 elmo;
bool ethernet_connected = false;

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
volatile float q0  = 1;
volatile float q1 = 0;
volatile float q2 = 0;
volatile float q3 = 0;
quat_t q_pitch; // negative 180 pitch to flip IMU right way around
// FYI, IMU is installed upside down

int reset_cmd = 0;

quat_t quat_init;
quat_t quat_init_inverse;
volatile bool initialized = false;
bool time_initialized = false;

Koios *koios;

int nF;
bool rt = 0;

boolean newData = false;
//this is only for the first debugging run
float state[13];    //current_state
float state_d[10];  //desired_state

// --------------------------------------------------------------------------
//  Ethernet Configuration
// --------------------------------------------------------------------------

constexpr uint16_t kPort = 4333;  // Chat port
EthernetUDP udp;                  //UDP Port
bool read_packet = false;


//===========SPEEDING UP BABY
char additional_read_buffer[3000]; //this values are out of nowhere
char additional_write_buffer[3000];


unsigned long last_Ethernet_message;
unsigned long current_Ethernet_message;
// char receivedCharsESP[46];
float foot_state[3];

//=================SETUP=============

bool exit_state = false;
File gainFile;
File data;
String gain_config = "gain_config.txt";
String dFile = "log.txt";

// SD CARD Variables
const byte NUMBER_OF_RECORDS = 10; // number of vars in gain_config.txt
char parameterArray[NUMBER_OF_RECORDS][30];
char aRecord[30];
byte recordNum;
byte charNum;
// For rotation about x and y (this is what Noel and I did the other day)
// The proportional and derivative gain should both be the same since the dynamics are coupled
double kp_y;
double kp_rp;
double kd_y;
double kd_rp;
// We inserted this bias because the true CG is a little of center
double r_offset;
double p_offset;
// experiment settings
double comms_on;       // 1 ESP is on, 0 ESP off
double foot_on;       // 1 foot is on, 0 foot is off. 
double flywheel_on;   // 1 flywheels on, 0 flywheels off. 
double debug_on;      // 1 if consant upright position, 0 if MPC trajs
double send_torque;
double pitch_offset;

// for parsing SD card txt files, line-by-line
void parseRecord(byte index){
  char * ptr;
  ptr = strtok(aRecord, " = ");  //find the " = "
  ptr = strtok(NULL, ""); //get remainder of text
  strcpy(parameterArray[index], ptr + 2); //skip 2 characters and copy to array
}  

  // start a diode to be sure all is working
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);

void setup() {
  //====================WIFI==============
  delay(100); //give time to open and print
  Serial.begin(115200); //this is for the monitor
  setupEthernet();      //Ethernet Setup
  readParams();         // read params from SD card
  data = SD.open(dFile.c_str(),FILE_WRITE);

  //  //================Koios=============
  koios = new Koios(tENC, elmo);
  koios->initKoios1(1);
  // initKoios2
  delay(250);
  koios->STO(1);
  koios->waitSwitch(1); //manual switch on robot
  if (foot_on > 0) {koios->setSigB(1); // Turn on comms with Bia if foot_on is true
    Serial.print("Foot is On");}
  // delay(250);
  rt = koios->motorsOn();
  Serial.print("Motors on? (1 if true): ");
  Serial.println(rt);
  // delay(5000);
  koios->waitSwitch(0);
  koios->resetStates();
  koios->setLEDs("0100");
  koios->setSigB(0);
  koios-> setLogo('A');
  koios->setLEDs("0001");
  // koios->flashG(2);

  rt = threads.setSliceMicros(25);
  threads.addThread(imuThread);
  threads.addThread(BiaThread);
  threads.addThread(EthernetThread);
  delay(500);
  foot_state[0] = 0;
  foot_state[1] = 0;
  foot_state[2] = 0;
  koios->setLEDs("0001");
  koios->setLogo('R');
  delay(500);
}

//============FUNCTIONS==========

// ===========ETHERNET=================
void setupEthernet(){

  printf("Starting...\r\n");

  uint8_t mac[6];
  Ethernet.macAddress(mac);  // This is informative; it retrieves, not sets
  printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Ethernet.onLinkState([](bool state) {
    printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
  });

  IPAddress ip(10, 0, 0, 7);
  IPAddress sn{255,255,255,0};  // Subnet Mask
  IPAddress gw{10,0,0,1};       // Default Gateway

  if (!Ethernet.begin(ip, sn, gw)) {
    printf("Failed to start Ethernet\r\n");
    return;
  }

  printf("Obtaining the IP, Subnet and Gateway\r\n");
  ip = Ethernet.localIP();
  printf("    Local IP     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.subnetMask();
  printf("    Subnet mask  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.broadcastIP();
  printf("    Broadcast IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.gatewayIP();
  printf("    Gateway      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.dnsServerIP();
  printf("    DNS          = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);

  // Start UDP listening on the port
  udp.begin(kPort);

  // t1 = micros();
}

void readParams() {
  //==============LOAD IN FROM SD CARD=============
  // init SD card module, using built-in Teensy SD
  Serial.print("Initializing SD card... ");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println(" initialization done.");

  // open gain file config
  gainFile = SD.open(gain_config.c_str());

  // read gain config parameters
  if (gainFile) {
    Serial.println("SD card opened succesfully. Reading now.");

    while (gainFile.available())
    {
      char inChar = gainFile.read();  //get a character
      if (inChar == '\n') //if it is a newline
      {
        parseRecord(recordNum);
        recordNum++;
        charNum = 0;  //start again at the beginning of the array record
      }
      else
      {
        aRecord[charNum] = inChar;  //add character to record
        charNum++;  //increment character index
        aRecord[charNum] = '\0';  //terminate the recordBUILT
      }
    }
    gainFile.close();
  }
  else {
    Serial.print("Error opening "); Serial.println(gain_config);
  }

  // Assign gain_config values from SD card to actual values
  kp_y =     atof(parameterArray[0]);
  kp_rp =    atof(parameterArray[1]);
  kd_y =     atof(parameterArray[2]);
  kd_rp =    atof(parameterArray[3]);
  r_offset = atof(parameterArray[4]);
  p_offset = atof(parameterArray[5]);
  comms_on  = atof(parameterArray[6]);
  foot_on  = atof(parameterArray[7]);
  send_torque  = atof(parameterArray[8]);
  pitch_offset  = atof(parameterArray[9]);

  if (pitch_offset > 0)
    q_pitch = quat_t(0,0,-1,0);
  else
    q_pitch = quat_t(1,0,0,0);

  Serial.println("Loaded Parameters: ");
  Serial.print("kp_y = ");     Serial.println(kp_y,2);
  Serial.print("kp_rp = ");    Serial.println(kp_rp,2);
  Serial.print("kd_y = ");     Serial.println(kd_y,2);
  Serial.print("kd_rp = ");    Serial.println(kd_rp,2);
  Serial.print("r_offset = "); Serial.println(r_offset,3);
  Serial.print("p_offset = "); Serial.println(p_offset,3);
  Serial.print("comms_on = ");  Serial.println(comms_on,0);
  Serial.print("foot_on = ");  Serial.println(foot_on,0);
  Serial.print("send_torque = "); Serial.println(send_torque,0);
  Serial.print("pitch_offset = "); Serial.println(pitch_offset,0);

  //=================================================
}

// Receives and prints chat packets.
void receivePacket() {
  // printf("receiving.. \n");
  
  int size = udp.parsePacket();
  if (size < 0) {
    threads.yield();
    return;
  };

  // Get the packet data and remote address
  const uint8_t *data = udp.data();
  // IPAddress ip = udp.remoteIP();

  //printf("[%u.%u.%u.%u][%d] ", ip[0], ip[1], ip[2], ip[3], size);
  
  for(int i = 0; i < (sizeof(state_d)/sizeof(float)); i++){
    state_d[i] = 0.0;
  }

  memcpy(state_d, data, sizeof(float) * 10);
  if (data[0] == 1 && data[1] == 2 && data[2] == 3 && data[3] == 4 &&
      data[4] == 5 && data[5] == 6 && data[6] == 7 && data[7] == 8) {
    reset_cmd = 1;
  }

  // printf("%u", sizeof(rcvdData));
  // printf("%u", sizeof(float));
  // for(int i = 0; i<sizeof(rcvdData)/sizeof(float); i++){
  //   printf("%f", rcvdData[i]);
  // }

  // printf("\r\n");
  read_packet = true;
  last_Ethernet_message = millis();
  ethernet_connected = true;
}

static void sendPacket() {
  // printf("sending ..\n");

  // float value[13] = {4.3, 4.2, -1, 9.12, 2.22, 3.64, 4.005,
  //                     0.44, 42.4, 222.33, 2232.3, 44.33, 31.11};
  // Serialize the float value to a byte array
  
  char line[sizeof(float) * 13];
  memcpy(line, state, sizeof(float) * 13);
  
  if (read_packet) {
    IPAddress ip_send(10,0,0,6);
    if (!udp.send(ip_send, kPort,
                  reinterpret_cast<const uint8_t *>(line),
                  sizeof(float)*13)) {
      printf("[Error sending]\r\n");
    }
  
  read_packet = false;
  threads.delay_us(100);
  }
}


void delayLoop(uint32_t T1, uint32_t L) {
  uint32_t T2 = micros();
  if ((T2 - T1) < L) {
    uint32_t a = L + T1 - T2;
    threads.delay_us(a);
  }
}

// volatile bool ESP_connected = false;

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

void EthernetThread() {
  // bool read_data = true;
  while (1) {
    if (initialized){
      receivePacket();
      sendPacket();
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
  Kp.setZero();
  Kd.setZero();
  Kp.diagonal() << kp_rp, kp_rp, kp_y;
  Kd.diagonal() << kd_rp, kd_rp, kd_y;

  vector_3t tau_fb;
  delta_omega = omega_a - omega_d;
  tau_fb = -quat_actuator.inverse()._transformVector(Kp * delta_quat) - quat_actuator.inverse()._transformVector(-Kd * delta_omega);

  torque << (tau_fb + tau_ff)*torque_to_current;
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
  static float Q1 = 0;
  static float Q2 = 0;
  static float Q3 = 0;
  static float DY = 0;
  static float DP = 0;
  static float DR = 0;

  if (Serial.available() > 0) {
    while (Serial.available() > 0)
      Serial.read();
    reset_cmd = 1;
  };

  quat_t q_tmp(0,0,0,0);

  while (!initialized) {
    while (q_tmp.norm() < 0.95 || q_tmp.norm() > 1.05) {
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
        quat_t q_measured(Q3, Q0, Q1, Q2); // Quaternions are a double cover of SO(3).
        quat_a = q_pitch.inverse()*q_measured;
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
        state[6] = quat_a.w();
        state[7] = quat_a.x();
        state[8] = quat_a.y();
        state[9] = quat_a.z();
        state[10] = foot_state[0];
        state[11] = foot_state[1];
        state[12] = foot_state[2];
      }
      q_tmp = quat_a;
    }
//      vector_4t quat_vec = vector_4t((float)state[9], (float)state[6], (float)state[7], (float)state[8]);
//      vector_3t log_quat_vec;
//      log_quat_vec = quat_vec.segment(1,3)/quat_vec.segment(1,3).norm()*acos(quat_vec(0)/quat_vec.norm());
//      vector_3t xi_0;
//      xi_0 << r_offset, p_offset+1.5708, log_quat_vec(2);
//      vector_4t quat_init_vec;
//      quat_init_vec << cos(xi_0.norm()), xi_0/xi_0.norm()*sin(xi_0.norm());
//      quat_init = quat_t(quat_init_vec(0), quat_init_vec(1), quat_init_vec(2), quat_init_vec(3));
//
//      Serial.println();
//      Serial.print(log_quat_vec[0]); Serial.print(", ");
//      Serial.print(log_quat_vec[1]); Serial.print(", ");
//      Serial.print(log_quat_vec[2]); Serial.print(", ");
//      Serial.println();
//      Serial.print(xi_0[0]); Serial.print(", ");
//      Serial.print(xi_0[1]); Serial.print(", ");
//      Serial.print(xi_0[2]); Serial.print(", ");
//      Serial.println();
//      Serial.println();
//            
//      Serial.print(state[9]); Serial.print(", ");
//      Serial.print(state[6]); Serial.print(", ");
//      Serial.print(state[7]); Serial.print(", ");
//      Serial.print(state[8]); Serial.print(", ");
//      Serial.println();
//      Serial.println();
//
//      Serial.print(quat_init.w()); Serial.print(", ");
//      Serial.print(quat_init.x()); Serial.print(", ");
//      Serial.print(quat_init.y()); Serial.print(", ");
//      Serial.print(quat_init.z()); Serial.print(", ");
//      Serial.println();

      //      // roll (x-axis rotation)
      // float sinr_cosp = 2 * (quat_a.w() * quat_a.x() + quat_a.y() * quat_a.z());
      // float cosr_cosp = 1 - 2 * (quat_a.x() * quat_a.x() + quat_a.y() * quat_a.y());
      // float roll = std::atan2(sinr_cosp, cosr_cosp);

      // // pitch (y-axis rotation)
      // float sinp = 2 * (quat_a.w() * quat_a.y() - quat_a.z() * quat_a.x());
      // float pitch;
      // if (std::abs(sinp) >= 1)
      //     pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      // else
      //     pitch = std::asin(sinp);

      // // yaw (z-axis rotation)
      // float siny_cosp = 2 * (quat_a.w() * quat_a.z() + quat_a.x() * quat_a.y());
      // float cosy_cosp = 1 - 2 * (quat_a.y() * quat_a.y() + quat_a.z() * quat_a.z());
      // float yaw = std::atan2(siny_cosp, cosy_cosp);

      // static float r_offset = 0.002;
      // static float p_offset = -0.02;

      // float cy = cos(yaw * 0.5);
      // float sy = sin(yaw * 0.5);
      // float cp = cos((p_offset) * 0.5);
      // float sp = sin((p_offset) * 0.5);
      // float cr = cos(((-1*(roll<0))*3.14+r_offset) * 0.5);
      // float sr = sin(((-1*(roll<0))*3.14+r_offset) * 0.5);

      // quat_t q;
      // q.w() = cr * cp * cy + sr * sp * sy;
      // q.x() = sr * cp * cy - cr * sp * sy;
      // q.y() = cr * sp * cy + sr * cp * sy;
      // q.z() = cr * cp * sy - sr * sp * cy;
      
      // quat_init = quat_t((float)state[9], (float)state[6], (float)state[7], (float)state[8]);
      auto initEuler = quat_a.toRotationMatrix().eulerAngles(0, 1, 2);
      quat_t initYawQuat;
      initYawQuat = AngleAxisf(0, Vector3f::UnitX())
                  * AngleAxisf(0, Vector3f::UnitY())
                    * AngleAxisf(initEuler[2], Vector3f::UnitZ());

      quat_init_inverse = initYawQuat.inverse();


      // quat_init = quat_t(1,0,0,0);
//      quat_init  = q;
      // quat_init_inverse = quat_init.inverse();
      initialized = true;
      koios->setLogo('G');
    { Threads::Scope scope(state_mtx);
      quat_a = quat_init_inverse * quat_a;
  
      state[6] = quat_a.w();
      state[7] = quat_a.x();
      state[8] = quat_a.y();
      state[9] = quat_a.z();
    }
  }

  if (comms_on > 0)
    while (!ethernet_connected) {threads.delay_us(100);}
  else
    state_d[0] = 1;

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
    quat_t q_measured(-Q3, -Q0, -Q1, -Q2); // Quaternions are a double cover of SO(3).
    quat_a = quat_init_inverse * q_pitch.inverse()*q_measured;

    Serial.print(quat_a.w());     Serial.print(",");
    Serial.print(quat_a.x());     Serial.print(",");
    Serial.print(quat_a.y());     Serial.print(",");
    Serial.print(quat_a.z());     Serial.print(",    ");

    Serial.print(dR);     Serial.print(",");
    Serial.print(dP);     Serial.print(",");
    Serial.print(dY);     Serial.print(",");
    Serial.println();
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
    state[6] = quat_a.w();
    state[7] = quat_a.x();
    state[8] = quat_a.y();
    state[9] = quat_a.z();
    state[10] = foot_state[0];
    state[11] = foot_state[1];
    state[12] = foot_state[2];

    if (quat_a.norm() > 1.05 || quat_a.norm() < 0.95) {
      Serial.print("Exiting because state norm was: ");
      Serial.println(quat_a.norm());
      exitProgram();
    }      
  }

  ////////////// Print the desired state ////////////////////////////
        // Serial.print(state_d[0]); Serial.print(", ");
        // Serial.print(state_d[1]); Serial.print(", ");
        // Serial.print(state_d[2]); Serial.print(", ");
        // Serial.print(state_d[3]); Serial.print(", ");
        // Serial.print(state_d[4]); Serial.print(", ");
        // Serial.print(state_d[5]); Serial.print(", ");
        // Serial.print(state_d[6]); Serial.print(", ");
        // Serial.print(state_d[7]); Serial.print(", ");
        // Serial.print(state_d[8]); Serial.print(", ");
        // Serial.print(state_d[9]);
        // Serial.println();


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
  if (initialized && send_torque > 0) {
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
        elmo.cmdTC(current[0],IDX_K1);
        elmo.cmdTC(current[1],IDX_K2);
        elmo.cmdTC(current[2],IDX_K3);
  }

//  vector_3t omega_a = vector_3t(state[3], state[4], state[5]);

  uint32_t Tc1 = micros();
  // Serial.println(Tc1); // timing
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


  if (ethernet_connected) {
    current_Ethernet_message = millis();
    if (current_Ethernet_message - last_Ethernet_message > TIMEOUT_INTERVAL) {
      Serial.print("Exiting because Ethernet message took: ");
      Serial.println(current_Ethernet_message - last_Ethernet_message);
      exitProgram();
    }
  }
   
  //  Serial.println(Tc1-Tc0);

  // send u4 current command to leg over serial RX/TX between teensy boards
  delay(1);
}
