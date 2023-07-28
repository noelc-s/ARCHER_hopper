#include <Archer_Config.h>
#include <TripENC.h>
#include <ELMO_CANt4.h>
#include <Koios.h>
#include <SPI.h>
#include <SD.h>
#include <TeensyThreads.h>
#include <ArduinoEigen.h>
#include <Flywheel.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace Archer;
using namespace Eigen;

Koios* koios;

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
        aRecord[charNum] = '\0';  //terminate the record
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

  Serial.println("Loaded Parameters: ");
  Serial.print("kp_y = ");     Serial.println(kp_y);
  Serial.print("kp_rp = ");    Serial.println(kp_rp);
  Serial.print("kd_y = ");     Serial.println(kd_y);
  Serial.print("kd_rp = ");    Serial.println(kd_rp);
  Serial.print("r_offset = "); Serial.println(r_offset);
  Serial.print("p_offset = "); Serial.println(p_offset);

  //=================================================

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
//============= FUNCTIONS =========
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

// for parsing SD card txt files, line-by-line
void parseRecord(byte index){
  char * ptr;
  ptr = strtok(aRecord, " = ");  //find the " = "
  ptr = strtok(NULL, ""); //get remainder of text
  strcpy(parameterArray[index], ptr + 2); //skip 2 characters and copy to array
}  

//==========================LOOP=============

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
      // Threads::Scope scope2(foot_state_mtx);
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
    // Threads::Scope scope2(foot_state_mtx);
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
  vector_3t omega_a = vector_3t(state[3], state[4], state[5]);

  quat_d = quat_t(state_d[0], state_d[1], state_d[2], state_d[3]);
  omega_d = vector_3t(state_d[4], state_d[5], state_d[6]);
  tau_ff = vector_3t(state_d[7], state_d[8], state_d[9]);
  ///// If you want to debug the low level controller, comment the above three lines, and uncomment the below three
  //  quat_t quat_d = quat_t(1,0,0,0);
  //  vector_3t omega_d = vector_3t(0,0,0);
  //  vector_3t tau_ff = vector_3t(0,0,0);

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
    elmo.cmdTC(current[0], IDX_K1);
    elmo.cmdTC(current[1], IDX_K2);
    elmo.cmdTC(current[2], IDX_K3);
    ////// If you want to set the foot current to zero for debugging, comment the above three lines and uncomment the below three
    //        current[0] = 0;
    //        current[1] = 0;
    //        current[2] = 0;
  }

  writeData(data, micros(), quat_a, quat_d, omega_a, omega_d, foot_state, current);


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
