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

#define MAX_HOP_TIMEOUT 1000000000 // 1000000

DualENC dENC(doub_CS1,doub_CS2);
ELMO_CANt4 elmo;
ControlBia cBia(1.0);
Bia bia(dENC,elmo,cBia);

uint32_t T1, Th;
float tau_max = 10;
int numHops = 50;
volatile uint32_t T0,nF,pF,rt;
volatile float rb,wb,xf,vf,u;
float d0 = 0.015; // spring deflection // 0.15
float b0 = 0.75; // deflection to consider impact
float u0 = -15.5; // offset torque
int h = 0;
float rb0,v0;
volatile bool initialized;

volatile char contact = 0;
char footStateToKoios[1+2*4+2 + 1]; // contact, foot_state, bitAdded, newline
float x = 0;
float v = 0;
float x_meters = 0;
float v_meters = 0;

void exitProgram() {
  rt = elmo.motorOff(IDX_BIA);
  bia.setLEDs("1000");
  while(1) {};
}

void setup() {
  Serial.begin(115200); //this is for the monitor
  delay(500);

  //initBia1
  bia.flashR(1);
  delay(5000);  
  rt = bia.initComm(1);
  delay(1000);
  if(rt>0){
    bia.flashA1(2); }
  else{
    bia.flashR(10); }
  delay(10);
  
  cBia.setTx(tau_max);
  v0 = 100;
  Th = 2000000*numHops + 1000000;

  //initBia2
  delay(250);
  bia.STO(1);
  bia.waitSigK(1);
  delay(250);
  elmo.motorOn(IDX_BIA);
  delay(5000);
  bia.resetState(1);
  bia.resetState(2);
  bia.findZero();
  bia.setLEDs("0100");
  bia.waitSigK(0);
  bia.reverseSig(1);
  bia.setSigK(0);
  delay(5000);
  bia.flashG(2);
  delay(7000);

  
  cBia.getRB0(rb0);
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
  while(1) {
    {std::lock_guard<std::mutex> lck(state_mtx);
    footStateToKoios[0] = contact;
    x_meters = x/1000;
    v_meters = v/1000;
    memcpy(footStateToKoios+1, &x_meters, 4);
    memcpy(footStateToKoios+5, &v_meters, 4);
    }
    Serial.print((float)footStateToKoios[0]); Serial.print(";  ");
    Serial.print(x_meters); Serial.print(";  ");
    Serial.print(v_meters); Serial.println(";  ");

    for (int i = 0; i < 2; i++) {
      byte oneAdded = 0b00000001;
      for (int j = 1; j < 8; j++){
        if (footStateToKoios[i*7+(j-1)] == 0b00000000) {
          footStateToKoios[i*7+(j-1)] = 0b00000001;
          oneAdded += (1 << (8-j));
        }
      }
      memcpy(&footStateToKoios[9+i], &oneAdded, 1);
    }
    footStateToKoios[11] = 0b0;

    K_PORT.write(footStateToKoios);
    K_PORT.flush();
    threads.delay_us(1000);
  }
}

void loop() {
  // Do not do leg control
//  exitProgram();
  
  compPhase();    //
  {std::lock_guard<std::mutex> lck(state_mtx);
  contact = 1;
  }
  releasePhase(); // control to rb = 0, end at xf = 0
  {std::lock_guard<std::mutex> lck(state_mtx);
  contact = 0;
  }
  h++;
  if(h>=numHops){
    exitProgram();
  }
}

void compPhase() {
  float theta,omega,xfs;
  uint32_t Tc0 = micros();
  uint32_t Ts0,dTs;
  int i = 0;
  Serial.println("------------Comp Phase---------------");
  
  bia.updateState(1,theta, omega); // theta and omega are motor angle and vel
  float theta_0 = theta;
  
  while(i<4) {
    contact = 0;
    // WIFI ESTOP: 
    if (bia.checkSigK() == 1) {
      exitProgram();
    }
    // Update states:
    bia.updateState(1,theta, omega); // theta and omega are motor angle and vel
    {std::lock_guard<std::mutex> lck(state_mtx);
    bia.updateState(2,x, v); // x and v are spring deflection in mm     
    }

    float kp = 0.4;
    float kd = 0.04;
    float x_star = 30;
    u = -kp*(x - x_star) - kd*v;
//    Serial.println(x);
    if (theta-theta_0 >= 0.04) {
      exitProgram();
    }
    rt = elmo.sendTC(-u+u0,4);

    if(i==0){                 // Waiting for compression
      if(x>8.0){             // Check for enough deflection
        i = 1;
      }
    }
    if(i==1){                 // waiting to slow down
      if(abs(v) < 20){          // Check for slow movement
        i = 2;
        Ts0 = micros();
      }
    }
    if(i==2){                 // waiting for settling
      dTs = micros() - Ts0;
      if(abs(v) > 20){          // Check if it sped up
        i = 1;
      }
      else if(dTs > 50000){   // Check if settle time reached (50000)
        i   = 3;
        xfs = x;
      }
    }
    if(i==3){                 // Check if impact occured 
      if((x-xfs) > 0.75){
        i = 4;
      }
    }
  }
}

//void compPhase(){
//  uint32_t Tc0 = micros();
//  uint32_t Ts0,dTs;
//  float x,v,r,w,xfs,U;
//  int i = 0;
//  Serial.println("------------Comp Phase---------------");
//  while(i<4){
//        // WIFI ESTOP: 
//    if (bia.checkSigK() == 1) {
//      exitProgram();
//    }
//    if((micros()-Tc0)>MAX_HOP_TIMEOUT){ // if in comp phase for too long, move-onto release and end experiment
//      h = numHops+1;
//      i = 4;
//      break;
//    }
//    bia.updateState(1,r,w);
//    bia.trackU0(d0,u0,x,v,U);
//    rb = r;
//    wb = w;
//    xf = x;
//    vf = v;
////    Serial.print("Spring Deflection: "); Serial.print(xf);
////    Serial.print("  Spring Velocity: "); Serial.print(vf); Serial.println();
//Serial.println(i);
//    u  = U;
//    if(i==0){                 // Waiting for compression
//      if(xf>8.0){             // Check for enough deflection
//        i = 1;
//      }
//    }
//    if(i==1){                 // waiting to slow down
//      if(abs(vf)<v0){          // Check for slow movement
//        i = 2;
//        Ts0 = micros();
//      }
//    }
//    if(i==2){                 // waiting for settling
//      dTs = micros() - Ts0;
//      if(abs(vf)>v0){          // Check if it sped up
//        i = 1;
//      }
//      else if(dTs > 50000){   // Check if settle time reached (50000)
//        i   = 3;
//        xfs = xf;
//      }
//    }
//    if(i==3){                 // Check if impact occured 
//      if((xf-xfs) > b0){
//        i = 4;
//      }
//    }
//    delayLoop(T1,1000);
//    T1 = micros();
//    nF++;
//  }
//}
//
void releasePhase(){
  uint32_t Tr0 = micros();
  float r,w,up,ud;
  int i = 0;
  Serial.println("------------Release Phase---------------");
  while(i<1){
    // WIFI ESTOP: 
    if (bia.checkSigK() == 1) {
      exitProgram();
    }
    if((micros()-Tr0)>MAX_HOP_TIMEOUT){ // if in release phase for too long, exit release phase and end experiment
      h = numHops+1;
      i = 1;
      break;
    }
    {std::lock_guard<std::mutex> lck(state_mtx);
    bia.testPDb(r,w,x,v,up,ud);
    }
    rb = r;
    wb = w;
    vf = v;
    u  = up + ud;
    if(x<0.5){
      i = 1;
    }
    delayLoop(T1,1000);
    T1 = micros();
    nF++;
  }
}

void delayLoop(uint32_t T1,uint32_t dT){
  uint32_t T2 = micros();
  if((T2-T1)<dT){
    uint32_t a = dT + T1 - T2;
    threads.delay_us(a);
  }
}
