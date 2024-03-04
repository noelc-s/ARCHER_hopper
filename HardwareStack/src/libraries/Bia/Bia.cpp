#include "Bia.h"

namespace Archer
{

  Bia::Bia(DualENC &dENC, ELMO_CANt4 &elmo, ControlBia &cBia):
  dENC_(dENC),
  elmo_(elmo),
  cBia_(cBia)
  {
    pinMode(_sigK,INPUT);
    pinMode(_relayB,OUTPUT);
    pinMode(_R,OUTPUT);
    pinMode(_Y1,OUTPUT);
    pinMode(_Y2,OUTPUT);
    pinMode(_G,OUTPUT);
    delay(10);
    digitalWrite(_relayB,LOW);
    digitalWrite(_R,LOW);
    digitalWrite(_Y1,LOW);
    digitalWrite(_Y2,LOW);
    digitalWrite(_G,LOW);
    delay(10);
    _rb0 = 0.0;
  }

  void Bia::exitProgram() {
    releaseMotor();
    rt = elmo_.motorOff(IDX_BIA);
    setLEDs("1000");
    while(1) {};
  }

  void Bia::releaseMotor(){
  float r,w,up,ud,x,v;
  int i = 0;
  float u;
  // Serial.println("------------Releasing Motor---------------");
  while(i<1){
    updateState(1,r,w);
    updateState(2,x,v);
    cBia_.releaseFoot(up,ud,r,w);
    u  = up + ud;
    rt = sendSafeTorque(r, u);
    if(x<0.5){
      i = 1;
    }
    delay(1);
  }
}

  int32_t Bia::initComm(int MC){
    setLEDs("1000");
    delay(10);
    Serial.begin(TTL_BAUD);
    delay(10);
    K_PORT.begin(TTL_BAUD);
    delay(10);
    if(MC>0){
      rt = elmo_.initB();
    }
    else{
      rt = 1;
    }
    delay(10);
    SPI.begin();
    delay(10);
    dENC_.initEncoders();
    delay(10);
    dENC_.clearEncoderCount(0);
    delay(10);
    return rt;
  }

  void Bia::sendIntToK(int send_val) {
    K_PORT.write(send_val);
  }

  void Bia::sendCharArrToK(char* send_chars) {
    K_PORT.write(send_chars);
    K_PORT.flush();
  }

  void Bia::initSD(){
    // Serial.print("Initializing SD card...");
    if(!SD.begin(BUILTIN_SDCARD)){
      // Serial.println("SD initialization failed!");
      return; }
    // Serial.println("initialization done.");
    delay(100);
  }

  void Bia::initBia1(int MC){
    flashR(1);
    delay(5000);
    rt = initComm(MC);
    delay(1000);
    if(rt>0){
      flashA1(2); }
    else{
      flashR(10); }
    initSD();
    delay(10);
  }

  void Bia::initBia2(){
    delay(250);
    STO(1);
    waitSigK(1);
    delay(250);
    elmo_.motorOn(IDX_BIA);
    delay(5000);
    resetState(1);
    resetState(2);
    findZero();
    setLEDs("0100");
    waitSigK(0);
    reverseSig(1);
    setSigK(0);
    delay(5000);
    flashG(2);
    delay(5000);
  }

  void Bia::setLEDs(String val){
    if(val[0]=='1'){
      digitalWrite(_R,HIGH); }
    else if(val[0]=='0'){
      digitalWrite(_R,LOW); }
    if(val[1]=='1'){
      digitalWrite(_Y1,HIGH); }
    else if(val[1]=='0'){
      digitalWrite(_Y1,LOW); }
    if(val[2]=='1'){
      digitalWrite(_Y2,HIGH); }
    else if(val[2]=='0'){
      digitalWrite(_Y2,LOW); }
    if(val[3]=='1'){
      digitalWrite(_G,HIGH); }
    else if(val[3]=='0'){
      digitalWrite(_G,LOW); }
  }

  void Bia::flashR(int Rep){
    setLEDs("0222");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("1000");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Bia::flashA1(int Rep){
    setLEDs("2022");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("0100");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Bia::flashA2(int Rep){
    setLEDs("2202");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("0010");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Bia::flashG(int Rep){
    setLEDs("2220");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("0001");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  int Bia::checkSigK(){
    int s1;
    s1 = digitalRead(_sigK);
    return s1;
  }

  void Bia::reverseSig(int val){
    if(val==1){
      pinMode(_sigK,OUTPUT);
    }
    else if(val==0){
      pinMode(_sigK,INPUT);
    }
  }

  void Bia::setSigK(int val){
    digitalWrite(_sigK,val);
  }

  void Bia::waitSigK(int trigger){
    int a;
    int s;
    if(trigger==1){
      a = 0;
      while(a==0){
        s = checkSigK();
        if(s==1) {
          a = 1;
        }
        delay(50);
      }
    }
    else{
      a = 1;
      while(a==1){
        s = checkSigK();
        if(s==0) {
          a = 0;
        }
        delay(50);
      }
    }
  }

  void Bia::STO(int val){
    digitalWrite(_relayB,val);
  }

  void Bia::resetState(int encID){
    uint32_t tmp  = micros();
    long     tcnt = dENC_.readEncoder(encID);
    if(encID==1){
      _prevTb = tmp;
      _prevCb = -tcnt;      // This was inverted
    }
    if(encID==2){
      _prevTf = tmp;
      _prevCf = tcnt;
    }
  }

  void Bia::updateState(int encID, float &x, float &v){
    uint32_t tmp  = micros();
    long     tcnt = dENC_.readEncoder(encID);
    float    pX,dt;
    if(encID==1){
      _nextTb = tmp;
      _nextCb = -tcnt;      // This was inverted
      
      _x = (_nextCb*2*M_PI)/_resB;
      pX = (_prevCb*2*M_PI)/_resB;
      dt = (_nextTb-_prevTb)/1000000.0;
      _v = (_x-pX)/dt;

      x = _x;
      v = _v;

      _prevTb = _nextTb;
      _prevCb = _nextCb;
    }
    if(encID==2){
      _nextTf = tmp;
      _nextCf = tcnt;
      
      _xF = _nextCf/_resF;
      pX  = _prevCf/_resF;
      dt  = (_nextTf-_prevTf)/1000000.0;
      _vF = (_xF-pX)/dt;

      x = _xF;
      v = _vF;

      _prevTf = _nextTf;
      _prevCf = _nextCf;
    }
  }

  int32_t Bia::sendSafeTorque(float xb, float u) {
    if (xb > theta_max || xb < theta_min) {
      Serial.print("xb was too large: ");
      Serial.print(xb);
      Serial.println(". Exiting");
      exitProgram();
    } 
    return elmo_.sendTC(u,IDX_BIA);
  }

  void Bia::findZero(){
    // Serial.println("Finding Zero");
    float xb,vb,xf,vf;
    float th1 = 0.02;
    float th2 = 0.01;
    float u   = 0.0;
    int i = 0;
    // Serial.println("Pulling in");
    while(i<1){
      u = u - 0.001;
      // Serial.print("U: ");
      // Serial.println(u);
      if (abs(u) > 10) {
        // Serial.println("Error. Required too much torque in initialization. Exiting.");
        exitProgram();
      }
      updateState(1,xb,vb);
      updateState(2,xf,vf);
      sendSafeTorque(xb, u);
      if(xf>th1){
        i = 1;
        cBia_.logZero(xb,1);
      }
    }
    // Serial.println("Deflection Registered.");
    // Serial.println("Releasing");
    while(i<2){
      u = u + 0.001;
      if (u > 6) {
        // Serial.println("Error. Torque went above 1. Exiting.");
        exitProgram();
      }
      updateState(1,xb,vb);
      updateState(2,xf,vf);
      sendSafeTorque(xb, u);
      if(xf<th2){
        i = 2;
        cBia_.logZero(xb,2);
      }
    }
    // Serial.println("Done.");
  }

  void Bia::trackPID0(float d0,float &x,float &u0){
    float rb,wb,u;
    updateState(1,rb,wb);
    cBia_.inputPID0(u,d0,rb,wb,0.0);
    rt = sendSafeTorque(rb, u);
    // rt = elmo_.sendTC(u,4);
    x  = rb-_rb0;
    u0 = u;
  }

  void Bia::trackU0(float d0,float u0,float &xf,float &vf,float &u){
    float up,ud;
    float rb,wb;
    updateState(2,xf,vf);
    cBia_.testU0(up,ud,d0,xf,vf);
    u  = u0 + up + ud;
    updateState(1,rb,wb);
    rt = sendSafeTorque(rb, u);
    // rt = elmo_.sendTC(u,4);
  }

  void Bia::testPID0(float d0,float &x,float &u0,float &up,float &ud,float &ui){
    float rb,wb,u;
    updateState(1,rb,wb);
    cBia_.testPID0(u0,up,ud,ui,d0,rb,wb);
    u  = u0 + up + ud + ui;
    rt = sendSafeTorque(rb, u);
    // rt = elmo_.sendTC(u,4);
    x  = rb;
  }

  void Bia::testPDb(float &rb,float &wb,float &xf,float &vf,float &up,float &ud){
    float u;
    updateState(1,rb,wb);
    updateState(2,xf,vf);
    cBia_.testPDb(up,ud,rb,wb);
    u  = up + ud;
    rt = sendSafeTorque(rb, u);
    // rt = elmo_.sendTC(u,4);
  }

  void Bia::testPDf(float d0,float &rb,float &xf,float &u0,float &up,float &ud){
    float wb,vf,u;
    updateState(1,rb,wb);
    updateState(2,xf,vf);
    cBia_.testPDf(u0,up,ud,d0,xf,vf);
    u  = u0 + up + ud;
    rt = sendSafeTorque(rb, u);
    // rt = elmo_.sendTC(u,4);
  }

  void Bia::testU0(float d0,float &xf,float u0){
    float up,ud,vf,u,wb,rb;
    updateState(1,rb,wb);
    updateState(2,xf,vf);
    cBia_.testU0(up,ud,d0,xf,vf);
    u  = u0 + up + ud;
    rt = sendSafeTorque(rb, u);
    // rt = elmo_.sendTC(u,4);
  }

  void Bia::updateRB0(){
    cBia_.getRB0(_rb0);
  }

  void Bia::delayLoop(uint32_t T0, uint32_t dTdes){
    uint32_t T1 = micros();
    if ((T1 - T0) < dTdes)
      delayMicroseconds(dTdes - T1 + T0);
  }

  void Bia::runSin() {
    float theta;
    float omega;
    float u = 0.0;
    int direction = 1;
    // Serial.println("Running Sawtooth pattern.");
    while(1) {
      updateState(1,theta, omega);
      if (abs(theta) > 0.1) {
        exitProgram();
      }
      if (direction == 1) {
        u -= 0.01;
      } else {
        u += 0.01;
      }
      if (abs(u) >= 1.0) {
        direction = -direction;
      }
      // Serial.println(u);
      sendSafeTorque(theta, u);
      // elmo_.sendTC(u,IDX_BIA);
    }
  }
}
