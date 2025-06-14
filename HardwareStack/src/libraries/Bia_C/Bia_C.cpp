#include "Bia_C.h"

namespace Archer
{
  Bia_C::Bia_C(DualENC &dENC, ELMO_CANt4 &elmo, ControlBia &cBia):
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

  int32_t Bia_C::initComm(int MC){
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

  void Bia_C::initSD(){
    Serial.print("Initializing SD card...");
    if(!SD.begin(BUILTIN_SDCARD)){
      Serial.println("SD initialization failed!");
      return; }
    Serial.println("initialization done.");
    delay(100);
  }

  void Bia_C::initBia1(int MC){
    flashR(1);
    delay(6000);
    rt = initComm(MC);
    delay(1000);
    if(rt>0){
      setLEDs("0100"); }
    else{
      flashR(10); }
    initSD();
    delay(10);
  }

  void Bia_C::initBia2(){
    delay(250);
    STO(1);
    delay(250);
    elmo_.motorOn(IDX_BIA);
    delay(7500);
    setLEDs("0000");
    delay(500);
    setLEDs("0100");
    resetState(1);
    resetState(2);
  }

  void Bia_C::setLEDs(String val){
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

  void Bia_C::flashR(int Rep){
    setLEDs("0222");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("1000");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Bia_C::flashA1(int Rep){
    setLEDs("2022");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("0100");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Bia_C::flashA2(int Rep){
    setLEDs("2202");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("0010");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Bia_C::flashG(int Rep){
    setLEDs("2220");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("0001");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  int Bia_C::checkSigK(){
    int s1;
    s1 = digitalRead(_sigK);
    return s1;
  }

  void Bia_C::waitSigK(int trigger){
    int a;
    int s;
    a = 0;
    while (a==0) {
      if (trigger==1) {
        s = checkSigK();
        if (s==1) {
          a = 1;
        }
        delay(50);
      }
      else {
        s = checkSigK();
        if (s==0) {
          a = 1;
        }
        delay(50);
      }
    }
  }

  void Bia_C::STO(int val){
    digitalWrite(_relayB,val);
  }

  void Bia_C::resetState(int encID){
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

  void Bia_C::updateState(int encID, float &x, float &v){
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

  void Bia_C::trackPID0(float d0,float &x,float &u0){
    float rb,wb,u;
    updateState(1,rb,wb);
    cBia_.inputPID0(u,d0,rb,wb,0.0);
    rt = elmo_.sendTC(u,4);
    x  = rb-_rb0;
    u0 = u;
  }

  void Bia_C::trackU0(float d0,float u0,float &xf,float &vf,float &u){
    float up,ud;
    updateState(2,xf,vf);
    cBia_.testU0(up,ud,d0,xf,vf);
    u  = u0 + up + ud;
    rt = elmo_.sendTC(u,4);
  }

  void Bia_C::testPID0(float d0,float &x,float &u0,float &up,float &ud,float &ui){
    float rb,wb,u;
    updateState(1,rb,wb);
    cBia_.testPID0(u0,up,ud,ui,d0,rb,wb);
    u  = u0 + up + ud + ui;
    rt = elmo_.sendTC(u,4);
    x  = rb;
  }

  void Bia_C::testPDb(float &rb,float &xf,float &up,float &ud){
    float wb,vf,u;
    updateState(1,rb,wb);
    updateState(2,xf,vf);
    cBia_.testPDb(up,ud,rb,wb);
    u  = up + ud;
    rt = elmo_.sendTC(u,4);
  }

  void Bia_C::testPDf(float d0,float &rb,float &xf,float &u0,float &up,float &ud){
    float wb,vf,u;
    updateState(1,rb,wb);
    updateState(2,xf,vf);
    cBia_.testPDf(u0,up,ud,d0,xf,vf);
    u  = u0 + up + ud;
    rt = elmo_.sendTC(u,4);
  }

  void Bia_C::testU0(float d0,float &xf,float u0){
    float up,ud,vf,u;
    updateState(2,xf,vf);
    cBia_.testU0(up,ud,d0,xf,vf);
    u  = u0 + up + ud;
    rt = elmo_.sendTC(u,4);
  }

  void Bia_C::updateRB0(){
    cBia_.getRB0(_rb0);
  }

  void Bia_C::delayLoop(uint32_t T0, uint32_t dTdes){
    uint32_t T1 = micros();
    if ((T1 - T0) < dTdes)
      delayMicroseconds(dTdes - T1 + T0);
  }
}