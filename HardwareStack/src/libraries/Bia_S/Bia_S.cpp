#include <Bia_S.h>

namespace Archer
{
  Bia_S::Bia_S(DualENC &dENC,ELMO_Serial &elmo):
  dENC_(dENC), elmo_(elmo)
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
  }

  void Bia_S::initComm(){
    setLEDs("1000");
    delay(10);
    Serial.begin(115200);
    delay(10);
    K_PORT.begin(Serial_BAUD);
    delay(10);
    elmo_.initELMO();
    delay(10);
    SPI.begin();
    delay(10);
    dENC_.initEncoders();
    delay(10);
    dENC_.clearEncoderCount(0);
    delay(10);
  }

  void Bia_S::initSD(){
    Serial.print("Initializing SD card...");
    if(!SD.begin(BUILTIN_SDCARD)){
      Serial.println("SD initialization failed!");
      return; }
    Serial.println("initialization done.");
    delay(100);
  }

  void Bia_S::setLEDs(String val){
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

  int Bia_S::checkSigK(){
    int s1;
    s1 = digitalRead(_sigK);
    return s1;
  }

  void Bia_S::waitSigK(int trigger){
    int a;
    int s;
    a = 0;
    while (a==0) {
      if (trigger==1) {
        s = checkSigK();
        if (s==1) {
          a = 1;
        }
        delay(100);
      }
      else {
        s = checkSigK();
        if (s==0) {
          a = 1;
        }
        delay(100);
      }
    }
  }

  void Bia_S::STO(int val){
    digitalWrite(_relayB,val);
  }

  void Bia_S::resetState(int encID){
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

  void Bia_S::updateState(int encID, float &x, float &v){
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

  void Bia_S::track(float xd, float vd, int load){
    float x,v,u;
    updateState(1,x,v);
    u = -_KpB*(x-xd) - _KdB*(v-vd);
    elmo_.motorCmd(u);
  }

  void Bia_S::comp(float d){
    // do nothing yet
  }

  void Bia_S::delayLoop(uint32_t T0, uint32_t dTdes){
    uint32_t T1 = micros();
    if ((T1 - T0) < dTdes)
      delayMicroseconds(dTdes - T1 + T0);
  }
}