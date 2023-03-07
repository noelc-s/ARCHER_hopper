#include <Koios_S.h>

namespace Archer
{
  Koios_S::Koios_S(TripENC &tENC,ELMO_Serial &elmo1,ELMO_Serial &elmo2,ELMO_Serial &elmo3):
  tENC_(tENC),
  elmo1_(elmo1),
  elmo2_(elmo2),
  elmo3_(elmo3)
  {
    pinMode(_sIN,INPUT);
    pinMode(_sOUT,OUTPUT);
    pinMode(_relayK,OUTPUT);
    pinMode(_relayR,OUTPUT);
    pinMode(_relayA,OUTPUT);
    pinMode(_relayG,OUTPUT);
    pinMode(_sigB,OUTPUT);
    pinMode(_R,OUTPUT);
    pinMode(_Y1,OUTPUT);
    pinMode(_Y2,OUTPUT);
    pinMode(_G,OUTPUT);
    delay(10);
    digitalWrite(_sOUT,LOW);
    digitalWrite(_relayK,LOW);
    digitalWrite(_relayR,LOW);
    digitalWrite(_relayA,LOW);
    digitalWrite(_relayG,LOW);
    digitalWrite(_sigB,LOW);
    digitalWrite(_R,LOW);
    digitalWrite(_Y1,LOW);
    digitalWrite(_Y2,LOW);
    digitalWrite(_G,LOW);
    delay(10);
  }

  void Koios_S::initComm(){
    setLEDs("1000");
    delay(10);
    digitalWrite(_sOUT,HIGH);
    delay(10);
    Serial.begin(115200);
    delay(10);
    B_PORT.begin(TTL_BAUD);
    delay(10);
    elmo1_.initELMO();
    delay(10);
    elmo2_.initELMO();
    delay(10);
    elmo3_.initELMO();
    delay(10);
    SPI.begin();
    delay(10);
    tENC_.initEncoders();
    delay(10);
    tENC_.clearEncoderCount(0);
    delay(10);
  }

  void Koios_S::initSD(){
    Serial.print("Initializing SD card...");
    if(!SD.begin(BUILTIN_SDCARD)){
      Serial.println("SD initialization failed!");
      return; }
    Serial.println("initialization done.");
    delay(100);
  }

  void Koios_S::setLEDs(String val){
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

  void Koios_S::setLogo(char Color){
    if(Color == 'R'){
      digitalWrite(_relayA,LOW);
      digitalWrite(_relayG,LOW);
      digitalWrite(_relayR,HIGH);
    }
    if(Color == 'A'){
      digitalWrite(_relayR,LOW);
      digitalWrite(_relayG,LOW);
      digitalWrite(_relayA,HIGH);
    }
    if(Color == 'G'){
      digitalWrite(_relayR,LOW);
      digitalWrite(_relayA,LOW);
      digitalWrite(_relayG,HIGH);
    }
  }

  void Koios_S::setSigB(int val){
    digitalWrite(_sigB,val);
  }

  int Koios_S::checkSwitch(){
    int s1;
    s1 = digitalRead(_sIN);
    return s1;
  }

  void Koios_S::waitSwitch(){
    int a;
    int s;
    a = 0;
    while(a==0){
      s = checkSwitch();
      if(s==1){
        a = 1;
      }
      delay(50);
    }
  }

  void Koios_S::STO(int val){
    digitalWrite(_relayK,val);
  }

  void Koios_S::resetState(int encID){
    uint32_t tmp  = micros();
    long     tcnt = tENC_.readEncoder(encID);
    if(encID==1){
      _prevT1 = tmp;
      _prevC1 = tcnt;
    }
    if(encID==2){
      _prevT2 = tmp;
      _prevC2 = tcnt;
    }
    if(encID==3){
      _prevT3 = tmp;
      _prevC3 = tcnt;
    }
  }

  void Koios_S::updateState(int encID, float &x, float &v){
    uint32_t tmp  = micros();
    long     tcnt = tENC_.readEncoder(encID);
    float    pX,dt;
    if(encID==1){
      _nextT1 = tmp;
      _nextC1 = tcnt;
      
      _x1 = (_nextC1*2*M_PI)/_resK;
      pX = (_prevC1*2*M_PI)/_resK;
      dt = (_nextT1-_prevT1)/1000000.0;
      _v1 = (_x1-pX)/dt;

      x = _x1;
      v = _v1;

      _prevT1 = _nextT1;
      _prevC1 = _nextC1;
    }
    if(encID==2){
      _nextT2 = tmp;
      _nextC2 = tcnt;
      
      _x2 = (_nextC2*2*M_PI)/_resK;
      pX = (_prevC2*2*M_PI)/_resK;
      dt = (_nextT2-_prevT2)/1000000.0;
      _v2 = (_x2-pX)/dt;

      x = _x2;
      v = _v2;

      _prevT2 = _nextT2;
      _prevC2 = _nextC2;
    }
    if(encID==3){
      _nextT3 = tmp;
      _nextC3 = tcnt;
      
      _x3 = (_nextC3*2*M_PI)/_resK;
      pX = (_prevC3*2*M_PI)/_resK;
      dt = (_nextT3-_prevT3)/1000000.0;
      _v3 = (_x3-pX)/dt;

      x = _x3;
      v = _v3;

      _prevT3 = _nextT3;
      _prevC3 = _nextC3;
    }
  }

  void Koios_S::track(int encID, float xd, float vd){
    float x,v,u;
    updateState(encID,x,v);
    u = -_KpK*(x-xd) - _KdK*(v-vd);
    if(encID==1){
      elmo1_.motorCmd(u);
    }
    else if(encID==2){
      elmo2_.motorCmd(u);
    }
    else if(encID==3){
      elmo3_.motorCmd(u);
    }
  }

  void Koios_S::comp(float d){
    // do nothing yet
  }

  void Koios_S::delayLoop(uint32_t T0, uint32_t dTdes){
    uint32_t T1 = micros();
    if ((T1 - T0) < dTdes)
      delayMicroseconds(dTdes - T1 + T0);
  }
}