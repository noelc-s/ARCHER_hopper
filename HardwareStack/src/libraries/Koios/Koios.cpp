#include "Koios.h"

namespace Archer
{
  Koios::Koios(TripENC &tENC, ELMO_CANt4 &elmo):
  tENC_(tENC),
  elmo_(elmo)
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

  int32_t Koios::initComm(int MC){
    setLEDs("1000");
    delay(10);
    digitalWrite(_sOUT,HIGH);
    delay(10);
    Serial.begin(TTL_BAUD);
    delay(10);
    B_PORT.begin(TTL_BAUD);
    delay(10);
    IMU_PORT.begin(115200);
    delay(10);
    if(MC>0){
      rt = elmo_.initK();
    }
    else{
      rt = 1;
    }
    delay(10);
    SPI.begin();
    delay(10);
    tENC_.initEncoders();
    delay(10);
    tENC_.clearEncoderCount(0);
    delay(10);
    return rt;
  }

  int Koios::getIntFromB() {
    int ret_val = 0;
    if (B_PORT.available() > 0) {
      while (B_PORT.available() > 0) {
        ret_val = B_PORT.read();	
      }
      return ret_val;
    } else {
      return -1;
    }
  } 


  void Koios::initSD(){
    Serial.print("Initializing SD card..");
    if(!SD.begin(BUILTIN_SDCARD)){
      Serial.println("SD initialization failed!");
      return; }
    Serial.println("initialization done.");
    delay(100);
  }

  void Koios::initIMU(){
    Serial.println("Initializing VN-100 Serial..");
    IMU_PORT.print("$VNASY,0*XX\r\n");
    delay(500);
    IMU_PORT.print("$VNWRG,05,921600*XX\r\n");
    delay(10);
    IMU_PORT.flush();
    delay(10);
    IMU_PORT.begin(921600);
    delay(475);
    IMU_PORT.print("$VNASY,1*XX\r\n");
    delay(500);
    IMU_PORT.print("$VNASY,0*XX\r\n");
    delay(500);
    IMU_PORT.print("$VNWRG,06,0*XX\r\n");
    delay(500);
    IMU_PORT.print("$VNWRG,75,2,1,01,0030*XX\r\n");
    delay(500);
    IMU_PORT.print("$VNASY,1*XX\r\n");
    delay(100);
    while (IMU_PORT.available() > 0) {
      IMU_PORT.read();
    }
    Serial.println("Initialized VN-100 Serial aka. IMU is working");
    Serial.println(' ');
  }

  void Koios::initKoios1(int MC){
    setLogo('R');
    setLEDs("1000");
    delay(3000);
    rt = initComm(MC);
    if(rt>0){
      setLEDs("0100"); }
    else{
      flashR(10); }
    initIMU();
    delay(10);
  }

  void Koios::initKoios2(){
    delay(250);
    STO(1);
    waitSwitch(1);
    //setSigB(1); //communication with Bia (leg?)
    delay(250);
    rt = motorsOn();
    //Serial.print("1.2");
    delay(5000);
    //resetStates();
    setLEDs("0100");
    //waitSwitch(0);
    //setSigB(0);
    setLogo('A');
    //reverseSig(0);
    delay(5000);
    flashG(2);
    delay(5000);
  }

  void Koios::setLEDs(String val){
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

  void Koios::flashR(int Rep){
    setLEDs("0222");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("1000");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Koios::flashA1(int Rep){
    setLEDs("2022");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("0100");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Koios::flashA2(int Rep){
    setLEDs("2202");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("0010");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Koios::flashG(int Rep){
    setLEDs("2220");
    delay(500);
    for(int i = 0; i < Rep; i++){
      setLEDs("0001");
      delay(500);
      setLEDs("0000");
      delay(500);
    }
  }

  void Koios::setLogo(char Color){
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
    if(Color == 'X'){
      digitalWrite(_relayR,LOW);
      digitalWrite(_relayA,LOW);
      digitalWrite(_relayG,LOW);
    }
  }

  void Koios::setSigB(int val){
    digitalWrite(_sigB,val);
  }

  void Koios::reverseSig(int val){
    if(val==1){
      pinMode(_sigB,OUTPUT);
    }
    else if(val==0){
      pinMode(_sigB,INPUT);
    }
  }

  int Koios::checkSigB(){
    int s1;
    s1 = digitalRead(_sigB);
    return s1;
  }

  int Koios::checkSwitch(){
    int s1;
    s1 = digitalRead(_sIN);
    return s1;
  }

  void Koios::waitSwitch(int val){
    int a;
    int s;
    if(val==1){
      a = 0;
      while(a==0){
        s = checkSwitch();
        if(s==1){
          a = 1;
        }
        delay(50);
      }
    }
    else{
      a = 1;
      while(a==1){
        s = checkSwitch();
        if(s==0){
          a = 0;
        }
        delay(50);
      }
    }
  }

  void Koios::STO(int val){
    digitalWrite(_relayK,val);
  }

  int32_t Koios::motorsOff(int val){
    int32_t rt1,rt2,rt3,ret;
    rt1 = elmo_.motorOff(IDX_K1);
    // delay(10);
    rt2 = elmo_.motorOff(IDX_K2);
    // delay(10);
    rt3 = elmo_.motorOff(IDX_K3);
    // delay(10);
    STO(val);
    ret = rt1*rt2*rt3;
    return ret;
  }

  int32_t Koios::motorsOn(){
    int32_t rt1,rt2,rt3,ret;
    setLEDs("2212");
    rt1 = elmo_.motorOn(IDX_K1);
    delay(10);
    rt2 = elmo_.motorOn(IDX_K2);
    delay(10);
    rt3 = elmo_.motorOn(IDX_K3);
    delay(10);
    ret = rt1*rt2*rt3;
    setLEDs("2202");
    return ret;
  }

  void Koios::resetStates(){
    uint32_t tmp  = micros();
    long     tcnt = tENC_.readEncoder(1);
    _prevT1 = tmp;
    _prevC1 = -tcnt;
    delayMicroseconds(50);
    tmp  = micros();
    tcnt = tENC_.readEncoder(2);
    _prevT2 = tmp;
    _prevC2 = -tcnt;
    delayMicroseconds(50);
    tmp  = micros();
    tcnt = tENC_.readEncoder(3);
    _prevT3 = tmp;
    _prevC3 = tcnt;
  }

  void Koios::updateStates(volatile float &x1,volatile float &v1,volatile float &x2,volatile float &v2,volatile float &x3,volatile float &v3){
    float    pX,dt;
    uint32_t tmp  = micros();
    long     tcnt = tENC_.readEncoder(1);
    _nextT1 = tmp;
    _nextC1 = -tcnt;  // This has been inverted
    _x1 = (_nextC1*2*M_PI)/_resK;
    pX = (_prevC1*2*M_PI)/_resK;
    dt = (_nextT1-_prevT1)/1000000.0;
    _v1 = (_x1-pX)/dt;

    x1 = _x1;
    v1 = _v1;
    _prevT1 = _nextT1;
    _prevC1 = _nextC1;
    delayMicroseconds(50);
    tmp  = micros();
    tcnt = tENC_.readEncoder(2);
    _nextT2 = tmp;
    _nextC2 = -tcnt;  // This has been inverted
    _x2 = (_nextC2*2*M_PI)/_resK;
    pX = (_prevC2*2*M_PI)/_resK;
    dt = (_nextT2-_prevT2)/1000000.0;
    _v2 = (_x2-pX)/dt;

    x2 = _x2;
    v2 = _v2;
    _prevT2 = _nextT2;
    _prevC2 = _nextC2;
    delayMicroseconds(50);
    tmp  = micros();
    tcnt = tENC_.readEncoder(3);
    _nextT3 = tmp;
    _nextC3 = tcnt;
    _x3 = (_nextC3*2*M_PI)/_resK;
    pX = (_prevC3*2*M_PI)/_resK;
    dt = (_nextT3-_prevT3)/1000000.0;
    _v3 = (_x3-pX)/dt;

    x3 = _x3;
    v3 = _v3;
    _prevT3 = _nextT3;
    _prevC3 = _nextC3;
  }

  int Koios::checkFrame(float q0,float q1,float q2,float q3,float dY,float dP,float dR){
    int rt;
    _q0 = q0;
    _q1 = q1;
    _q2 = q2;
    _q3 = q3;
    _dY = dY;
    _dP = dP;
    _dR = dR;
    if(isnormal(q0)){
      rt = 1;
      if(abs(q0)>1.0){
        rt = 0;
      }
    }
    else{
      return 0; }
    if(isnormal(q1)){
      if(abs(q1)>1.0){
        rt = 0;
      }
    }
    else{
      return 0; }
    if(isnormal(q2)){
      if(abs(q2)>1.0){
        rt = 0;
      }
    }
    else{
      return 0; }
    if(isnormal(q3)){
      if(abs(q3)>1.0){
        rt = 0;
      }
    }
    else{
      return 0; }
    if(isnormal(dY)){
      if(dY==0.0){
        rt = 0;
      }
    }
    else{
      return 0; }
    if(isnormal(dP)){
      if(dP==0.0){
        rt = 0;
      }
    }
    else{
      return 0; }
    if(isnormal(dR)){
      if(dR==0.0){
        rt = 0;
      }
    }
    else{
      return 0; }
    return rt;
  }

  int Koios::updateAtt(float &Yaw,float &Pitch,float &Roll){
    float yaw = atan2(2.0*(_q0*_q1 + _q2*_q3),_q3*_q3 + _q0*_q0 - _q1*_q1 - _q2*_q2);
    Pitch = asin(2.0*(_q0*_q2 - _q1*_q3));
    float roll = atan2(2.0*(_q1*_q2 + _q0*_q3),_q2*_q2 + _q3*_q3 - _q0*_q0 - _q1*_q1);

    if(-yaw < 0.0){
      Yaw = -yaw + M_PI; }
    else{
      Yaw = -yaw - M_PI;
    }
    if(roll < 0.0){
      Roll = roll + M_PI; }
    else{
      Roll = roll - M_PI;
    }
    _Yaw = Yaw;
    _Pitch = Pitch;
    _Roll = Roll;

    int rt = 1;
    if(abs(Pitch)>0.25){
      rt = 0;
    }
    else if(abs(Roll)>0.25){
      rt = 0;
    }
    return rt;
  }

  void Koios::delayLoop(uint32_t T0, uint32_t dTdes){
    uint32_t T1 = micros();
    if ((T1 - T0) < dTdes)
      delayMicroseconds(dTdes - T1 + T0);
  }
}
