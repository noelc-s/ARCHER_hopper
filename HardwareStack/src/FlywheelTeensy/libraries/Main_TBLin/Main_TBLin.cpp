#include <Main_TBLin.h>

Main_TBLin::Main_TBLin(DualENC &dENC,Twitter &elmo,PCBpixar &pcb):
dENC_(dENC), elmo_(elmo), pcb_(pcb)
{
  // Do nothing here
}

void Main_TBLin::resetState(){
  uint32_t tmp  = micros();
  long     tcnt = dENC_.readEncoder(1);

  _prevT1 = tmp;
  _prevC1 = tcnt;
}

void Main_TBLin::updateState(float &x,float&v){
  uint32_t tmp  = micros();
  long     tcnt = dENC_.readEncoder(1);
  float    pX,dt;

  _nextT1 = tmp;
  _nextC1 = tcnt;
  
  _x1 = _nextC1/_res1;
  pX  = _prevC1/_res1;
  dt  = (_nextT1-_prevT1)/1000000.0;
  _v1 = (_x1-pX)/dt;

  x = _x1;
  v = _v1;

  _prevT1 = _nextT1;
  _prevC1 = _nextC1;
}

void Main_TBLin::delayLoop(uint32_t T0, uint32_t dtDes){
  uint32_t T1 = micros();
  if ((T1 - T0) < dtDes)
    delayMicroseconds(dtDes - T1 + T0);
}

void Main_TBLin::motorStartup(){
  pcb_.setLEDs("222010");
  pcb_.waitSwitch(1);
  elmo_.initTwitter();
  delay(1000);
  elmo_.motorOn();
  pcb_.setLEDs("222100");
  delay(1000);
}

void Main_TBLin::sendCmd(float desI, float Imax){
  float I;
  if(desI>Imax){
    I = Imax;
  }
  else if(desI<-Imax){
    I = -Imax;
  }
  else{
    I = desI;
  }
  elmo_.motorCmd(I);
}

void Main_TBLin::desTraj(float t, float &yDes, float &dyDes, float &ddyDes){
  float A  = 0.05;    // Sinusoid Amplitude         [m]
  float B  = 0.075;   // Center of range of motion  [m]
  float w  = 6.28;    // Sinusoid frequency         [rad/s]

  yDes   = A*cos(w*t) + B;
  dyDes  = -A*w*sin(w*t);
  ddyDes = -A*w*w*cos(w*t);
}

void Main_TBLin::initPos(float &x, float &v){
  uint32_t T,T0;
  float yd=0.0,dyd=0.0,ddyd=0.0;
  float t,y,dy,ddy,I0,I1,I;
  float A  = (yd-x)/2;
  float B  = (yd+x)/2;
  float w  = 3.14;
  float kp = -75.0;
  float kd = -10.0;
  pcb_.setLEDs("010222");
  updateState(x,v);
  desTraj(0.0, yd,dyd,ddyd);
  T0 = micros();
  T = T0;
  t = 0;
  while(t<(1.0)){
    y   = -A*cos(w*t) + (abs(A)/A)*B;
    dy  = A*w*sin(w*t);
    ddy = A*w*w*cos(w*t);
    updateState(x,v);

    I0 = (_M*ddy + dy)/_kf;
    I1 = kp*(x-y) + kd*(v-dy);
    I  = I0 + I1;
    sendCmd(I,5.0);

    delayLoop(T,2000);
    T = micros();
    t = (T - T0)/1000000.0;
  }
  while(t<2.0){
    updateState(x,v);
    I = kp*(x-yd) + kd*v;
    sendCmd(I,5.0);

    delayLoop(T,2000);
    T = micros();
    t = (T - T0)/1000000.0;
  }
  elmo_.motorCmd(0.0);
}

void Main_TBLin::trackTraj(uint32_t T0){
  float yd=0.0,dyd=0.0,ddyd=0.0,x=0.0,v=0.0;
  float I0,I1,I;
  float kp = -75.0;
  float kd = -10.0;

  float t = (micros() - T0)/1000000.0;
  desTraj(t,yd,dyd,ddyd);
  updateState(x,v);

  I0 = (_M*ddyd + dyd)/_kf;
  I1 = kp*(x-yd) + kd*(v-dyd);
  I  = I0 + I1;
  sendCmd(I,5.0);
}

void Main_TBLin::stopMotion(){
  elmo_.motorOff();
  pcb_.setLEDs("222001");
  delay(1000);
}