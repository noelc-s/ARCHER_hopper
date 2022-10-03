#include <CPU.h>

CPU::CPU(DualENC &dENC, Twitter &elmo,  int A):
dENC_(dENC), elmo_(elmo)
{
  int SWCH_OUT = 32;

  pinMode(SWCH_OUT,OUTPUT);
  pinMode(_SWCH_IN,INPUT);
  pinMode(_R1,OUTPUT);
  pinMode(_Y1,OUTPUT);
  pinMode(_G1,OUTPUT);
  pinMode(_G2,OUTPUT);
  pinMode(_Y2,OUTPUT);
  pinMode(_R2,OUTPUT);
  delay(10);
  digitalWrite(SWCH_OUT,HIGH);
  digitalWrite(_R1,HIGH);
  digitalWrite(_Y1,LOW);
  digitalWrite(_G1,LOW);
  digitalWrite(_G2,LOW);
  digitalWrite(_Y2,LOW);
  digitalWrite(_R2,HIGH);
  delay(10);

  switch (A) {
    case 0:
      Q_.reserve(10000);
      break;
    case 1:
      H_.reserve(1000);
      break;
  }
}

CPU::~CPU(void){
  delete[] _traj;
}

void CPU::initComm(){
  Serial.begin(115200);      // Serial to PC
  Serial1.begin(115200);     // Serial to Twitter
  delay(10);
  SPI.begin();
  delay(10);
  SPI.setSCK(27);
  delay(10);
  dENC_.initEncoders();
  delay(10);
  dENC_.clearEncoderCount(0);
  delay(10);
}

void CPU::initSD(){
  Serial.print("Initializing SD card...");
  if(!SD.begin(BUILTIN_SDCARD)){
    Serial.println("SD initialization failed!");
    return; }
  Serial.println("initialization done.");
  delay(100);
}

void CPU::trajSpecs(File specFile, uint32_t &t, int &C){
  _n = specFile.parseInt();
  t  = specFile.parseInt();
  C  = specFile.parseInt();

  _traj = new float[_n];
}

void CPU::createTraj(File dataFile){
  for(int i=0;i<_n;i++){
    _traj[i] = dataFile.parseInt();
  }
}

void CPU::loadTraj(String sFile, String hFile, uint32_t &tf, int &C0){
  File specFile = SD.open(sFile.c_str());
  delay(100);
  trajSpecs(specFile, tf, C0);
  specFile.close();
  delay(100);

  File dataFile = SD.open(hFile.c_str());
  delay(100);
  createTraj(dataFile);
  dataFile.close();
  delay(100);
}

float CPU::findCommand(uint32_t T0){
  uint32_t t = micros()-T0;
  int i = (t+250)/500;
  if(i>(_n-1))
    i = _n-1;
  
  float C = (_traj[i])/100.0;
  return C;
}

void CPU::setLEDs(String val){ // '0' turn off, '1' turn on, '2' no change
  if(val[0]=='1'){
    digitalWrite(_R1,HIGH); }
  else if(val[0]=='0'){
    digitalWrite(_R1,LOW); }
  if(val[1]=='1'){
    digitalWrite(_Y1,HIGH); }
  else if(val[1]=='0'){
    digitalWrite(_Y1,LOW); }
  if(val[2]=='1'){
    digitalWrite(_G1,HIGH); }
  else if(val[2]=='0'){
    digitalWrite(_G1,LOW); }
  if(val[3]=='1'){
    digitalWrite(_G2,HIGH); }
  else if(val[3]=='0'){
    digitalWrite(_G2,LOW); }
  if(val[4]=='1'){
    digitalWrite(_Y2,HIGH); }
  else if(val[4]=='0'){
    digitalWrite(_Y2,LOW); }
  if(val[5]=='1'){
    digitalWrite(_R2,HIGH); }
  else if(val[5]=='0'){
    digitalWrite(_R2,LOW); }
}

int CPU::readSwitch(){
  int s1;
  s1 = digitalRead(_SWCH_IN);
  return s1;
}

void CPU::waitSwitch(int b){
  int a;
  int s;
  a = 0;
  while (a==0) {
    if (b==1) {
      s = readSwitch();
      if (s==1) {
        a = 1;
      }
      delay(100);
    }
    else {
      s = readSwitch();
      if (s==0) {
        a = 1;
      }
      delay(100);
    }
  }
}

void CPU::motorStartup(){
  setLEDs("222010");
  waitSwitch(1);
  elmo_.initTwitter();
  delay(1000);
  elmo_.motorOn();
  setLEDs("222100");
}

void CPU::resetState(int encID){
  uint32_t tmp  = micros();
  long     tcnt = dENC_.readEncoder(encID);
  if(encID==1){
    _prevT1 = tmp;
    _prevC1 = tcnt;
  }
  if(encID==2){
    _prevT2 = tmp;
    _prevC2 = tcnt;
  }
}

void CPU::updateState(int encID,float &x,float&v){
  uint32_t tmp  = micros();
  long     tcnt = dENC_.readEncoder(encID);
  float    pX,dt;
  if(encID==1){
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
  if(encID==2){
    _nextT2 = tmp;
    _nextC2 = tcnt;
    
    _x2 = (_nextC2/_res2)*_D2*M_PI;
    pX  = (_prevC2/_res2)*_D2*M_PI;
    dt  = (_nextT2-_prevT2)/1000000.0;
    _v2 = (_x2-pX)/dt;

    x = _x2;
    v = _v2;

    _prevT2 = _nextT2;
    _prevC2 = _nextC2;
  }
}

void CPU::stateThreshold(float thresh, int a){
  float x, v;
  switch(a){
    case 1:               // "1px"
      x = thresh - 1.0;
      v = 0.0;
      while(x < thresh){
        updateState(1,x,v);
        delayMicroseconds(950); }
      break;
    case 2:               // "1pv"
      x = 0.0;
      v = thresh - 1.0;
      while(v < thresh){
        updateState(1,x,v);
        delayMicroseconds(950); }
      break;
    case 3:               // "1nx"
      x = thresh + 1.0;
      v = 0.0;
      while(x > thresh){
        updateState(1,x,v);
        delayMicroseconds(950); }
      break;
    case 4:               // "1nv"
      x = 0.0;
      v = thresh + 1.0;
      while(v > thresh){
        updateState(1,x,v);
        delayMicroseconds(950); }
      break;
    case 5:               // "2px"
      x = thresh - 1.0;
      v = 0.0;
      while(x < thresh){
        updateState(2,x,v);
        delayMicroseconds(950); }
      break;
    case 6:               // "2pv"
      x = 0.0;
      v = thresh - 1.0;
      while(v < thresh){
        updateState(2,x,v);
        delayMicroseconds(950); }
      break;
    case 7:               // "2nx"
      x = thresh + 1.0;
      v = 0.0;
      while(x > thresh){
        updateState(2,x,v);
        delayMicroseconds(950); }
      break;
    case 8:               // "2nv"
      x = 0.0;
      v = thresh + 1.0;
      while(v > thresh){
        updateState(2,x,v);
        delayMicroseconds(950); }
      break;
  }
}

void CPU::raiseUp(float H0){
  resetState(1);
  resetState(2);
  setLEDs("010222");
  stateThreshold(H0, 5);
  setLEDs("001222");
}

void CPU::delayLoop(uint32_t t0, uint32_t dtDes){
  uint32_t t1 = micros();
  if ((t1 - t0) < dtDes)
    delayMicroseconds(dtDes - t1 + t0);
}

void CPU::fillStates(uint32_t t, float x1, float x2, float c){
  Q_.emplace_back(t, array<float, 3> {x1, x2, c});
}

void CPU::saveStates(String dFile){
  Q_.shrink_to_fit();
  delay(100);
  initSD();
  data_ = SD.open(dFile.c_str(),FILE_WRITE);
  delay(100);
  for (int i = 0; i < Q_.size(); i++) {
    uint32_t ti = Q_[i].first;
    float x1 = Q_[i].second[0];
    float x2 = Q_[i].second[1];
    float c  = Q_[i].second[2];
    String dataLine = String(ti)+','+String(x1, 5)+','+String(x2, 5)+','+String(c, 2);
    data_.println(dataLine);
    delayMicroseconds(1000);
  }
  data_.close();
  delay(100);
}

void CPU::fillHop(uint32_t t, float h0, float dC, float C){
  H_.emplace_back(t, array<float, 3> {h0, dC, C});
}

void CPU::saveHop(String dFile){
  H_.shrink_to_fit();
  delay(100);
  initSD();
  data_ = SD.open(dFile.c_str(),FILE_WRITE);
  delay(100);
  for (int i = 0; i < H_.size(); i++) {
    uint32_t ti = H_[i].first;
    float h0 = H_[i].second[0];
    float dC = H_[i].second[1];
    float C  = H_[i].second[2];
    String dataLine = String(ti)+','+String(h0, 4)+','+String(dC, 2)+','+String(C, 2);
    data_.println(dataLine);
    delayMicroseconds(1000);
  }
  data_.close();
  delay(100);
}

void CPU::saveHeight(String hFile, float dH){
  float x1 = 0.0;
  float v1 = 0.0;
  float x2 = 0.0;
  float v2 = 0.0;
  File height;
  height = SD.open(hFile.c_str(),FILE_WRITE);
  delay(100);
  waitSwitch(0);
  delay(1000);
  updateState(1,x1,v1);
  updateState(2,x2,v2);
  x2 -= dH;
  height.println(x1,5);
  height.println(x2,5);
  height.println(dH,5);
  delay(100);
  height.close();
  delay(100);
  setLEDs("001100");
  delay(2000);
  setLEDs("100001");
}