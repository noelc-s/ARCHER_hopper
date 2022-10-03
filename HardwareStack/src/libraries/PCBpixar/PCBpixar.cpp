#include <PCBpixar.h>

PCBpixar::PCBpixar(DualENC &dENC):
dENC_(dENC)
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
}

void PCBpixar::initComm(){
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

void PCBpixar::initSD(){
  Serial.print("Initializing SD card...");
  if(!SD.begin(BUILTIN_SDCARD)){
    Serial.println("SD initialization failed!");
    return; }
  Serial.println("initialization done.");
  delay(100);
}

void PCBpixar::setLEDs(String val){ // '0' turn off, '1' turn on, '2' no change
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

int PCBpixar::readSwitch(){
  int s1;
  s1 = digitalRead(_SWCH_IN);
  return s1;
}

void PCBpixar::waitSwitch(int b){
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