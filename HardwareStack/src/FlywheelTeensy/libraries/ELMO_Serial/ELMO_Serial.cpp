#include "ELMO_Serial.h"

namespace Archer
{
  ELMO_Serial::ELMO_Serial(HardwareSerial &port):
  baud_(ELMO_BAUD),
  recTimeout_(R_TIMEOUT),
  port_(port)
  {}

  void ELMO_Serial::initELMO(){
    Serial.println("Initializing the Twitter");
    port_.begin(baud_);
    port_.setTimeout(recTimeout_);
    delay(50);
    port_.print("MO=0;");
    delayMicroseconds(5000);
    port_.print("EO=0;");
    delayMicroseconds(5000);
    port_.print("UM=1;");
    delayMicroseconds(5000);
    port_.print("RM=0;");
    delayMicroseconds(5000);
  }

  void ELMO_Serial::motorOn(){
    Serial.println("Turning Motor On");
    port_.print("MO=1;");
    delayMicroseconds(5000);
    port_.print("TC=0;");
    delayMicroseconds(5000);
  }

  void ELMO_Serial::motorInitOn(){
    Serial.println("Turning Motor On");
    port_.print("MO=1;");
    delay(10000);
    port_.print("TC=0;");
    delayMicroseconds(5000);
  }

  void ELMO_Serial::motorOff(){
    Serial.println("Turning Motor Off");
    port_.print("TC=0;");
    delayMicroseconds(5000);
    port_.print("MO=0;");
    delayMicroseconds(5000);
  }

  void ELMO_Serial::motorCmd(float C){
    String msg;
    msg = "TC=" + String(C) + ";";
    // Serial.print("Sending  ");
    // Serial.println(msg);
    port_.print(msg);
  }

  void ELMO_Serial::motorCmdSat(float C, float S){
    if(C>S){
      C = S; }
    else if(C<-S){
      C = -S; }
    String msg;
    msg = "TC=" + String(C) + ";";
    // Serial.print("Sending  ");
    // Serial.println(msg);
    port_.print(msg);
  }
}