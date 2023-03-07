#include <Twitter.h>

Twitter::Twitter(){
  // Does nothing here
}

void Twitter::initTwitter(){
  Serial.println("Initializing the Twitter");
  Serial1.print("MO=0;");
  delayMicroseconds(5000);
  Serial1.print("EO=0;");
  delayMicroseconds(5000);
  Serial1.print("UM=1;");
  delayMicroseconds(5000);
  Serial1.print("RM=0;");
  delayMicroseconds(5000);
}

void Twitter::motorOn(){
  Serial.println("Turning Motor On");
  Serial1.print("MO=1;");
  delayMicroseconds(5000);
  Serial1.print("TC=0;");
  delayMicroseconds(5000);
}

void Twitter::motorOff(){
  Serial.println("Turning Motor Off");
  Serial1.print("TC=0;");
  delayMicroseconds(5000);
  Serial1.print("MO=0;");
  delayMicroseconds(5000);
}

void Twitter::motorCmd(float C){
  String msg;
  msg = "TC=" + String(C) + ";";
  // Serial.print("Sending  ");
  // Serial.println(msg);
  Serial1.print(msg);
}