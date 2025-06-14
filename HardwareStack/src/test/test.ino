#include <SD.h>

File data;
String dFile = "log.txt";

void setup() {
  Serial.begin(115200); //this is for the monitor
  Serial.print("Initializing SD card... ");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println(" initialization done.");
  data = SD.open(dFile.c_str(),FILE_WRITE);
}

void loop() {
  Serial.println("Hello");
  // put your main code here, to run repeatedly:
  data.println("Hello");  
  data.flush();
  delay(100);
}
