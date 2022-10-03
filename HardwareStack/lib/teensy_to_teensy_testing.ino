//use for debugging with different teesny
void setup() {
  delay(100); //give time to open and write
  Serial.begin(115200); //this is for the monitor
  Serial1.begin(115200); //chekc if Serial1 baud is the same!
  delay(100);
}

void loop() {
  //send string states, T41 -> ESP8266
  Serial1.write("<from teensy0>");

   //read stringtorques, T41 <- ESP8266
  while (Serial1.available() > 0) {
          char c = Serial1.read();
          Serial.write(c);
        }
  
  delay(1000);
  }
