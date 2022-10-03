/*
   working two way communication ESP to PC sending strings
*/

#include <ESP8266WiFi.h> //lib for esp chipp

const char *ssid = "NETGEAR35";
const char *password = "amberlab";

// Set up TCP server and port number
int port = 8888;
WiFiServer server(port);

// Set static IP address, gateway IP address, subnet
IPAddress local_IP(192, 168, 1, 4); //this will be assigned to the ESP
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

const byte numChars = 40; //size of the message we expect to receive
boolean newDataTeensy = false;
boolean newDataPC = false;

String readValue;

void setup() {

  // Begin USB Serial
  delay(100);                    //give time to open serial
  Serial.begin(115200);        //there is only one serial on esp!!!
  // wait for serial port to connect. Needed for native USB
  while (!Serial) {
    ;
  }
  delay(100);                  //baud rates at both ends must match!!

  // configure static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  // Set to Station mode. ESP8266 connects to desired network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  //beging server
  server.begin();

  //start a diode to be sure all is working
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); //to start the diode on ESP turn it to low

  //  Serial.println("STA ready.");
}

union convert {
  float x[10];
  char b[40];
  unsigned long long int i[5];
} val;

void loop() {
  char receivedCharsTeensy[13*sizeof(float)+8+1];
  char receivedCharsPC[46];
  char sendCharsTeensy[46+1];

  WiFiClient client = server.available();

  if (client) {
    for (int i = 0; i < 60; i++) {
      receivedCharsTeensy[i] = 0b10;
    }
    receivedCharsTeensy[60] = 0b0;
    client.flush();
    delay(2000);
    while (client.connected()) {
      int index = 0;
      while(index < 60) {
        if (Serial.available() > 0) {
          receivedCharsTeensy[index] = Serial.read();
          index++;
        }
      }
      client.print(receivedCharsTeensy); //turned off for debugging
      client.flush();
  
      index = 0;
      while (index < 46) {
        if (client.available() > 0) {
          receivedCharsPC[index] = client.read();
          index++;
        }
      }
      memcpy(sendCharsTeensy, receivedCharsPC, sizeof(sendCharsTeensy));
      sendCharsTeensy[46] = 0b0;
      Serial.print(sendCharsTeensy); //turned off for debugging
    }
    client.stop();
    exit(0);
  }
}
