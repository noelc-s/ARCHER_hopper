#include <QNEthernet.h>

using namespace qindesign::network;

// --------------------------------------------------------------------------
//  Configuration
// --------------------------------------------------------------------------

constexpr uint16_t kPort = 4333;  // Chat port

// --------------------------------------------------------------------------
//  Program State
// --------------------------------------------------------------------------

// UDP port.
EthernetUDP udp;
bool read_packet = false;

// --------------------------------------------------------------------------
//  Main Program
// --------------------------------------------------------------------------

// unsigned long t1, t2;

// Program setup.
void setup() {

  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for Serial
  }
  printf("Starting...\r\n");

  uint8_t mac[6];
  Ethernet.macAddress(mac);  // This is informative; it retrieves, not sets
  printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Ethernet.onLinkState([](bool state) {
    printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
  });

  IPAddress ip(10, 0, 0, 7);
  IPAddress sn{255,255,255,0};  // Subnet Mask
  IPAddress gw{10,0,0,1};       // Default Gateway

  if (!Ethernet.begin(ip, sn, gw)) {
    printf("Failed to start Ethernet\r\n");
    return;
  }

  printf("Obtaining the IP, Subnet and Gateway\r\n");
  ip = Ethernet.localIP();
  printf("    Local IP     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.subnetMask();
  printf("    Subnet mask  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.broadcastIP();
  printf("    Broadcast IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.gatewayIP();
  printf("    Gateway      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.dnsServerIP();
  printf("    DNS          = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);

  // Start UDP listening on the port
  udp.begin(kPort);

  // t1 = micros();
}

void loop() {
  
  receivePacket();
  sendPacket();
  // printf("Receiving Packet\n");
}

// Receives and prints chat packets.
void receivePacket() {
  // printf("receiving.. \n");
  
  int size = udp.parsePacket();
  if (size < 0) {
    return;
  };

  // Get the packet data and remote address
  const uint8_t *data = udp.data();
  // IPAddress ip = udp.remoteIP();

  //printf("[%u.%u.%u.%u][%d] ", ip[0], ip[1], ip[2], ip[3], size);
  float rcvdData[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  memcpy(rcvdData, data, sizeof(float) * 10);

  // printf("%u", sizeof(rcvdData));
  // printf("%u", sizeof(float));
  for(int i = 0; i<sizeof(rcvdData)/sizeof(float); i++){
    printf("%f", rcvdData[i]);
  }

  printf("\r\n");
  read_packet = true;
}

static void sendPacket() {
  // printf("sending ..\n");

  float value[13] = {4.3, 4.2, -1, 9.12, 2.22, 3.64, 4.005,
                      0.44, 42.4, 222.33, 2232.3, 44.33, 31.11};
  // Serialize the float value to a byte array
  
  char line[sizeof(float) * 13];
  memcpy(line, value, sizeof(float) * 13);
  
  if (read_packet) {
    IPAddress ip_send(10,0,0,6);
    if (!udp.send(ip_send, kPort,
                  reinterpret_cast<const uint8_t *>(line),
                  sizeof(float)*13)) {
      printf("[Error sending]\r\n");
    }
  
  read_packet = false;
  }
}
