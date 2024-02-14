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
}

// Control character names.
static const String kCtrlNames[]{
  "NUL", "SOH", "STX", "ETX", "EOT", "ENQ", "ACK", "BEL",
  "BS",  "HT",  "LF",  "VT",  "FF",  "CR",  "SO",  "SI",
  "DLE", "DC1", "DC2", "DC3", "DC4", "NAK", "SYN", "ETB",
  "CAN", "EM",  "SUB", "ESC", "FS",  "GS",  "RS",  "US",
};

// Receives and prints chat packets.
void receivePacket() {
  
  int size = udp.parsePacket();
  if (size < 0) {
    return;
  };

  // Get the packet data and remote address
  const uint8_t *data = udp.data();
  // IPAddress ip = udp.remoteIP();

  //printf("[%u.%u.%u.%u][%d] ", ip[0], ip[1], ip[2], ip[3], size);
  float rcvdData[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  memcpy(rcvdData, data, sizeof(float) * 5);

  for(int i = 0; i<sizeof(rcvdData)/sizeof(float); i++){
    printf("%f", rcvdData[i]);
  }


  // // Print each character
  // for (int i = 0; i < size; i++) {
  //   uint8_t b = data[i];
  //   if (b < 0x20) {
  //     printf("<%s>", kCtrlNames[b].c_str());
  //   } else if (b < 0x7f) {
  //     putchar(data[i]);
  //   } else {
  //     printf("<%02xh>", data[i]);
  //   }
  // }
  printf("\r\n");
  read_packet = true;
  // t2 = micros();
}

static void sendPacket() {
  // static String line;
  // line = "Dane Joe Note that micro-benchmarking is hard. An accurate timer is only a small part of what's necessary to get meaningful results for short timed regions. See Idiomatic way of performance evaluation? for some more general caveats)";

  float value[4] = {9.12,2.22,3.64,4.005};
  // Serialize the float value to a byte array
  
  char line[sizeof(float) * 4];
  memcpy(line, value, sizeof(float) * 4);
  
  // printf(line);
  // printf('\r\n');


  
  if (read_packet) {
    IPAddress ip_send(10,0,0,6);
    if (!udp.send(ip_send, kPort,
                  reinterpret_cast<const uint8_t *>(line),
                  sizeof(float)*4)) {
      printf("[Error sending]\r\n");
    }
  
  read_packet = false;
  // Serial.println(t2-t1);
  // t1 = t2;
  
  }
}
