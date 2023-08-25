// server program for udp connection
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <strings.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <future>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "Hopper.h"
#include "Controller.h"
#include "Simulator.h"   ////

using namespace Eigen;
using namespace Hopper_t;
using namespace pinocchio;

// Driver code
int main(int argc, char *argv[]) {

  Controller controller;

  Simulator simulator; ////

  if (argc < 2) {
  } else {
    controller.port.reset(new uint16_t(static_cast<uint16_t>(std::stoul(argv[1]))));
  }

  controller.run();
  simulator.run(); ////
}
