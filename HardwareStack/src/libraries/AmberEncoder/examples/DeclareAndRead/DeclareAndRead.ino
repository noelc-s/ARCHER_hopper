#include <AmberEncoder.h>

Encoder ENC_1(1,2,4000);    // Setup an encoder, connected to pins 1 and 2, with 4000 pulses per revolution
Encoder ENC_2(3,4,20000);   // Setup an encoder, connected to pins 3 and 4, with 20000 pulses per revolution
Encoder_t state1,state2;    // Declare encoder data structures for each encoder to be read later

float angle1,angle2,vel1,vel2; // Declaring the angle and velocity variables
uint32_t t1,t2;                // Declaring the encoder pulse time variables

void setup(){
  // Don't need anything in setup for reading the encoders
}

void loop(){
  state1 = ENC_1.read(); // Read the latest info from encoder 1
  state2 = ENC_2.read(); // Read the latest info from encoder 2

  angle1 = state1.pos;   // Angle of encoder 1, in radians, with respect to the 0 position
  angle2 = state2.pos;   // Angle of encoder 2, in radians, with respect to the 0 position
  vel1   = state1.vel;   // Velocity of encoder 1, in rad/s
  vel2   = state2.vel;   // Velocity of encoder 2, in rad/s
  t1     = state1.t;     // Time of the most recent pulse of encoder 1, in microseconds
  t2     = state2.t;     // Time of the most recent pulse of encoder 2, in microseconds

  // t1 and t2 are referenced to the time when the controller first turns on
}
