#pragma once
#include "Types.h"

using namespace Hopper_t;

struct EstimatedState
{
  scalar_t x, y, z;
  scalar_t cam_x, cam_y, cam_z;
  scalar_t cam_xdot, cam_ydot, cam_zdot;
  scalar_t x_dot, y_dot, z_dot;
  scalar_t q_w, q_x, q_y, q_z;
  scalar_t cam_q_w, cam_q_x, cam_q_y, cam_q_z;
  scalar_t omega_x, omega_y, omega_z;
};

// Kalman Filter gain
bool contact = false;
scalar_t alpha; // filtering alpha