#include "../inc/Trajectory.h"
#include<iostream>

// for now, linearly interpolate between two points
vector_t Bezier_T::interpolate(scalar_t t, scalar_t t_0, scalar_t t_f, vector_t xbar_0, vector_t xbar_f) {

  vector_t xbar;
  scalar_t T = t_f - t_0;
  matrix_t Xi(4, xbar_0.size()/2);
  vector_t X_d(4);
  B.tau = t_f - t_0;
  for (int i = 0; i < xbar_0.size()/2; i++) {
    X_d << xbar_0(i), xbar_0(i+xbar_0.size()/2), xbar_f(i), xbar_f(i+xbar_f.size()/2);
    Xi.block(0,i,4,1) << B.D.transpose().partialPivLu().solve(X_d);
  }

  xbar = B.B(t-t_0,Xi).transpose();
  return xbar;
}

vector_array_t Bezier_T::getState(vector_t t){
  vector_array_t XD;
  vector_t xd;
  for (int i = 0; i < t.size(); i++) {
    xd = getState(t(i));
    XD.push_back(xd);
  }
  return XD;
}

// based on time, figure out which two nodes to interpolate
// realative time [t1 t2 t3 .. tn]
//
// TODO: There is a bug when you evaluate exactly on the start and end of an interval
vector_t Bezier_T::getState(scalar_t t){
  
  // t in which time interval
  scalar_array_t time_vals;
  time_vals = traj.times;
  
  int interval = -1;
  for (int i=0; i<time_vals.size()-1; i++) {
    if (time_vals[i] <= t && t < time_vals[i+1]) {
     interval = i;
     break;
    }
  }

  // set interval xbar endpoints
  scalar_t t_0, t_f;
  vector_t xbar_0, xbar_f;
  t_0 = time_vals[interval];
  t_f = time_vals[interval+1];
  
  xbar_0 = traj.nodes[interval];
  xbar_f = traj.nodes[interval+1];

  // return xbar(t)
  vector_t xbar(xbar_0.rows());
  
  xbar = interpolate(t, t_0, t_f, xbar_0, xbar_f);
  
  return xbar;
}

