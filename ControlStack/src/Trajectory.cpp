#include "../inc/Trajectory.h"


// for now, linearly interpolate between two points
vector_t Trajectory::interpolate(scalar_t t_0, scalar_t t_f, vector_t xbar_0, vector_t xbar_f) {

  vector_t xbar;
  scalar_t T = t_f - t_0;
  scalar_t t = T/2.;

  xbar = (1-t/T) * xbar_0 + (t/T) * xbar_f;
  return xbar;
}



// based on time, figure out which two nodes to interpolate
// realative time [t1 t2 t3 .. tn]
vector_t Trajectory::getState(scalar_t t){
  
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
  
  xbar = interpolate(t_0, t_f, xbar_0, xbar_f);
  
  return xbar;
}

