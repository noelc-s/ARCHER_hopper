#pragma once
#include "Types.h"
#include "Bezier.h"

using namespace Hopper_t;

// trajectory struct
struct Traj {
  vector_array_t nodes;  // std vector of Eigen vectors
  scalar_array_t times;  // std vector of Eigen scalars
  int num_nodes;
};
  
// Abstract Trajectory class
class Trajectory {
  
  public:
    Traj traj;
    scalar_t t_init;

    Trajectory(Traj traj, scalar_t t_init) : traj(traj), t_init(t_init){}
    virtual ~Trajectory() {}
  
    // returns xbar(t)
    virtual vector_t getState(scalar_t t) = 0;
    virtual vector_array_t getState(vector_t t) = 0;
  
  private:
    // interpolate between two points, returns xbar
    virtual vector_t interpolate(scalar_t t, scalar_t t_0, scalar_t t_f, vector_t xbar_0, vector_t xbar_f) = 0;
};

class Bezier_T : public Trajectory {
  
  public: 
    Bezier_T(Traj traj, scalar_t t_init):Trajectory(traj, t_init){};
    vector_t getState(scalar_t t);
    vector_array_t getState(vector_t t);
    Bezier B = Bezier(3,2,1);

  private:
    vector_t interpolate(scalar_t t, scalar_t t_0, scalar_t t_f, vector_t xbar_0, vector_t xbar_f);
};



