#include "Types.h"

using namespace Hopper_t;

    // trajectory struct
    struct Traj {
      vector_array_t nodes;  // std vector of Eigen vectors
      scalar_array_t times;  // std vector of Eigen scalars
      int num_nodes;
    };
  
class Trajectory {
  
  public: 
  
    Traj traj;

    // initialization time
    scalar_t t_init;
  
    // returns xbar(t)
    vector_t getState(scalar_t t);

    //  need to figure out how to initialize intial time
    Trajectory(Traj traj, scalar_t t_init) : traj(traj), t_init(t_init) {}

  private:
    
    // interpolate between two points, returns xbar
    vector_t interpolate(scalar_t t_0, scalar_t t_f, vector_t xbar_0, vector_t xbar_f);

};



