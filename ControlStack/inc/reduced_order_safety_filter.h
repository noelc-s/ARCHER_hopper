// reduced_order_safety_filter.h

#ifndef REDUCED_ORDER_SAFETY_FILTER
#define REDUCED_ORDER_SAFETY_FILTER

#include <Eigen/Dense>
#include <algorithm>

// Julia likes to generate C++ code that uses this for some reason
#define ifelse(condition, true_value, false_value)                             \
  ((condition) ? (true_value) : (false_value))

class ReducedOrderSafetyFilter {
public:
  Eigen::Vector2d xd; // Goal location
  double Kp;          // Proportional gain on desired controller
  double umax;        // Maximum input
  Eigen::Vector2d xo; // Location of obstacle
  double ro;          // Radius of obstacle
  double alpha;       // Class K function coefficient on CBF
  double epsilon;     // Gain on ISSf term for tracking error
  double sigma;       // Gain on ISSf term for ROM model error

  // Function to compute control
  Eigen::Vector2d get_input(const Eigen::Vector2d &x);
  // Function to compute barrier value
  double h(const Eigen::Vector2d &x);
};

#endif // REDUCED_ORDER_SAFETY_FILTER
