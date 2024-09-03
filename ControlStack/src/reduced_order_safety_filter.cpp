// reduced_order_safety_filter.cpp

#include "../inc/reduced_order_safety_filter.h"

/*
Compute input for reduced-order model.

Arguments:
- Eigen::Vector2d &x (current state of reduced-order model)

Returns:
- Eigen::Vector2d u (input generated by CBF controller)
*/
Eigen::Vector2d ReducedOrderSafetyFilter::get_input(const Eigen::Vector2d &x) {

  // Initialize input vector
  Eigen::Vector2d u;

  // Fill up first component of input
  u(0) = ifelse(
      ifelse(-1 * Kp * (x(0) + -1 * xd(0)) > umax, umax,
             ifelse(-1 * Kp * (x(0) + -1 * xd(0)) < -1 * umax * 1,
                    -1 * umax * 1, -1 * Kp * (x(0) + -1 * xd(0)))) +
              2 * (x(0) + -1 * xo(0)) *
                  std::max(
                      0.0,
                      ((4 * pow(x(0) + -1 * xo(0), 2) +
                        4 * pow(x(1) + -1 * xo(1), 2)) /
                           epsilon +
                       (4 * pow(x(0) + -1 * xo(0), 2) +
                        4 * pow(x(1) + -1 * xo(1), 2)) /
                           sigma +
                       -2 * (x(0) + -1 * xo(0)) *
                           ifelse(-1 * Kp * (x(0) + -1 * xd(0)) > umax, umax,
                                  ifelse(-1 * Kp * (x(0) + -1 * xd(0)) <
                                             -1 * umax * 1,
                                         -1 * umax * 1,
                                         -1 * Kp * (x(0) + -1 * xd(0)))) +
                       -2 * (x(1) + -1 * xo(1)) *
                           ifelse(-1 * Kp * (x(1) + -1 * xd(1)) > umax, umax,
                                  ifelse(-1 * Kp * (x(1) + -1 * xd(1)) <
                                             -1 * umax * 1,
                                         -1 * umax * 1,
                                         -1 * Kp * (x(1) + -1 * xd(1)))) +
                       -1 *
                           (-1 * (ro * ro) + pow(x(0) + -1 * xo(0), 2) +
                            pow(x(1) + -1 * xo(1), 2)) *
                           alpha) /
                          (4 * pow(x(0) + -1 * xo(0), 2) +
                           4 * pow(x(1) + -1 * xo(1), 2))) >
          umax,
      umax,
      ifelse(
          ifelse(-1 * Kp * (x(0) + -1 * xd(0)) > umax, umax,
                 ifelse(-1 * Kp * (x(0) + -1 * xd(0)) < -1 * umax * 1,
                        -1 * umax * 1, -1 * Kp * (x(0) + -1 * xd(0)))) +
                  2 * (x(0) + -1 * xo(0)) *
                      std::max(
                          0.0,
                          ((4 * pow(x(0) + -1 * xo(0), 2) +
                            4 * pow(x(1) + -1 * xo(1), 2)) /
                               epsilon +
                           (4 * pow(x(0) + -1 * xo(0), 2) +
                            4 * pow(x(1) + -1 * xo(1), 2)) /
                               sigma +
                           -2 * (x(0) + -1 * xo(0)) *
                               ifelse(-1 * Kp * (x(0) + -1 * xd(0)) > umax,
                                      umax,
                                      ifelse(-1 * Kp * (x(0) + -1 * xd(0)) <
                                                 -1 * umax * 1,
                                             -1 * umax * 1,
                                             -1 * Kp * (x(0) + -1 * xd(0)))) +
                           -2 * (x(1) + -1 * xo(1)) *
                               ifelse(-1 * Kp * (x(1) + -1 * xd(1)) > umax,
                                      umax,
                                      ifelse(-1 * Kp * (x(1) + -1 * xd(1)) <
                                                 -1 * umax * 1,
                                             -1 * umax * 1,
                                             -1 * Kp * (x(1) + -1 * xd(1)))) +
                           -1 *
                               (-1 * (ro * ro) + pow(x(0) + -1 * xo(0), 2) +
                                pow(x(1) + -1 * xo(1), 2)) *
                               alpha) /
                              (4 * pow(x(0) + -1 * xo(0), 2) +
                               4 * pow(x(1) + -1 * xo(1), 2))) <
              -1 * umax * 1,
          -1 * umax * 1,
          ifelse(-1 * Kp * (x(0) + -1 * xd(0)) > umax, umax,
                 ifelse(-1 * Kp * (x(0) + -1 * xd(0)) < -1 * umax * 1,
                        -1 * umax * 1, -1 * Kp * (x(0) + -1 * xd(0)))) +
              2 * (x(0) + -1 * xo(0)) *
                  std::max(
                      0.0,
                      ((4 * pow(x(0) + -1 * xo(0), 2) +
                        4 * pow(x(1) + -1 * xo(1), 2)) /
                           epsilon +
                       (4 * pow(x(0) + -1 * xo(0), 2) +
                        4 * pow(x(1) + -1 * xo(1), 2)) /
                           sigma +
                       -2 * (x(0) + -1 * xo(0)) *
                           ifelse(-1 * Kp * (x(0) + -1 * xd(0)) > umax, umax,
                                  ifelse(-1 * Kp * (x(0) + -1 * xd(0)) <
                                             -1 * umax * 1,
                                         -1 * umax * 1,
                                         -1 * Kp * (x(0) + -1 * xd(0)))) +
                       -2 * (x(1) + -1 * xo(1)) *
                           ifelse(-1 * Kp * (x(1) + -1 * xd(1)) > umax, umax,
                                  ifelse(-1 * Kp * (x(1) + -1 * xd(1)) <
                                             -1 * umax * 1,
                                         -1 * umax * 1,
                                         -1 * Kp * (x(1) + -1 * xd(1)))) +
                       -1 *
                           (-1 * (ro * ro) + pow(x(0) + -1 * xo(0), 2) +
                            pow(x(1) + -1 * xo(1), 2)) *
                           alpha) /
                          (4 * pow(x(0) + -1 * xo(0), 2) +
                           4 * pow(x(1) + -1 * xo(1), 2)))));

  // Fill up second component of input
  u(1) = ifelse(
      ifelse(-1 * Kp * (x(1) + -1 * xd(1)) > umax, umax,
             ifelse(-1 * Kp * (x(1) + -1 * xd(1)) < -1 * umax * 1,
                    -1 * umax * 1, -1 * Kp * (x(1) + -1 * xd(1)))) +
              2 * (x(1) + -1 * xo(1)) *
                  std::max(
                      0.0,
                      ((4 * pow(x(0) + -1 * xo(0), 2) +
                        4 * pow(x(1) + -1 * xo(1), 2)) /
                           epsilon +
                       (4 * pow(x(0) + -1 * xo(0), 2) +
                        4 * pow(x(1) + -1 * xo(1), 2)) /
                           sigma +
                       -2 * (x(0) + -1 * xo(0)) *
                           ifelse(-1 * Kp * (x(0) + -1 * xd(0)) > umax, umax,
                                  ifelse(-1 * Kp * (x(0) + -1 * xd(0)) <
                                             -1 * umax * 1,
                                         -1 * umax * 1,
                                         -1 * Kp * (x(0) + -1 * xd(0)))) +
                       -2 * (x(1) + -1 * xo(1)) *
                           ifelse(-1 * Kp * (x(1) + -1 * xd(1)) > umax, umax,
                                  ifelse(-1 * Kp * (x(1) + -1 * xd(1)) <
                                             -1 * umax * 1,
                                         -1 * umax * 1,
                                         -1 * Kp * (x(1) + -1 * xd(1)))) +
                       -1 *
                           (-1 * (ro * ro) + pow(x(0) + -1 * xo(0), 2) +
                            pow(x(1) + -1 * xo(1), 2)) *
                           alpha) /
                          (4 * pow(x(0) + -1 * xo(0), 2) +
                           4 * pow(x(1) + -1 * xo(1), 2))) >
          umax,
      umax,
      ifelse(
          ifelse(-1 * Kp * (x(1) + -1 * xd(1)) > umax, umax,
                 ifelse(-1 * Kp * (x(1) + -1 * xd(1)) < -1 * umax * 1,
                        -1 * umax * 1, -1 * Kp * (x(1) + -1 * xd(1)))) +
                  2 * (x(1) + -1 * xo(1)) *
                      std::max(
                          0.0,
                          ((4 * pow(x(0) + -1 * xo(0), 2) +
                            4 * pow(x(1) + -1 * xo(1), 2)) /
                               epsilon +
                           (4 * pow(x(0) + -1 * xo(0), 2) +
                            4 * pow(x(1) + -1 * xo(1), 2)) /
                               sigma +
                           -2 * (x(0) + -1 * xo(0)) *
                               ifelse(-1 * Kp * (x(0) + -1 * xd(0)) > umax,
                                      umax,
                                      ifelse(-1 * Kp * (x(0) + -1 * xd(0)) <
                                                 -1 * umax * 1,
                                             -1 * umax * 1,
                                             -1 * Kp * (x(0) + -1 * xd(0)))) +
                           -2 * (x(1) + -1 * xo(1)) *
                               ifelse(-1 * Kp * (x(1) + -1 * xd(1)) > umax,
                                      umax,
                                      ifelse(-1 * Kp * (x(1) + -1 * xd(1)) <
                                                 -1 * umax * 1,
                                             -1 * umax * 1,
                                             -1 * Kp * (x(1) + -1 * xd(1)))) +
                           -1 *
                               (-1 * (ro * ro) + pow(x(0) + -1 * xo(0), 2) +
                                pow(x(1) + -1 * xo(1), 2)) *
                               alpha) /
                              (4 * pow(x(0) + -1 * xo(0), 2) +
                               4 * pow(x(1) + -1 * xo(1), 2))) <
              -1 * umax * 1,
          -1 * umax * 1,
          ifelse(-1 * Kp * (x(1) + -1 * xd(1)) > umax, umax,
                 ifelse(-1 * Kp * (x(1) + -1 * xd(1)) < -1 * umax * 1,
                        -1 * umax * 1, -1 * Kp * (x(1) + -1 * xd(1)))) +
              2 * (x(1) + -1 * xo(1)) *
                  std::max(
                      0.0,
                      ((4 * pow(x(0) + -1 * xo(0), 2) +
                        4 * pow(x(1) + -1 * xo(1), 2)) /
                           epsilon +
                       (4 * pow(x(0) + -1 * xo(0), 2) +
                        4 * pow(x(1) + -1 * xo(1), 2)) /
                           sigma +
                       -2 * (x(0) + -1 * xo(0)) *
                           ifelse(-1 * Kp * (x(0) + -1 * xd(0)) > umax, umax,
                                  ifelse(-1 * Kp * (x(0) + -1 * xd(0)) <
                                             -1 * umax * 1,
                                         -1 * umax * 1,
                                         -1 * Kp * (x(0) + -1 * xd(0)))) +
                       -2 * (x(1) + -1 * xo(1)) *
                           ifelse(-1 * Kp * (x(1) + -1 * xd(1)) > umax, umax,
                                  ifelse(-1 * Kp * (x(1) + -1 * xd(1)) <
                                             -1 * umax * 1,
                                         -1 * umax * 1,
                                         -1 * Kp * (x(1) + -1 * xd(1)))) +
                       -1 *
                           (-1 * (ro * ro) + pow(x(0) + -1 * xo(0), 2) +
                            pow(x(1) + -1 * xo(1), 2)) *
                           alpha) /
                          (4 * pow(x(0) + -1 * xo(0), 2) +
                           4 * pow(x(1) + -1 * xo(1), 2)))));

  return u;
}