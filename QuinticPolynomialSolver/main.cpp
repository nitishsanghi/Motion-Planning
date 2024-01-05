#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include "grader.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * TODO: complete this function
 */
vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  vector<double> output;
  output.push_back(start[0]);
  output.push_back(start[1]);
  output.push_back(start[2]/2);
  
  auto sf = end[0] - output[0] - output[1]*T - output[2]*T*T;
  auto sfdot = end[1] - output[1] - 2*output[2]*T;
  auto sfdotdot = end[2] - 2*output[2];
	
 VectorXd V = VectorXd(3,1);
  V << sf, sfdot, sfdotdot;
  
  MatrixXd A = MatrixXd(3, 3);
  A << pow(T,3), pow(T,4), pow(T,5),3*pow(T,2), 4*pow(T,3), 5*pow(T,4),6*T, 12*pow(T,2), 20*pow(T,3);
  
  auto A_i = A.inverse();
  
  auto S = A_i*V;
  
  for(int i = 0; i < S.size(); ++i) {
    output.push_back(S.data()[i]);
  }
  return output;
}

int main() {

  // create test cases
  vector< test_case > tc = create_tests();

  bool total_correct = true;

  for(int i = 0; i < tc.size(); ++i) {
    vector<double> jmt = JMT(tc[i].start, tc[i].end, tc[i].T);
    bool correct = close_enough(jmt, answers[i]);
    total_correct &= correct;
  }

  if(!total_correct) {
    std::cout << "Try again!" << std::endl;
  } else {
    std::cout << "Nice work!" << std::endl;
  }

  return 0;
}