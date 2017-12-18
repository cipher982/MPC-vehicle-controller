#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 15;
double latency = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// desired speed
const double desired_speed = 60.0;

//indexes to reference variables inside the `vars` vector
size_t x_start     = 0;
size_t y_start     = x_start + N;
size_t psi_start   = y_start + N;
size_t v_start     = psi_start + N;
size_t cte_start   = v_start + N;
size_t epsi_start  = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start     = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    // calculate errors
    for (int t = 0; t < N; t++){
      
      // cte (cross-track error)
      fg[0] += 2.0*CppAD::pow(vars[cte_start + t], 2);
      // trajectory error
      fg[0] += 1300.0*CppAD::pow(vars[epsi_start + t], 2);
      // speed error
      fg[0] += 0.25*CppAD::pow(vars[v_start + t] - desired_speed, 2);  
      

      // cte (cross-track error)
      //fg[0] += CppAD::pow(vars[cte_start + t], 2);
      // trajectory error
      //fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      // speed error
      //fg[0] += CppAD::pow(vars[v_start + t] - desired_speed, 2);    
      
      // dilute actuators
      if (t < N - 1) {
        // steering
        fg[0] += CppAD::pow(vars[delta_start + t], 2);
        // throttle/brake
        fg[0] += CppAD::pow(vars[a_start + t], 2);
      }
      
      // actuator shock absorber
      if (t < N - 2) {
        //steering
        fg[0] += 10000.0*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        //fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);

        //acceleration
        fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);    
      }
    }

    ///////////////////////////////
    ////// setup constraints //////
    ///////////////////////////////

    fg[1 + x_start]    = vars[x_start];
    fg[1 + y_start]    = vars[y_start];
    fg[1 + psi_start]  = vars[psi_start];
    fg[1 + v_start]    = vars[v_start];
    fg[1 + cte_start]  = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // current state
      AD<double> x0     = vars[x_start + t - 1];
      AD<double> y0     = vars[y_start + t - 1];
      AD<double> psi0   = vars[psi_start + t - 1];
      AD<double> v0     = vars[v_start + t - 1];
      AD<double> cte0   = vars[cte_start + t - 1];
      AD<double> epsi0  = vars[epsi_start + t - 1];
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0     = vars[a_start + t - 1];
      
      // next state
      AD<double> x1    = vars[x_start + t];
      AD<double> y1    = vars[y_start + t];
      AD<double> psi1  = vars[psi_start + t];
      AD<double> v1    = vars[v_start + t];
      AD<double> cte1  = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];


      // 3rd order polynomial for the wiggle, no straight here like 1st!
      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*pow(x0,2) + coeffs[3]*pow(x0,3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));

      // Setup the rest of the model constraints
      fg[1 + x_start + t]    = x1    - (x0 + v0 * CppAD::cos(psi0) * latency);
      fg[1 + y_start + t]    = y1    - (y0 + v0 * CppAD::sin(psi0) * latency);
      fg[1 + psi_start + t]  = psi1  - (psi0 + v0 * delta0 / Lf * latency);
      fg[1 + v_start + t]    = v1    - (v0 + a0 * latency);
      fg[1 + cte_start + t]  = cte1  - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * latency));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * latency);      
    }
  }
};

//////////////////////////
//////// MPC class definition implementation.
//////////////////////////

MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;

  // Number of constraints
  size_t n_constraints = N * 6;

  // *** Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start]    = state[0];
  vars[y_start]    = state[1];
  vars[psi_start]  = state[2];
  vars[v_start]    = state[3];
  vars[cte_start]  = state[4];
  vars[epsi_start] = state[5];

  
  // *** Set lower and upper limits for the state variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -9999999999;
    vars_upperbound[i] =  9999999999;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.5;
    vars_upperbound[i] =  0.5;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }
  

  // *** Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  //initial state
  constraints_lowerbound[x_start]    = state[0];
  constraints_lowerbound[y_start]    = state[1];
  constraints_lowerbound[psi_start]  = state[2];
  constraints_lowerbound[v_start]    = state[3];
  constraints_lowerbound[cte_start]  = state[4];
  constraints_lowerbound[epsi_start] = state[5];
  
  constraints_upperbound[x_start]    = state[0];
  constraints_upperbound[y_start]    = state[1];
  constraints_upperbound[psi_start]  = state[2];
  constraints_upperbound[v_start]    = state[3];
  constraints_upperbound[cte_start]  = state[4];
  constraints_upperbound[epsi_start] = state[5];  

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.

  vector<double> res;

  std::cout << "delta_start:" << delta_start << " value: " << solution.x[delta_start] << std::endl;
  std::cout << "a start:" << a_start << " value:" << solution.x[a_start] << std::endl;

  res.push_back(solution.x[delta_start]);
  res.push_back(solution.x[a_start]);

  for(int i = 0; i < N; i++){
      res.push_back(solution.x[x_start + i]);
      res.push_back(solution.x[y_start + i]);
  }

return res;
}
