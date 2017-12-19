### MPC Vehicle Controller Project
##### David Rose


-------------------------
### Overview 
Through using a Unity game engine simulation of a vehicle on a track will be controlled using MPC (model predictive control) process control. This is similar to [my other project on GitHub that used a PID controller for this same task](https://github.com/cipher982/PID-Control). The difference is that an MPC controller (forgive the redundancy of words here, I prefer this phrasing) can effectively model future time states and plan ahead. The model was optimize throughout this finite time-horizon each actuator, which in this case is steering and throttle/brake. Throttle and braking are controlled via the same actuation via values with a range of  ```[-1,1]```.


Below you can see a high-level view of MPC operation with a single discrete input. Keep in mind this model uses two inputs (actuations) and operates on a continuous scale rather than discrete.
![MPC Diagram](https://github.com/cipher982/MPC-vehicle-controller/blob/master/media/434px-MPC_scheme_basic.svg.png "MPC Diagram")

### Steps Involved:
- Understand methods of accessing and controlling the car.
- Convert the MPC method to C++ code and implement in both main.cpp and mpc.cpp to control the vehicle
- Observe and take into account the simulated latency (in this case 100ms) of the sensor stack and optimize the model using this knowledge.

### Dependencies
- cmake >= 3.5
- All OSes: [click here for installation instructions](https://cmake.org/install/)
- make >= 4.1(mac, linux), 3.81(Windows)
- Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
- gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
- [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt** and **CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


### Vehicle Control API
Using a WebSocket server, the C++ console program communicates with the driving simulator through a JSON file that takes in and reads out various telemetry variables. These include: CTE, speed, angle, throttle, and steering angle.

In this MPC controller I will need to observe the CTE, heading, and speed, while pushing throttle (including braking for negative values under 0) and steering angle in an attempt to keep up the highest possible speed while staying safely within the lane. Though in this case I have locked speed to a specific value to aim for at 60mph. Below is the code for reading the telemetry into C++:



``` cpp 
// j[1] is the data JSON object
vector<double> ptsx = j[1]["ptsx"];
vector<double> ptsy = j[1]["ptsy"];
double px  = j[1]["x"];
double py  = j[1]["y"];
double psi = j[1]["psi"];
double v   = j[1]["speed"];
```


### MPC Controller
The aim of this is to model the behavior of dynamical systems, in this case the vehicle positioning and trajectory. This is a kinematic model that is a more advanced version of the [PID controller from my other repo](https://github.com/cipher982/PID-Control) that does not take into account future time states. This will model the change in dependent variables that are caused by changes in the independent variables. 

This is an iterative process based on a finite-horizon optimization of states. We will attempt to minimize the cost (CTE, heading, velocity) of the vehicle on the track. This is what is called on online model, in that it calculates on-the-fly.

This is a multivariable control algorithm that uses:
* a dynamical model of the process 
* a history of past actuations
* an optimization of a cost function (CTE, heading, velocity) over the prediction horizon

Below is the mathematical notation for some of the state prediction updates:
![MPC Overview](https://github.com/cipher982/MPC-vehicle-controller/blob/master/media/mpc_overview.png "MPC Overview")

### The code to implement these is as follows:
``` cpp
      fg[1 + x_start + t]    = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t]    = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t]  = psi1  - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t]    = v1    - (v0 + a0 * dt);
      fg[1 + cte_start + t]  = cte1  - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt); 
```
### Variables above, explained:
* **X** - Vehicle location in X coordinate
* **Y** - Vehicle location in Y coordinate
* **PSI** Vehicle heading (simulation works with radians, algorithm uses degrees). The two formula used to convert back and forth between the two are:
``` cpp 
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); } 
```
* **CTE** Cross-Track-Error (distance of vehicle from ideal (middle) line on track)
* **EPSI** Vehicle heading error (difference from ideal and actual, ideal is tangent to road curve)

### Tuning the Model
For me personally, I found these variables to work best with my model at a target speed of 50mph:
* **Timestep Length** - ```N``` - For this project I used a length of **12** as it predicted around longer turns but did not run too slowly for my computer. Though I have a feeling on slower CPUs it may struggle. 
* **Duration** - ```dt``` - Elapsed time at **100** should coincide very cleanly with the *100ms* delay built in to the model. 


### Fitting the Polynomial
Using the code below, a 3rd order polynomial is fitted to the ideal vehicle path for the future to best reduce the total error value. Using 3 polynomials allows for a more advanced curvature on some tighter sections of road. 

``` cpp
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}
```

### Simulating the Sensor Stack Latency 
In main.cpp I incremented the position forward in time with ``` latency = 0.1``` to represent 100ms delay in positioning detection. 

``` cpp
         // simulate latency compensation
          const double latency = 0.1;

          px = px + v * cos(psi) * latency;
          py = py + v * sin(psi) * latency;
```




