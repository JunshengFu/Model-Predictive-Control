# **Autonomous driving with Model Predictive Control**

### 1. Objective
This project is to use Model Predictive Control (MPC) to drive a car in a game simulator. The server provides 
reference waypoints (yellow line in the demo video) via websocket, and we use MPC to compute steering and throttle commands 
to drive the car. The solution must be robust to 100ms latency, since it might encounter in real-world application.

In this project, the MPC optimize the actuators (steering and throttle), simulate 
the vehicle trajactory, and minimize the cost like cross-track error.

A **max speed** of **128** km/h (**80** MPH) is achieved in this project.


#### Demo: Autonomous driving with MPC (click to see the full video)

[![demo_gif1][gif1]](https://youtu.be/0l2K9EZ862U)

---

### 2. System in brief


#### 2.1 Kinematic model

A kinematic model is implemented to control the vehicle around the track. Kinematic models are 
simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy 
of the models, but it also makes them more tractable.

**States**: 

* x: cars x position
* y: cars y position
* ψ (psi): vehicle's angle in radians from the x-direction (radians)
* ν: vehicle's velocity
* cte: cross track error
* eψ : orientation error

**Actuator values**

* δ (delta): steering angle
* a : acceleration (including throttle and break)
 
**Update equations**

![state_update][image1]

#### 2.2 Timestep Length and Elapsed Duration (N & dt)

* N = 10
* dt = 0.12 s  // tested with 0.3, 0.12, 0.1, 0.08s

The prediction horizon is the duration over which future predictions are made. We’ll refer to this as T.
T is the product of two other variables, T =  N * dt. In the case of driving a car, T should be a few seconds, 
at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further 
into the future.  N and dt are hyperparameters you will need to tune for each model predictive controller you build. 
However, there are some general guidelines: T should be as large as possible, while dt should be as small as possible.
These guidelines create tradeoffs.

These following dt has been tested 0.3, 0.12, 0.1, 0.08, and I found that the dt should be greater than the
 latency in order to make the MPC work. So, I choose 0.12 as dt in the end.

The goal of Model Predictive Control is to optimize the control inputs: [δ,a]. An optimizer will tune these inputs 
until a low cost vector of control inputs is found. 


#### 2.3 Polynomial Fitting and MPC Preprocessing

Since the reference waypoints are given in the map global coordinate, and I transfer them into the car's coordinate
by a funciton called `map2car` (see line 117 in `main.cpp`), then a 3rd order polynomial is fitted to waypoints. 

#### 2.4 Model Predictive Control with Latency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates 
through the system. A realistic delay might be on the order of 100 milliseconds, so in this project 100 millisecond 
latency is handled by Model Predictive Controller. 

The latency is handled by using kinematic equations to predict the states for after 100ms before sending them to MPC 
(see code in line 126-134 in `main.cpp`).

---

## Code & Files
### 1. Dependencies & environment

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* uWebSockets: used for communication between the main code and the simulator.
    * [libs/install-mac](install-mac.sh) install uWebSockets in Mac.
    * [libs/install-ubuntu](install-ubuntu.sh) install uWebSockets in Ubuntu.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Download the tgz package instead of zip
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 

* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.    

### 2. How to run the code

1. Clone this repo.
2. Clean the project: `$./clean.sh`
3. Build the project: `$./build.sh` 
4. Run the project: `$./run.sh`
5. Start the [simulator v1.45](https://github.com/udacity/self-driving-car-sim/releases), 
select the MPC Controller. 


### 3. My project files

* [CMakeLists.txt](CMakeLists.txt) is the cmake file.
* [data](data) folder contains gif,fig, waypoints data. 
* [src](src) folder contains the source code.
* [clean.sh](clean.sh) cleans the project.
* [build.sh](build.sh) builds the project.
* [run.sh](run.sh) runs the project.
* [libs](libs) scripts to install uWebSockets.



### 4. Code Style

* [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


### 5. Release History

* 0.1.2
    * Fix two bugs in handling 100ms latency processing
    * Update the documentation 
    * Date 18 July 2017

* 0.1.1
    * Add bash files and demos
    * Date 17 July 2017

* 0.1.0
    * Create the repo with first proper release
    * Date 17 July 2017


[//]: # (Image References)
[image1]: ./data/state_update.png
[gif1]: ./data/demo.gif
