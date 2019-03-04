
# Udacity Self-Driving Car Nanodegree Term 2, Model Predictive Control
![mpc](./mpc.png)

## Project Steps
- Fitting a line based on road waypoints and evaluating the current state based on that polynomial line.

- Implementing the MPC calculation, including setting variables and constraints

- Calculating actuator values from the MPC calc based on current state

- Accounting for latency (I used a predicted state 100ms in the future to replace the actual current state in the calculation)

- Calculating steering angle & throttle/brake based on the actuator values

- Setting timestep length and duration

- Testing/tuning of above implementations on Udacity simulator

## Discussion/Reflection

## The Model

states are included:

ptsx (x-position of waypoints ahead on the track in global coordinates)

ptsy (y-position of waypoints ahead on the track in global coordinates)

px (current x-position of the vehicle's position in global coordinates)

py (current y-position of the vehicle's position in global coordinates)

psi (current orientation angle of the vehicle)

v (current velocity of the vehicle)

delta (steering angle of the car)

throttle (current throttle)

## Tuning Timesteps (N) and Timestep Duration (dt)
Based on the visualization in the track, I found the N value is good between range 10 to 15, as N goes higherm the model starts to slow down, so I choose N = 10,  and I tried differetn dt value from 0.1 to 0.2, at 0.2 for dt it is two slow to react ,so I choose dt = 0.1 in the end with good performance.

## Polynomial Fitting & Preprocessing
First, I transform the points from the simulator's global coordinates into the vehicle's coordinates.Then, each of the waypoints are adjusted by subtracting out px and py accordingly such that they are based on the vehicle's position. Next, the waypoint coordinates are changed using standard 2d vector transformation equations to be in vehicle coordinates:

ptsx_car[i] = x * cos(-psi) - y * sin(-psi)

ptsy_car[i] = x * sin(-psi) + y * cos(-psi)

Using the polyfit() function, a third-degree polynomial line is fit to these transformed waypoints, essentially drawing the path the vehicle should try to travel. Moving on further is where the transformations are critical - because we are operating from the vehicle's coordinates, we can use px, py and psi all equal to zero: from the vehicle's standpoint, it is the center of the coordinate system, and it is always pointing to a zero orientation. The cross-track error can then be calculated by evaluating the polynomial function (polyeval()) at px (which in this case is now zero, so technically could also just be calculated as the first coefficient value - i.e. the one with a zero-order x). The psi error, or epsi, which is calculated from the derivative of polynomial fit line, is therefore simpler to calculate, as polynomials above the first order in the original equation are all eliminated through multiplication by zero (since x is zero). It is the negative arc tangent of the second coefficient (the first-order x was in the original polynomial).

##  Latency
The model in this project accounts for the simulator's added 100ms latency between the actuator calculation (when the model tells the car to perform a steering or acceleration/braking change) and when the simulator will actually perform that action. To implement this, I added in a step to predict where the vehicle would be after 100ms (0.1 seconds), in order to take the action that needed to actually be taken at that time, instead of the one in reaction to an old situation. I set the "dt" value here (not to be confused with the one in MPC.cpp, although both are the same value) to equal the latency. Then, using the same update equations as those used in the actual MPC model, I predicted the state and fed that into the true model. Note that these equations were able to be simplified again because of the coordinate system transformation - using x, y and psi all of zero made these equations a little simpler, as lots of the values end up being zero or one. See lines 131-143 in main.cpp. This new predicted state, along with the coefficients, are then fed into the mpc.Solve() function found in MPC.cpp.

# Requirments:

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
