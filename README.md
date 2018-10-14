# PID Control
Self-Driving Car Engineer Nanodegree Program

---

##Objective

The purpose of this project was to build a PID controller to successfully drive the car around the track. 

##Controller Design

The car's steering and throttle were controlled using PID and PD controllers, respectively. While keeping the car's throttle constant, the steering PID controller hyperparameters were tuned manually first to get the car to roughly drive around the track and then, further fine-tuned using Twiddle to smoothen the drive. Then the throttle PD controller was manually tuned and fine-tuned using Twiddle to slow down and smoothen the drive around sharp corners. The final car throttle was set to 'constant value + PD-controller input'.

A PID controller was chosen for the steering because a proportional-only controller caused the car to oscillate from side-to-side and eventually, become unstable and drive off the road. See video of the behaviour of a P-only controller here. The oscillatory behaviour of the P-only steering controller was stabilized by adding in the derivative control component. With a PD controller, the car is able to drive successfully around the track. See video of the behaviour of the PD-only controller here. The integral control component was added to compensate for any persistent disturbances/errors in the steering control.

The video of the steering PID and throttle PD controllers in action can be viewed here.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
