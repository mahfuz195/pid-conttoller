# CarND-Controls-PID Project 4
Self-Driving Car Engineer Nanodegree Program

---

The goal of this project is to drive the car by controlling the steering wheel using a PID controller. The tune the PID parameters I used different configurations of Kp, Kd, and Ki. After observing the performance of the car, I came up with the following values of PID constants: Kp = 0.15, Kd= 10.0, Ki=0.0; 

PID parameters selection:

PID controller stands for  proportional–integral–derivative controller is a feedback loop controller to control a machine.
The term P is proportional to the current value of the error (e.g., cross track error) at time T. The term D is a best estimate of the future trend of error, based on its current rate of change. The term I accounts for past values of the error and integrates them over time to produce the I term. Three constants Kp, Kd, Ki controllers the amount of P, D, and I  value need to consider into the system respectively. 


For the selction of Kp, I started with the the 1.0 keeping the Kd and Ki value to zero (only P controller). I selected the value of 0.15 where the car was osciliating on the track. Then, I tune the Kd for PD controller to reduce the oscillation. The challaning task was to make the car turn properly in the curvature. Too much of P was creating oscciliaiotn and with too high D reducing the ossiclaition but pushing the car out of the track. Finally, I added a small amount of Ki to consider systematic bias. 


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
