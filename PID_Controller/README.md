# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program


PID stands for Proportional-Integral-Derivative.These three components are combined in such a way that it produces a control signal. Before getting into the implementation, let’s discuss the PID controller components.
### Cross Track Error :
A cross track error is a distance of the vehicle from trajectory. In theory it’s best suited to control the car by steering in proportion to Cross Track Error(CTE).

### P component :
It sets the steering angle in proportion to CTE with a proportional factor tau. In other words, the P, or "proportional", component had the most directly observable effect on the car’s behaviour. It causes the car to steer proportional (and opposite) to the car’s distance from the lane center(CTE) - if the car is far to the right it steers hard to the left, if it’s slightly to the left it steers slightly to the right.

### I component :
It’s the integral or sum of error to deal with systematic biases. In other words, the I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift , but I believe that in this particular implementation the I component particularly serves to reduce the CTE around curves. And combination of these we can get PID controller to control the steering value.

### D component :
It’s the differential component of the controller which helps to take temporal derivative of error. This means when the car turned enough to reduce the error, it will help not to overshoot through the x axis. In other words, the D, or "differential", component counteracts the P component’s tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.

## Comparison of Controllers:

### P Controller :

The main usage of the P controller is to decrease the steady state error of the
system. As the proportional gain factor K increases, the steady state error of the system
decreases. However, despite the reduction, P control can never manage to eliminate the steady
state error of the system. As we increase the proportional gain, it provides smaller amplitude
and phase margin, faster dynamics satisfying wider frequency band and larger sensitivity to
the noise. We can use this controller only when our system is tolerable to a constant steady
state error. In addition, it can be easily concluded that applying P controller decreases the rise
time and after a certain value of reduction on the steady state error, increasing K only leads to
overshoot of the system response. P control also causes oscillation if sufficiently aggressive in
the presence of lags and/or dead time. 


![pCntrl](https://user-images.githubusercontent.com/37708330/56099352-f3bc6580-5f0b-11e9-8b31-ad0252f36608.png)

### PD Controller :

 In order to avoid effects of the sudden change in the value of the error signal, the derivative is taken from the
output response of the system variable instead of the error signal. Therefore, D mode is
designed to be proportional to the change of the output variable to prevent the sudden changes
occurring in the control output resulting from sudden changes in the error signal.

![pdCntrl](https://user-images.githubusercontent.com/37708330/56099353-f3bc6580-5f0b-11e9-9b88-4e33eb01fa23.png)

### PID Controller :
P-I-D controller has the optimum control dynamics including zero steady state error, fast
response (short rise time), no oscillations and higher stability. The necessity of using a
derivative gain component in addition to the PI controller is to eliminate the overshoot and the
oscillations occurring in the output response of the system. 

![unnamed](https://user-images.githubusercontent.com/37708330/56099354-f3bc6580-5f0b-11e9-9df6-91cf52f0c2cd.png)

### Twiddle :

### Tuning of Parameters :

 As an inital configuration I used the values from the lesson (p=0,2, i=0,004, d=3). These parameters worked well for constant throttle of 0.3. The I tried to increase the speed of the car by using higher constant throttle values and figured out that the car got of the track. In order to overcome this issue, a PID components were manually tuned to navigate through the given track.


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



## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

