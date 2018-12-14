# Sensor fusion by Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

## Motiavation for Lidar & Radar Sensors

<p align="justify">
As we approach towards level autonomous 4-5, the number of sensors in a car increases as the parallel processing shows efficient competence , but which types of sensor will be the most effective for perception and localization? </p>

<p align="justify">
- Lidar is the master of 3D mapping. Lidar, short for light detection and ranging, is a technology that measures distance using laser light. The technology can scan more than 100 meters in all directions, generating a precise 3D map of the car’s surroundings. This information is then used by car to make intelligent decisions about what to do next. The problem with lidar is that they generate a large amount of data and are still quite expensive for OEMs to cheaply implement. </p>

<p align="justify">
- Radar is the master of motion measurement. Radar, short for radio detection and ranging, is a sensor system that uses radio waves to determine the velocity, range and angle of objects. Radar is computationally lighter than a camera and uses far less data than a Lidar. While less angularly accurate than lidar, radar can work in every condition and even use reflection to see behind obstacles. Modern self-driving prototypes rely on radar and lidar to “cross validate” what they’re seeing and to predict motion.  </p>

<p align="justify">
- Cameras are the master of classification and texture interpretation. By far the cheapest and most available sensor (but not the cheapest processing), cameras use massive amounts of data (full HD means millions of pixel or Megabytes at every frame), making processing a computational intense and algorithmically complex job. Unlike both lidar and radar, cameras can see color, making them the best for scene interpretation.
 </p>


![classification](https://user-images.githubusercontent.com/37708330/50005132-8a523780-ffa9-11e8-8229-aaf60b765043.png)


## Sensor Fusion by Kalman Filter:

<p align="justify"> As we could see, both Lidar and Radar has its own strengths and limitations. As mentioned above, Lidar can map the surrounding and Radar can detect motion of other objects in surroundings. Combining measurements from both these sensors can result in accurate tracking of objects around a moving vechicle. Therefore sensor fusion technique should be implemented to use the advantages of both the sensors. One way of implementing sensor fusion is by implementing a Kalman Filter. In this project, we use kalman filter to track a moving car from a source vehicle. </p>

<p align="justify">
Kalman Filter works on prediction-correction model used for linear and time-variant or time-invariant systems. State prediction model involves the actual system and the process noise .The Measurement update model involves updating the predicated or the estimated value with the observation noise. </p>


![2step](https://user-images.githubusercontent.com/37708330/50016920-d6fa3a80-ffca-11e8-8a80-6fa0384adf5a.png) 

## Measurements from Sensors:
<p align="justify">
Till now we have discussed about why kalman filters are used for tracking problem. Before we dive into the working of kalman filter, we need to know what all are the necessary inputs and preprocessing steps involved in the measurements.</p>

### Lidar Measurement :

<p align="justify">
- z = transpose (px py) is the measurement vector. For a lidar sensor, the z vector contains the position−x and position−y measurements.
</p>

<p align="justify">
- H is the matrix that projects your belief about the object current state into the measurement space of the sensor. For lidar, this is a fancy way of saying that we discard velocity information from the state variable since the lidar sensor only measures position: The state vector x contains information about [p​x​​,p​y​​,v​x​​,v​y​​] whereas the z vector will only contain [px,py]. Multiplying Hx allows us to compare x, our belief, with z, the sensor measurement.</p>


![lildar_meas](https://user-images.githubusercontent.com/37708330/50017874-c0091780-ffcd-11e8-9a1f-d96a0c7ab46e.png)



### Radar Measurement :

- The range, (ρ), is the distance to the observed car. The range is basically the magnitude of the position vector ρ which can be defined as ρ=sqrt(p​x​2​​+p​y​2​​).
- φ=atan(p​y​​/p​x​​). Note that φ is referenced counter-clockwise from the x-axis, so φ from the video clip above in that situation would actually be negative.
- The range rate, ​ρ​˙​​, is the projection of the velocity, v, onto the line, L.


![radar_meas](https://user-images.githubusercontent.com/37708330/50017876-c13a4480-ffcd-11e8-8230-86e570b94b33.PNG)

## Design of Kalman Filter & Extended Kalman Filter :

The Kalman Filter algorithm will go through the following steps:

1. **First measurement** <p align="justify"> - the filter will receive initial measurements of the car position relative to the source vehicle. These measurements will come from a radar or lidar sensor.</p>
2. **Initialize state and covariance matrices**  <p align="justify"> - the filter will initialize the car position based on the first measurement. Then the source vehicle will receive another sensor measurement after a time period Δt.</p>
 
3. **Predict**  <p align="justify"> - the algorithm will predict where the car will be after time Δt. One basic way to predict the car location after Δt is to assume the car velocity is constant; thus the car will have moved velocity Δt.</p>
4. **Update**  <p align="justify"> - the filter compares the "predicted" location with what the sensor measurement says. The predicted location and the measured location are combined to give an updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value. The source vehicle will receive another sensor measurement after a time period Δt. The algorithm then does another predict and update step.</p>



However, there is one major change while implementing a kalman filter with Radar and Lidar sensor. Measurement of the Lidar sensor can be fitted in linear model but measurement function of a Radar sensor cannot be fitted into linear model. The reason behind this is, while applying a Lidar measurement with a linear model, the resultant is not a normal distribution (not Gaussian as required to implement Kalman filter). For linearising the model, we use a preprocessing step to make the resultant in a normal distribution so that the kalman filter can be applied. So, an extended Kalman filter (EKF) is just an extension of Kalman that can be applied to nonlinear systems.











![ekf_flow](https://user-images.githubusercontent.com/37708330/50034416-e4c8b380-fffc-11e8-872e-03881bfedae2.jpg)



















------------------------------------------------update on progress----------------------------------------------------------

## Setting up the project :
In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.

## Note!

 * Please note that the ```Eigen``` library does not initialize ```VectorXd``` or ```MatrixXd``` objects with zeros upon creation.
