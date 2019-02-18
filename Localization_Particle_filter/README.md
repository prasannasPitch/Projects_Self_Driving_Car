# Localization of Kidnapped Vehicle Project
Self-Driving Car Engineer Nanodegree Program

<p align="justify">
The goal of the project is to localize the movement of the kidnapped vehicle  with a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data. In this project, a 2 dimensional particle filter is implemented in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter, it will also get observation and control data.
</p>

## Table Content: ##
- [Motivation for Particle Filter](#motivation)
- [General Workflow](#imp)
- [Pseudo Code  Explaination](#psu)
- [Running the Code](#run)
- [File Structure](#file)
- [Result](#result)
- [Integrated Efficient C++ Features](#c++)


## Motivation for Particle Filter <a name="motivation"></a>
Before diving into particle filter, we recall the concept of localization. As the table given below, eventhough both sensor fusion and localization goal is to predict the location of a moving vehicle, it should be clear to know the similarities and the difference between these concepts.

| Localization            | Sensor Fusion                                                 |
|------------------|-------------------------------------------------------------|
| Things are in vehicle coordinates OR map coordinates. The entire objective of localization is to find the transformation between vehicle coordinates and map coordinates. In other words, we’re trying to find the position of the car in the map!            | Everything is in vehicle coordinates where the x axis points in the direction of the car’s heading and the y axis points to the left of the car.                                          |
| The position of the car is described in map coordinates.      | The car is always assumed to be at the origin of the vehicle coordinate system.  |
| The sensor measurements are usually described in vehicle coordinates. Vehicle coordinates have the x-axis in the direction of the car’s heading, the y-axis pointing orthogonal to the left of the car, and the z-axis pointing upwards.       | Sensor measurements are in vehicle coordinates. |
| Map landmarks are in map coordinates.       | There’s no map involved!   |

<p align="justify">
Localization of an object within a given map can be effected by probabilistic filter methods. One such filter is a particle filter - uses a random sampling method to generate different system states & then assign high weights to those state that are supported by sensor data. It is oftenly used for non-linear system and very easy to implement.
 </p>
 
 ## General Workflow<a name="imp"></a>
 
 
![image](https://user-images.githubusercontent.com/37708330/52903657-f444c280-3220-11e9-812c-d40fd60c681a.png)
 
 <p align="justify">
 The main objective of a particle filter is to track a variable of interest as it evolves over time, typically with a non-Gaussian and potentially multi-modal PDF (Probability Distribution Function). The basis of the method is to construct a sample-based representation of the entire PDF. A series of actions are taken, each one modifying the state of the variable of interest according to some model. Moreover, at certain times an observation arrives that constrains the state of the variable of interest at that time. Multiple copies (particles) of the variable of interest are used, each one associated with a weight that signifies the quality of the specific particle. An estimate of the variable of interest is obtained by the weighted sum of all the particles. The particle filter algorithm is recursive in nature and operates in two phases. 1. Prediction 2. Update. After each action, each particle is modified according to the existing model (prediction stage), including the addition of random noise in order to simulate the effect of noise on the variable of interest. Then, each particles weight is re-evaluated based on the latest sensory information available. At times the particles with small weights are eliminated, a process called resampling. 
 </p>

## Pseudocode  Explanation<a name="psu"></a>


![image](https://user-images.githubusercontent.com/37708330/52903692-95cc1400-3221-11e9-945c-2fd7250307eb.png)
 <p align="justify">
This is an outline of steps you will need to take with your code in order to implement a particle filter for localizing an autonomous vehicle. The pseudo code steps correspond to the steps in the algorithm flow chart, initialization, prediction, particle weight updates, and resampling.  </p> 
 
 1. At the initialization step we estimate our position from GPS input. The subsequent steps in the process will refine this estimate to localize our vehicle.

2. During the prediction step we add the control input (yaw rate & velocity) for all particles.

3. During the update step, we update our particle weights using map landmark positions and feature measurements.

4. During resampling we will resample M times (M is range of 0 to length_of_particleArray) drawing a particle i (i is the particle index) proportional to its weight.

5. The new set of particles represents the Bayes filter posterior probability. We now have a refined estimate of the vehicles position based on input evidence.


## Running the Code <a name="run"></a>
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


## File Structure  <a name="file"></a>
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

### Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Result<a name="result"></a>


<p align="justify">
Based on the comparison between the ground truth value and the predicted location of the particle filter, we could come to a conclusion if the particle filter gets passed. Below is a video of what it looks like when the simulator successfully is able to track the car to a particle. Notice that the green laser sensors from the car nearly overlap the blue laser sensors from the particle, this means that the particle transition calculations were done correctly.
 </p>
 
![ezgif com-video-to-gif 3](https://user-images.githubusercontent.com/37708330/52909920-0dd12300-3290-11e9-90ab-636567d88f77.gif)

The final result for the project is uploaded in youtube : https://youtu.be/AYDJ7L6k4Bw


## Integrated Efficient C++ Features <a name="c++"></a>

### Usage of Auto Keyword :
<p align="justify">
 Before C++ 11, each data type needs to be explicitly declared at compile time, limiting the values of an expression at runtime but after C++ (11 or more), many keywords are included which allows a programmer to leave the type deduction to the compiler itself. it's a useful way of simplifying object declarations as well as cleaning up the syntax for certain situations.
</p>

In this case, instead of looping with a conventional "for loop" structure, auto could be replaced. 


| Conventional for loop            | Using Auto                                                 |
|------------------|-------------------------------------------------------------|
| for(int i=0; i < observations.size(); ++i)| for (auto& observations_Object : observations) |
LandmarkObs& observations_Object = observations.at(i); | observations_Object.x = needed variable |
observations_Object.x = needed variable (3 line code yet too many variables)| two lines of code yet simple|

<p align="justify">
But the major point here is, in the conventional for loop, we check i < observations.size(); . 'i' being int and 'observation.size()' has a return type of size_type. Thus some compilers may throw error or atleast give warning to compare an int with an size_type datatype. This can be completly avoided by using auto as it takes care of everything thats needed for iteration.  </p>

