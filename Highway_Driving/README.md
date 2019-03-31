# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Table Content: ##
- [Goal of the Project](#goal)
- [Introduction - Path Planning](#path)
- [Prediction](#predict)
- [Behavior Planning](#behav_planning)
- [Trajectory Generation](#trajectory)
- [Result](#result)
- [File Structure](#file)
- [Integrated Efficient C++ (11) Features](#c++)


### Goal <a name="goal"></a>
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Introduction <a name="path"></a>
<p align="justify">
Path planning and decision making for autonomous vehicles in urban environments enable self-driving cars to find the safest, most convenient, and most economically beneficial routes from point A to point B. Finding routes is complicated by all of the static and maneuverable obstacles that a vehicle must identify and bypass. Today, the major path planning approaches include the predictive control model, feasible model, and behavior-based model. Let’s first get familiar with some terms to understand how these approaches work.

![image](https://user-images.githubusercontent.com/37708330/53702229-f6a04280-3e04-11e9-95c6-26888f5b78ff.png)

- A path is a continuous sequence of configurations beginning and ending with boundary configurations. These configurations are also referred to as initial and terminating.
- Path planning involves finding a geometric path from an initial configuration to a given configuration so that each configuration and state on the path is feasible (if time is taken into account).
- A maneuver is a high-level characteristic of a vehicle’s motion, encompassing the position and speed of the vehicle on the road. Examples of maneuvers include going straight, changing lanes, turning, and overtaking.
- Maneuver planning aims at taking the best high-level decision for a vehicle while taking into account the path specified by path planning mechanisms.
- A trajectory is a sequence of states visited by the vehicle, parameterized by time and, most probably, velocity.
T- rajectory planning or trajectory generation is the real-time planning of a vehicle’s move from one feasible state to the next, satisfying the car’s kinematic limits based on its dynamics and as constrained by the navigation mode.

 <a name="predict"></a>
### Prediction
<p align="justify">
Prediction is the first step of path planning. It involves, predicting the behavior of other vehicles, and estimating their location at a time step in the future. Because of the time limitations, predictions only performed by increasing the s values of other cars, and the d value of each other car assumed to be constant in each step. The provided vx and vy values were combined in a single v, and this v value used to calculate the s value in the future. Consider the below example in which the green car could end up in two possible trajectories (straight trajectory or righ turn trajectory). So, for accurate predictions, either one of this approach can be used. 
</p>

- Model Based
- Machine Learning Aproach

![image](https://user-images.githubusercontent.com/37708330/55282517-5c6ce500-5345-11e9-962e-5647366a6b04.png)

 <a name="behav_planning"></a>
### Behavior Planning
<p align="justify">
 This step is responsible for providing guidance for trajectory planner about what sort of maneuvers they should plan the trajectories. Based on the predictions, route and map the behavior planner gives the possible maneuvers that could be possible at the given circumstances. Factors like feasibility, safety, optimality, legality are incorprated in this step. 
</p>

![image](https://user-images.githubusercontent.com/37708330/53702249-29e2d180-3e05-11e9-82ea-c99735b90cbe.png)

#### Finite State Machines
In the real time scenario there are could be a lot of maneuvers or states that can be possible for navigating from one place to another. For example, in a high way driving scenario there could be states like lane changing state, follow proceeding car state, maintain speed limit state and so on. So we require something like a finite state machine model to plan the behavior of the autonomous vehicle.

![image](https://user-images.githubusercontent.com/37708330/53702329-0bc9a100-3e06-11e9-80c7-f48eb9fa8072.png)

<a name="trajectory"></a>
### Trajectory Generation 
Trajectory Generation is the second step and probably the most challenging step of path planning. During generation step, maximum acceleration and maximum velocity values were set in advance. The interactions with the other cars were calculated, such as closest approach, and lowest time to collide. In addition, other penalties added for not driving at the target speed, changing lanes, driving at side lanes, canceling the previous action, driving in an occupied lane.



   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
## Lecture Notes

![software-architecture](https://user-images.githubusercontent.com/37708330/55198216-711a7300-51b5-11e9-91da-01b3994b6b17.png)

## Result<a name="result"></a>

The final result for the project is uploaded in youtube : https://youtu.be/vnXjv3sCKpA

### Highway Map
Inside data/highway_map.csv there is a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway.

The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway.

The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change.

Waypoint Data
Each waypoint has an (x,y) global map position, and a Frenet s value and Frenet d unit normal vector (split up into the x component, and the y component).

The s value is the distance along the direction of the road. The first waypoint has an s value of 0 because it is the starting point.

The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. The d vector can be used to calculate lane positions. For example, if you want to be in the left lane at some waypoint just add the waypoint's (x,y) coordinates with the d vector multiplied by 2. Since the lane is 4 m wide, the middle of the left lane (the lane closest to the double-yellow dividing line) is 2 m from the waypoint.

If you would like to be in the middle lane, add the waypoint's coordinates to the d vector multiplied by 6 = (2+4), since the center of the middle lane is 4 m from the center of the left lane, which is itself 2 m from the double-yellow dividing line and the waypoints.

Converting Frenet Coordinates
We have included a helper function, getXY, which takes in Frenet (s,d) coordinates and transforms them to (x,y) coordinates.

Interpolating Points
If you need to estimate the location of points between the known waypoints, you will need to "interpolate" the position of those points.

In previous lessons we looked at fitting polynomials to waypoints. Once you have a polyn
