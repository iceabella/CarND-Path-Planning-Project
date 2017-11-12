# Path-Planning-Project
This is my implementation of the Path Planning project for Udacity's Self-Driving Car Engineer Nanodegree Program.

## Project introduction
In this project the goal is to safely navigate a subject vehicle around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit (link to simulator further down). The car's localization and sensor fusion data is provided as well as a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Implementation details
The steps used to implement the project were:
1. CHECK IMPOSSIBLE NEW STATES (because of road restrictions)
2. PREDICT TRAJECTORIES FOR TARGETS
3. GET COST FOR EACH SUBJECT TRAJECTORY (Finite State Machine)
4. CHOOSE TRAJECTORY WITH LOWEST COST
5. CREATE TRAJECTORY POINTS TO USE IN SIMULATOR

These steps will be described further below. If you want to skip forward and just see my vehicle drive you can check it out [here](https://www.youtube.com/watch?v=ekPMzGfzSbo) and [here](https://www.youtube.com/watch?v=Treo3DD_XKY)

### Check impossible new states
An assumption was made that the map and localization data is 100% accurate, why lane change to the right is not considered if driving in the rightmost lane and likewise  lane change to the left is not considered when driving in the leftmost lane.

### Predict trajectories for targets
To be able to estimate the best trajectory for the subject vehicle, the targets future positions need to be estimated. The estimates are made in Fernet coordinates where the assumption is that the target vehicle will not change lane but use all the speed to go forward, i.e. updating the s-position only. The implementation can be seen in main.cpp from line 125.

### Get cost for each subject trajectory.
The trajectory planning problem is implemented as a Finate State Machine with the states:
 - keep lane and speed
 - keep lane and decelerate
 - keep lane and decelerate hard
 - keep lane and accelerate
 - lane change right (keep speed)
 - lane change left (keep speed)
 
For each state a cost is estimated for the manouver. The cost functions can be seen in cost_functions.hpp, it is also here the prediction of the future positions of the subject vehicle is made. It is assumed that the subject vehicle will stay in the same lane during the whole prediction horizon and with constant speed, this means that a lane change trajectory is estimated as just going forward dependent on speed in the s-direction but in the lane next to the one currently driving in. This is a good assumption to check wheather it is safe to change lane or not. Note also that the predictions are made from some time steps ahead in the future, dependent on how many trajectory points is left in the list of points sent to the simulator in the previous time step. This means that the first position evaluated of the subject and target vehicle are:
``` c++
target_s += target_v*prev_size*real_dt; 
```
as can be seen in main.cpp, line 150.

The costs considered (in decreasing order) are:
- collision cost:
Cost dependent on how close we are to a collision, cost = exp(-TTC^2) (TTC = Time To Collision). If no collision is seen during prediction horizon cost will be 0.
- legal speed cost:
Cost to keep our speed below the speed limit.
- distance buffer cost:
Cost to keep us on a safe distance to targets.
- efficiency speed cost:
Cost to keep our speed as close as possible to the speed  limit.

### Choosing trajectory with lowest cost
The trajectory with lowest cost is chosen, but if there are several costs that have the minimum value the following order will be considered because of how it is implemented (see main.cpp line 215):
 - keep lane and decelerate
 - keep lane and decelerate hard
 - keep lane and speed
 - keep lane and accelerate
 - lane change right (keep speed)
 - lane change left (keep speed)

Quite often the cost for:
 - keep lane and speed
 - lane change right (keep speed)
 - lane change left (keep speed)

are the same since for the moment no jerk or acceleration costs are used, which means that there is not any additional cost to change lane. The jerk is solved instead by creating a smooth trajectory (see next section) as well as not having a too high acceleration/deceleration, which also solves the acceleration issue. Worth noting is also that if more points are sent to the simulator it will also take longer for the subject vehicle to change from one trajectory choice to another, why jerk will be a smaller problem. Note though that too many points sent to the simulator will make the vehicle slower in reaction to sudden changes, why this needs to be tuned carefully.

### Create trajectory points to use in simulator
To create smooth trajectories splines are used by using implementation from http://kluge.in-chemnitz.de/opensource/spline/, this can be seen from line 260 in main.cpp.

### Tuning and motivation of chosen parameters
The parameters that were tuned were how many trajectory points to send to the simulator, the prediction horizon, the prediction step and safe distance to other vehicles (for distance buffer cost calculations).

The prediction horizon was designed to predict far enough to be able to detect collisions early, but with the care that it shall not be chosen too long since the uncertainty of what a target vehicle will do will increase for each time step further into the future. For the current implementation this was chosen to be 2.0s. 

The prediction step was chosen to be 0.4s, since we do not need to analyse all situations in detail but want to check roughly the future trajectory and possible dangers.

The number of trajectory points was designed (as mentioned before) with consideration that too many trajectory points will mean that we already have decided where we shall go in the future i.e. makes the system slow in reaction to changes, which could make us end up in a collision. But we need to make sure to not choose a too small number of points since this will make it possible for the system to change trajectory very often, which will make it jerky.

The buffer cost really helped the system to keep a good distance to targets even though the vehicles were driving in similar speeds. The distance parameter in this cost has a big influence on how early we will change lane and how close we will get to other vehicles if we cannot change lane. This was considered when tuning this parameter. 

### Future improvements
I am pretty satisfied with my result with a system that has been able to drive for above 30 miles, more or less 7 times as long as the project rubic, but there are always room for improvements.

First of all I would like to incorporate subject acceleration better into the model, now it is only one time step that will change speed... there is no constant acceleration. I am also thinking about using a solution where the whole trajectory can be changed instead of adding points to the end of the earlier trajectory. This would make the system react faster if there are sudden changes, if this is not taken care of by an Autonomous Emergency Braking system. In general I also think it could be good to add more possible trajectories.

The target accelerations should also be possible to take into consideration, maybe use some kalman filter to better track the vehicles, this would maybe give us the possibility to also detect possible lane change manouvres.

Last but not least, it would be good to define more cost functions e.g. for jerk and acceleration.


---
## Additional project information
Following is some information that can be good to know to run and modify the project.
   
### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Dependencies

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

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Input data
Here is the data provided from the Simulator to the C++ Program

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 10 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


### Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
