/*
 * cost_functions.hpp
 */
 

/* IDEA?
Predict target vehicle trajectories T (e.g. 3s) ahead.
Use safety margins (we do not know the size of the vehicles)
[if target speed > speed limit, do not change lane]
Generate some trajectories straight, turn left, turn right (these will generate different speed, jerk etc.)
- straight keeping speed limit
- if target ahead, trajectory with a end position at a safe distance behind the vehicle.
- trajectory turing to the right and left
- if target in lane to the left/right, trajectories on a safe distance to targets (ahead/behind)
Check that trajectories are within limits (jerk, acceleration etc)
Calculate cost for each trajectory
Choose the one with lowest cost
*/

#include <cmath>
#include "vehicle.hpp"
#include "spline.h"

// priority levels for costs
extern const double COLLISION  = pow(10, 6);
extern const double LEGAL      = pow(10, 5);
extern const double DANGER     = pow(10, 5);
extern const double REACH_GOAL = pow(10, 5);
extern const double COMFORT    = pow(10, 5);
extern const double EFFICIENCY = pow(10, 3);


const double DESIRED_BUFFER = 2/0.2; // time steps


double legalSpeedCost(double subjectSpeed, double speedLimit){ //keep our speed below speed limit
  
  double diff = speedLimit - subjectSpeed;
  double pct = diff / speedLimit;
  double multiplier = pow(pct,2);
  return multiplier * LEGAL;
}

// add something to keep speed close to target speed -> or solve by using cost function for distance to Target as big as possible?

double efficiencySpeedCost(double subjectSpeed, double speedLimit){ // keep our speed as close as possible to speed limit of road
  double diff = subjectSpeed - speedLimit;  
  double pct = diff / speedLimit;
  double multiplier;
  if(pct < - 0.5 || pct > 0){
    multiplier = 1; //maybe make a complex function later...
  } else{
    multiplier = 0;
  }
  //multiplier = pct ** 2*/
  return multiplier * EFFICIENCY;

}

double collisionCost(vector<vector<vector<double>>> targets, vector<vector<double>> subjectTrajectory, double T, double dt){ // make sure to not collide with target
  // 1. check for each time step for prediction horizon how close vehicles + safe distance (vehicle size) is to end up in collision
  // 2. check for shortest time to collision during whole prediction horizon
  // 3. cost = cost for shortest time to collision
  
  double length = 4; //approximate vehicle length 
  double width = 1; // approximate vehicle width

  int N = static_cast<int>( T/dt );
  // check when, if any, collision is present
  for(int i = 0; i <= N; i++){        
    double subject_y = subjectTrajectory[1][i];
    double subject_x = subjectTrajectory[0][i];
    
    /*if(i==0){
      std::cout << "SUBJECT (x,y) = ( " << subject_x << ", " << subject_y << " )\n";
      std::cout << "TARGETS \n";
    }*/
   
    // for each target
    for(int j = 0; j < targets.size(); j++){
      vector<vector<double>> target = targets[j];
      vector<double> target_xs = target[0];
      vector<double> target_ys = target[1];
      double target_x = target_xs[i];
      double target_y = target_ys[i];
      /*if(i==0){
        std::cout << "(x,y) = ( " << target_x << ", " << target_y << " )\n";
      }*/

      // check collision, using whole length and with for safety measures
      //d::cout << "distance between vehicles, x: " << target_x - subject_x << ", y: " << target_y - subject_y << "\n";
      if( -length <= target_x - subject_x && target_x - subject_x <= length && -width <= target_y - subject_y && target_y - subject_y <= width ){
        std::cout << "COLLISION!! \n";
        double TTC = i*dt;
        double multiplier = exp(-pow(TTC,2));
        return multiplier * COLLISION;
      }
    }    
  }
  // no collision found, return cost 0
  return 0;
}

double jerkAccelerationCost(vector<vector<double>> subjectTrajectory, double T, double dt){
  int N = static_cast<int>( T/dt );
  // check jerk values
  double totalAcceleration = 0;
  double totalJerk = 0;
  double previous_y = subjectTrajectory[0][0];
  double y;
  double previous_vy;
  double previousAcceleration;
  //double previous_x = subjectTrajectory[0][0];
  for(int i = 1; i <= N; i++){        
    //double x = subjectTrajectory[0][i];
    y = subjectTrajectory[0][i];
    double vy = (y-previous_y)/dt;
    if(i > 2){
      double acceleration = (vy-previous_vy)/dt;
      totalAcceleration += pow(acceleration,2);
      if(i > 3){
      totalJerk += pow((acceleration-previousAcceleration)/dt,2);
      }
      previousAcceleration = acceleration;
    }    
    previous_y = y;
    previous_vy = vy;
  }
  return (totalAcceleration + totalJerk) * COMFORT; 
}

/* distanceBufferCost(vehicle, trajectory, predictions, data){ // make sure to keep a safe distance to targets ahead
  // 1. check in each time step for prediction horizon where both vehicles will be
  // 2. save check what is the shortest distance to target during whole prediction horizon
  // 3. Check which cost this distance gives


  double d_closest = data.closest_approach // distance to target that is in our lane and closests to us (get as argument?)
  if(d_closest < 3 && d_closest > -3) //some buffer for vehicle size
    return 10 * DANGER;

  double timesteps_away = d_closest / data.avg_speed
  if(timesteps_away > DESIRED_BUFFER)
    return 0.0
  
  double multiplier = 1.0 - pow((timesteps_away / DESIRED_BUFFER), 2);
  return multiplier * DANGER;
}*/

vector<vector<double>> getTrajectory(tk::spline splineTrajectory, Vehicle subject, double T, double ref_vel, double dt){
  //double dt = 0.02;
  double subject_x = 0;
  double subject_y = 0;
  
  vector<vector<double>> subjectTrajectory;  
  vector<double> subjectTrajectory_x;
  vector<double> subjectTrajectory_y;
  subjectTrajectory_x.push_back(subject_x);
  subjectTrajectory_y.push_back(subject_y);
  int N = static_cast<int> (T/dt);
  double yaw = subject.car_yaw_;
  
  
  for(int i = 1; i <= N; i++){        
    // estimate position of subject
    subject_x += ref_vel*cos(yaw)*dt; 
    subject_y = splineTrajectory(subject_x);
    subjectTrajectory_x.push_back(subject_x);
    subjectTrajectory_y.push_back(subject_y);    
  }
  subjectTrajectory.push_back(subjectTrajectory_x);
  subjectTrajectory.push_back(subjectTrajectory_y);
  return subjectTrajectory;
}

double calculate_cost(vector<vector<vector<double>>> targets, tk::spline splineTrajectory, Vehicle vehicle, double speedLimit, double ref_vel,
                      double predictionHorizon, double dt){
  vector<vector<double>> subjectTrajectory = getTrajectory(splineTrajectory, vehicle, predictionHorizon, ref_vel, dt);
  double cost = 0.0;
  cost += legalSpeedCost(ref_vel, speedLimit);
  cost += efficiencySpeedCost(ref_vel, speedLimit);
  cost += collisionCost(targets, subjectTrajectory, predictionHorizon, dt);
  cost += jerkAccelerationCost(subjectTrajectory, predictionHorizon, dt);
  //cost += distanceBufferCost(...);
  // cost += ...
    
  return cost;
}


// add more cost functions
// minimize jerk dep on vehicle limitations and comfort limitations (especially lateral jerk)
// minimize acceleration based on vehicle limitations (and comfort?)
