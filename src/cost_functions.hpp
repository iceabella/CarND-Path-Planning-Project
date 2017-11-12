/*
 * cost_functions.hpp
 */


#include <cmath>

// priority levels for costs
extern const double COLLISION  = pow(10, 6);
extern const double LEGAL      = pow(10, 5);
extern const double DANGER     = pow(10, 5);
extern const double COMFORT    = pow(10, 4);
extern const double EFFICIENCY = pow(10, 2);


double legalSpeedCost(double subjectSpeed, double speedLimit){ //keep our speed below speed limit
  
  double diff = speedLimit - subjectSpeed;
  double pct = diff / speedLimit;
  double multiplier = pow(pct,2);
  return multiplier * LEGAL;
}


double efficiencySpeedCost(double subjectSpeed, double speedLimit){ // keep our speed as close as possible to speed limit of road
  double diff = subjectSpeed - speedLimit;  
  double pct = diff / speedLimit;
  double multiplier;
  if(pct < - 0.5 || pct > 0){
    multiplier = 1; //maybe make a complex function later...
  } else{
    multiplier = 0;
  }
  return multiplier * EFFICIENCY;

}


double collisionCost(vector<vector<vector<double>>> targets, int ref_lane, double ref_vel, double T, double dt){ // make sure to not collide with target
  // 1. check for each time step for prediction horizon how close vehicles (vehicle size) is to end up in collision
  // 2. check for shortest time to collision during prediction horizon
  // 3. cost = cost for shortest time to collision
  
  const double length = 6; //approximate vehicle length 

  int N = static_cast<int>( T/dt );
  double subject_s;
  // check when, if any, collision is present
  for(int i = 0; i <= N; i++){        
    subject_s = ref_vel*dt*i;
   
    // for each target
    for(int j = 0; j < targets.size(); j++){
      vector<vector<double>> target = targets[j];
      vector<double> target_ss = target[0];
      vector<double> target_lanes = target[1];
      double target_s = target_ss[i];
      double target_lane = target_lanes[i];

      // check collision, using length and safety measures
      if(target_lane == ref_lane){ // target is driving in our trajectory lane
        if(-length/2 <= target_s - subject_s && target_s - subject_s <= length){
          double TTC = i*dt;
          //std::cout << "COLLISION at " << TTC << "\n";
          double multiplier = exp(-pow(TTC,2));
          return multiplier * COLLISION;
        }
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

  for(int i = 1; i <= N; i++){        
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

double distanceBufferCost(vector<vector<vector<double>>> targets, int ref_lane, double ref_vel, double T, double dt){ // make sure to keep a safe distance to targets ahead
  // 1. check in each time step for prediction horizon where both vehicles will be
  // 2. check what is the shortest distance to target during whole prediction horizon
  // 3. Check which cost this distance gives
  
  const double desiredTime = 2.0; // s
  const double minimumDistance = 10; // m
 
  double closestDistance = 2000; // m
  double signClosestDistance = 0;
  double relativeVel = 0.01; // m/s
  
  int N = static_cast<int>( T/dt );
  double subject_s;
  // check when, if any, collision is present
  for(int i = 1; i <= N; i++){        
    subject_s = ref_vel*dt*i;
   
    // for each target
    for(int j = 0; j < targets.size(); j++){
      vector<vector<double>> target = targets[j];
      vector<double> target_ss = target[0];
      vector<double> target_lanes = target[1];
      vector<double> target_vels = target[2];
      double target_s = target_ss[i];
      double target_lane = target_lanes[i];
      double target_vel = target_vels[i];

      // check distance to target
      if(target_lane == ref_lane){ // target is driving in our trajectory lane
        if(-closestDistance <= abs(target_s - subject_s) && abs(target_s - subject_s) <= closestDistance){
          closestDistance = abs(target_s - subject_s);
          signClosestDistance = (((target_s - subject_s)<0) ? -1 : 1);
          relativeVel = ref_vel - target_vel; // if the target is ahead of us (>0) and we have a higher velocity(>0) we will collide
        }
      }
    }    
  }
  
  if(relativeVel == 0){
    relativeVel = 0.01;
  }

  if(-minimumDistance/2 < signClosestDistance*closestDistance && signClosestDistance*closestDistance < minimumDistance) 
    return 10 * DANGER;

  double timeAway = signClosestDistance*closestDistance / relativeVel;
  if(timeAway > desiredTime || timeAway < 0){ // if time away is < 0 the distance between vehicles are increasing
    return 0.0;
  }
  
  double multiplier = 1.0 - pow((timeAway / desiredTime), 2);
  return multiplier * DANGER;
}



double calculate_cost(vector<vector<vector<double>>> targets, int ref_lane, double speedLimit, double ref_vel, double predictionHorizon, double dt){
  double cost = 0.0;
  cost += legalSpeedCost(ref_vel, speedLimit);
  cost += efficiencySpeedCost(ref_vel, speedLimit);
  cost += collisionCost(targets, ref_lane, ref_vel, predictionHorizon, dt);
  //cost += jerkAccelerationCost(subjectTrajectory, predictionHorizon, dt); // maybe add lane to vehicle and check if lane > or < vehicle.lane_ add some extra cost
  cost += distanceBufferCost(targets, ref_lane, ref_vel, predictionHorizon, dt);
  // cost += ...
    
  return cost;
}


// add more cost functions
// minimize jerk dep on vehicle limitations and comfort limitations (especially lateral jerk)
// minimize acceleration based on vehicle limitations (and comfort?)
