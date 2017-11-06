#include "vehicle.hpp"

using namespace std;

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::GetStartWaypoints(){

  // set yaw to radians
  ref_x_ = car_x_;
  ref_y_ = car_y_;
  car_yaw_ = deg2rad(car_yaw_);

  // previous state is almost empty, use the car as a starting reference
  // use 2 points that makes the path tagent to the car
  double prev_car_x = car_x_ - cos(car_yaw_);
  double prev_car_y = car_y_ - sin(car_yaw_);

  ptsx_.push_back(prev_car_x);
  ptsx_.push_back(car_x_);

  ptsy_.push_back(prev_car_y);
  ptsy_.push_back(car_y_);

}

void Vehicle::GetStartWaypoints(double ref_x, double ref_y, double ref_x_prev, double ref_y_prev){
 
  // use the previous path's end point as starting reference
  ref_x_ = ref_x;
  ref_y_ = ref_y;
  car_yaw_ = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

  // use 2 points that make the path tagent to the previous path's end points
  ptsx_.push_back(ref_x_prev);
  ptsx_.push_back(ref_x);

  ptsy_.push_back(ref_y_prev);
  ptsy_.push_back(ref_y);

}

void Vehicle::GetWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, 
                                    int lane){                               
  for(int i = 30; i <= 90; i+=30) // TODO: maybe update this to depend on speed
  {
    double next_s = car_s_ + i;
    double next_d = 2 + 4*lane; // we want to stay in the middle of the lane (lane width = 4)
    vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    
    ptsx_.push_back(xy[0]);
    ptsy_.push_back(xy[1]);
  }

  // shift to local vehicle coordinates
  for(int i = 0; i < ptsx_.size(); i++){
    double shift_x = ptsx_[i]-ref_x_;
    double shift_y = ptsy_[i]-ref_y_;
    
    ptsx_[i] = shift_x*cos(0-car_yaw_) - shift_y*sin(0-car_yaw_);
    ptsy_[i] = shift_x*sin(0-car_yaw_) + shift_y*cos(0-car_yaw_);
  }                                    
}                                    

void Vehicle::ClearWaypoints(){
  ptsx_.clear();
  ptsy_.clear();
}


// HELPER FUNCTIONS FOR VEHICLE

double Vehicle::distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Vehicle::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int Vehicle::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Vehicle::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Vehicle::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

