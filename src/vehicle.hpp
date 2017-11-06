#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include <vector>
#include <cmath>

using namespace std;

class Vehicle {

  public:
    double car_x_;
    double car_y_;
    double ref_x_;
    double ref_y_;
    double car_s_;
    double car_d_;
    double car_yaw_; // radians in global coordinates
    double car_speed_; // in m/s
    vector<double> ptsx_; // in local vehicle coordinates
    vector<double> ptsy_; // in local vehicle coordinates
    double end_path_s_;
    double end_path_d_;
  
    Vehicle();
  
    ~Vehicle();

    void GetStartWaypoints(); // if no previous data
    void GetStartWaypoints(double ref_x, double ref_y, double ref_x_prev, double ref_y_prev);
    void GetWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, int lane);
    void ClearWaypoints();

    // HELPER FUNCTIONS
    // For converting back and forth between radians and degrees.
    double pi() { return M_PI; }
    double deg2rad(double x) { return x * pi() / 180; }
    double rad2deg(double x) { return x * 180 / pi(); }
  
    double distance(double x1, double y1, double x2, double y2);
    int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
    int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
    vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
    vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

};
#endif