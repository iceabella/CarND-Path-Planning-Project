#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "cost_functions.hpp"
#include "vehicle.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  // start in lane 1
  int lane = 1;
  
  int count = 0;
  
  // have a reference velocity below speed limit
  double ref_vel = 0; //m/s, we start at 0
  
  // create vehicle for to keep track of subject vehicle data
  Vehicle subject;

  h.onMessage([&ref_vel, &lane, &subject, &count, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          //std::cout << "new data \n";
          std::cout << count << "\n";
          count++;
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          int prev_size = previous_path_x.size();

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // update subject vehicle data
          subject.car_x_ = car_x;
          subject.car_y_ = car_y;
          subject.car_s_ = car_s;
          subject.car_d_ = car_d;
          subject.car_yaw_ = car_yaw;
          subject.car_speed_ = car_speed*1.6/3.6; //mph to m/s
          subject.end_path_s_ = end_path_s;
          subject.end_path_d_ = end_path_d;

          if(prev_size > 0){
            subject.car_s_ = end_path_s;          	
          }        	
         

          // 1. CHECK IMPOSSIBLE NEW STATES (because of road restrictions)
          bool changeLeft = true;
          bool changeRight = true;
          if(lane == 0)
            changeLeft = false;
          else if(lane == 2)
            changeRight = false;

          // 2. PREDICT TRAJECTORIES FOR TARGETS
          double T = 3.0; // predict 3s into the future
          double dt = 0.4; // predcition step size 0.4s
          double speedLim = 22; // m/s -> ~49mph
          vector<vector<vector<double>>> targets_data;
          for(int i=0; i<sensor_fusion.size(); i++){
            double vx = sensor_fusion[i][3]; // m/s
            double vy = sensor_fusion[i][4]; // m/s
            //double check_speed = sqrt(vx*vx+vy*vy);
            double target_x = sensor_fusion[i][1];
            double target_y = sensor_fusion[i][2];
            //vector<double> targetTrajectory_s;
            //vector<double> targetTrajectory_d;
            vector<double> targetTrajectory_x;
            vector<double> targetTrajectory_y;            
            targetTrajectory_x.push_back(target_x);
            targetTrajectory_y.push_back(target_y);

            //targetTrajectory_s.push_back(sensor_fusion[i][5]);
            //targetTrajectory_d.push_back(sensor_fusion[i][6]);
            // predict future target position
            for(double j=0; j < T; j+=dt){
              target_x += vx*dt;
              target_y += vy*dt;
              targetTrajectory_x.push_back(target_x);
              targetTrajectory_y.push_back(target_y);
              }
              // shift to subject vehicle's coordinate system
              double target_heading = atan2(vy,vx);
              for(int j = 0; j < targetTrajectory_x.size(); j++){
                double shift_x = targetTrajectory_x[j]-subject.car_x_;
                double shift_y = targetTrajectory_y[j]-subject.car_y_;

                targetTrajectory_x[j] = shift_x*cos(target_heading-car_yaw) - shift_y*sin(target_heading-car_yaw);
                targetTrajectory_y[j] = shift_x*sin(target_heading-car_yaw) + shift_y*cos(target_heading-car_yaw);
              }                          
            vector<vector<double>> targetNTrajectory;
            targetNTrajectory.push_back(targetTrajectory_x);
            targetNTrajectory.push_back(targetTrajectory_y);
            targets_data.push_back(targetNTrajectory);
          }  

          // 3. CREATE EGO TRAJECTORIES FOR EACH POSSIBLE STATE and get cost
          // start with 1 trajectory per state maybe increase later

          //clear earlier waypoints
          subject.ClearWaypoints();
          
          // if previous state is almost empty, use the car as a starting reference
          if(prev_size < 2){
            // get new waypoints
            subject.GetStartWaypoints();
          }
          // use the previous path's end point as starting reference
          else{
            double ref_x = previous_path_x[prev_size-1];
            double ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            // get new waypoints
            subject.GetStartWaypoints(ref_x, ref_y, ref_x_prev, ref_y_prev);
          }

          // STATE: keep lane and speed
          subject.GetWaypoints(map_waypoints_s, map_waypoints_x, map_waypoints_y, lane);

          // create a spline
          tk::spline s_keepLane;
          // set x,y points to the spline
          std::cout << "s_keepLane \n";
          s_keepLane.set_points(subject.ptsx_,subject.ptsy_);

          double cost_keepLaneSpeed = calculate_cost(targets_data, s_keepLane, subject, speedLim, ref_vel, T, dt);  	  

          // STATE: decelerate in same lane
          // use same points as keeplane but lower speed

          double ref_velDecelerate = ref_vel - 0.112; // m/s
          if(ref_velDecelerate < 0){
            ref_velDecelerate = 0;
          }
          double cost_keepLaneDec = calculate_cost(targets_data, s_keepLane, subject, speedLim, ref_velDecelerate,T, dt);      	  

          // STATE: accelerate in same lane
          double ref_velAccelerate = ref_vel + 0.134; // m/s
          double cost_keepLaneAcc = calculate_cost(targets_data, s_keepLane, subject, speedLim, ref_velAccelerate, T, dt);

          // STATE: keep same speed as target in same lane


          // STATE: lane change right
          // create a spline
          tk::spline s_changeRight;
          double cost_changeRight = -1;
          if(changeRight){
            // reset ptsx and ptsy
            double start_ptsx0 = subject.ptsx_[0];
            double start_ptsy0 = subject.ptsy_[0];
            double start_ptsx1 = subject.ptsx_[1];
            double start_ptsy1 = subject.ptsy_[1];

            subject.ClearWaypoints();
            subject.ptsx_.push_back(start_ptsx0);
            subject.ptsx_.push_back(start_ptsx1);
            subject.ptsy_.push_back(start_ptsy0);
            subject.ptsy_.push_back(start_ptsy1);

            subject.GetWaypoints(map_waypoints_s, map_waypoints_x, map_waypoints_y, lane+1); // change lane to the right

            // set x,y points to the spline
            std::cout << "s_changeRight \n";
            s_changeRight.set_points(subject.ptsx_, subject.ptsy_);  	

            cost_changeRight = calculate_cost(targets_data, s_changeRight, subject, speedLim, ref_vel, T, dt);
          }

          // STATE: lane change left
          // create a spline
          tk::spline s_changeLeft;
          double cost_changeLeft = -1; 
          if(changeLeft){
            // reset ptsx and ptsy
            double start_ptsx0 = subject.ptsx_[0];
            double start_ptsy0 = subject.ptsy_[0];
            double start_ptsx1 = subject.ptsx_[1];
            double start_ptsy1 = subject.ptsy_[1];

            subject.ClearWaypoints();
            subject.ptsx_.push_back(start_ptsx0);
            subject.ptsx_.push_back(start_ptsx1);
            subject.ptsy_.push_back(start_ptsy0);
            subject.ptsy_.push_back(start_ptsy1);
            if(count == 80 || count == 79){
              std::cout << subject.ptsx_[0] << "\n";
              std::cout << subject.ptsx_[1] << "\n";
            }

            subject.GetWaypoints(map_waypoints_s, map_waypoints_x, map_waypoints_y, lane-1); // change lane to the right

            // set x,y points to the spline
            std::cout << "s_changeLeft \n";
            if(count == 80 || count == 79){
              for(int i = 0; i < subject.ptsx_.size(); i++)
                std::cout << subject.ptsx_[i] << "\n";
            }
            s_changeLeft.set_points(subject.ptsx_,subject.ptsy_);  	
            std::cout << "s_changeLeft \n";

            cost_changeLeft = calculate_cost(targets_data, s_changeLeft, subject, speedLim, ref_vel, T, dt);       	 
          }

          // 5. CHOOSE TRAJECTORY WITH LOWEST COST
          const int KEEPLANESPEED = 0;
          const int KEEPLANEDEC = 1;
          const int KEEPLANEACC = 2;
          const int CHANGERIGHT = 3;
          const int CHANGELEFT = 4;

          double minCost = -1;
          int minCost_Id = -1;
          vector<double> costs = {cost_keepLaneSpeed, cost_keepLaneDec, cost_keepLaneAcc, cost_changeRight, cost_changeLeft};
          for(int i = 0; i < costs.size(); i++){

            if((costs[i] < minCost && costs[i] > -1) || minCost_Id == -1){
              minCost = costs[i];
              minCost_Id = i;
            }
          }

          tk::spline s_minCost;
          switch(minCost_Id){
            case KEEPLANESPEED:
              s_minCost = s_keepLane;
              break;
            case KEEPLANEDEC:
            {
              s_minCost = s_keepLane;
              ref_vel = ref_velDecelerate;
            }
            break;
            case KEEPLANEACC:
            {
              s_minCost = s_keepLane;
              ref_vel = ref_velAccelerate;
            }
            break;
            case CHANGERIGHT:
              s_minCost = s_changeRight;
              lane++;
              break;
            case CHANGELEFT:
              s_minCost = s_changeLeft;
              lane--;
              break;                  
          }
          std::cout <<"chosen state: " << minCost_Id << "\n";

          // define the actual x,y points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all of the previous path points from last time
          for(int i=0; i<previous_path_x.size();i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so that  we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s_minCost(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          // fill up the rest of our path planner with new points, total number of points will here always be 50
          for(int i=1;i<= 50-previous_path_x.size();i++){
            double N = target_dist/(0.02*ref_vel); // 0.02s = 20ms -> update speed
            double x_point = x_add_on + target_x/N;
            double y_point = s_minCost(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // shift back to global coordinates
            x_point = x_ref*cos(subject.car_yaw_) - y_ref*sin(subject.car_yaw_);
            y_point = x_ref*sin(subject.car_yaw_) + y_ref*cos(subject.car_yaw_);

            x_point += subject.ref_x_;
            y_point += subject.ref_y_;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // END

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;



          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
