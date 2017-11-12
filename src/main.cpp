#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper_functions.hpp"
#include "cost_functions.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

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
  
  // have a reference velocity below speed limit
  double ref_vel = 0; //mph, we start at 0
  
  int count = 0; // to make sure that we do not fill out first states with 0s

  h.onMessage([&count, &ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            car_speed = car_speed*0.44704; //mph to m/s

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

          	// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	
          	if(prev_size >0){
          	  car_s = end_path_s;  
              car_d = end_path_d;
          	}
          
          //***************** TUNABLE PARAMS *******************
          double T = 2.0; // predict 2s into the future
          double dt = 0.4; // prediction step size 0.4
          double speedLim = 22.3; // m/s -> ~49.8 mph

          // 1. CHECK IMPOSSIBLE NEW STATES (because of road restrictions)
          bool changeLeft = true;
          bool changeRight = true;
          if(lane == 0)
            changeLeft = false;
          else if(lane == 2)
            changeRight = false;
          
          // 2. PREDICT TRAJECTORIES FOR TARGETS
          double real_dt = 0.02; // step size used in simulation
          vector<vector<vector<double>>> targets_data;
          for(int i=0; i<sensor_fusion.size(); i++){
            double vx = sensor_fusion[i][3]; // m/s
            double vy = sensor_fusion[i][4]; // m/s
            double target_v = sqrt(vx*vx+vy*vy);

            double target_s = sensor_fusion[i][5];
            double target_d = sensor_fusion[i][6];
            vector<double> targetTrajectory_s;
            vector<double> targetTrajectory_lane;
            vector<double> targetTrajectory_velocity;
            double target_lane;
            // which lane is the target in? 
          	  if(target_d <= 4){
                target_lane = 0; // left lane         	    	                	      
          	  }
          	  else if(4.0 < target_d && target_d <= 8.0){
                target_lane = 1; // middle lane
          	  } else if(8.0 < target_d){
                target_lane = 2; // right lane
          	  }
            
            // we will predict subject position from end of last trajectory point sent to simulator
            // -> start predition for target shall start at length(sent subject points to simulator)
            target_s += target_v*prev_size*real_dt;

            targetTrajectory_s.push_back(target_s - car_s);
            targetTrajectory_lane.push_back(target_lane);
            targetTrajectory_velocity.push_back(target_v);

            // predict future target position
            for(double j=dt; j <= T; j+=dt){
              target_s += target_v*dt; // assuming keeping lane 
              // assuming target keeps lane and speed and shift to subject vehicle's coordinates
              targetTrajectory_s.push_back(target_s - car_s);
              targetTrajectory_lane.push_back(target_lane);
              targetTrajectory_velocity.push_back(target_v);
              }
                       
            vector<vector<double>> targetNTrajectory;
            targetNTrajectory.push_back(targetTrajectory_s);
            targetNTrajectory.push_back(targetTrajectory_lane);
            targetNTrajectory.push_back(targetTrajectory_velocity);
            targets_data.push_back(targetNTrajectory);
          }
          
          
          // 3. GET COST FOR EACH SUBJECT TRAJECTORY

          // STATE: keep lane and speed
          double cost_keepLaneSpeed = calculate_cost(targets_data, lane, speedLim, ref_vel, T, dt);
          std::cout << "cost_keepLaneSpeed = " << cost_keepLaneSpeed << "\n";

          // STATE: decelerate in same lane
          double ref_velDecelerate = ref_vel - 3*dt*dt; // m/s (a = -3m/s²)
          if(ref_velDecelerate < 0){
            ref_velDecelerate = 0;
          }
          double cost_keepLaneDec = calculate_cost(targets_data, lane, speedLim, ref_velDecelerate, T, dt);
          std::cout << "cost_keepLaneDec = " << cost_keepLaneDec << "\n";
          
          // STATE: decelerate hard in same lane 
          double ref_velDecelerateHard = ref_vel - 5*dt*dt; // m/s (a = -5m/s²)
          if(ref_velDecelerateHard < 0){
            ref_velDecelerateHard = 0;
          }
          double cost_keepLaneDecHard= calculate_cost(targets_data, lane, speedLim, ref_velDecelerateHard, T, dt);
          std::cout << "cost_keepLaneDecHard = " << cost_keepLaneDecHard << "\n";

          // STATE: accelerate in same lane
          double ref_velAccelerate = ref_vel + 3*dt*dt; // m/s (a = 3 m/s²)
          double cost_keepLaneAcc = calculate_cost(targets_data, lane, speedLim, ref_velAccelerate, T, dt);          
          std::cout << "cost_keepLaneAcc = " << cost_keepLaneAcc << "\n";

          // STATE: lane change right
          double cost_changeRight = -1;
          if(changeRight){
            cost_changeRight = calculate_cost(targets_data, lane+1, speedLim, ref_vel, T, dt);
            std::cout << "cost_changeRight = " << cost_changeRight << "\n";
          }

          // STATE: lane change left
          double cost_changeLeft = -1; 
          if(changeLeft){         
            cost_changeLeft = calculate_cost(targets_data, lane-1, speedLim, ref_vel, T, dt);
            std::cout << "cost_changeLeft = " << cost_changeLeft << "\n";
          }

          // 4. CHOOSE TRAJECTORY WITH LOWEST COST
          const int KEEPLANESPEED = 0;
          const int KEEPLANEDEC = 1;
          const int KEEPLANEACC = 2;
          const int CHANGERIGHT = 3;
          const int CHANGELEFT = 4;
          const int KEEPLANEDECHARD = 5;

          double minCost = -1;
          int minCost_Id = -1;
          
          vector<double> costs = {cost_keepLaneSpeed, cost_keepLaneDec, cost_keepLaneAcc, cost_changeRight, cost_changeLeft, cost_keepLaneDecHard};
          for(int i = 0; i < costs.size(); i++){
            if((costs[i] < minCost && costs[i] > -1) || minCost_Id == -1){
              minCost = costs[i];
              minCost_Id = i;
            }
          }
          
          if(count < 2){
            ref_vel = ref_velAccelerate;
          }else{           
            switch(minCost_Id){
              case KEEPLANEDEC:
              {
                ref_vel = ref_velDecelerate;
              }
              break;
              case KEEPLANEDECHARD:
                ref_vel = ref_velDecelerateHard;
                break;
              case KEEPLANESPEED:
                break;
              case KEEPLANEACC:
              {
                ref_vel = ref_velAccelerate;
              }
              break;
              case CHANGERIGHT:
                lane++;
                break;
              case CHANGELEFT:
                lane--;
                break;                  
            }
          }
          count++;
            
          std::cout <<"chosen state: " << minCost_Id << "\n\n";


          // 5. CREATE TRAJECTORY POINTS TO USE IN SIMULATOR         	
          // list of widely spaced waypoints, evenly spaced at 30m
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y,yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous state is almost empty, use the car as a starting reference
          if(prev_size < 2){
            // use 2 points that makes the path tagent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's end point as starting reference
          else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            // use 2 points that make the path tagent to the previous path's end points
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }


          for(int i = 30; i <= 90; i+=30)
          {
            double next_s = car_s + i;
            double next_d = 2 + 4*lane; // we want to stay in the middle of the lane (lane width = 4)
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(xy[0]);
            ptsy.push_back(xy[1]);
          }

          // shift to local vehicle coordinates
          for(int i = 0; i < ptsx.size(); i++){
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
          }

          // create a spline
          tk::spline s;
          // set x,y points to the spline
          s.set_points(ptsx,ptsy);

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
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          double numberTrajectoryPoints = 50; // numberTrajectoryPoints*real_dt = prediction horizon

          // fill up the rest of our path planner with new points, total number of points will here always be numberTrajectoryPoints
          for(int i=1;i<= numberTrajectoryPoints-previous_path_x.size();i++){
            double N = target_dist/(real_dt*ref_vel); // 0.02s = 20ms -> update speed, 2.24 mph -> m/s
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // shift back to global coordinates
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


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
