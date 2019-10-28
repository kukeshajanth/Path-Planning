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
#include "helpers.h"
// for convenience
using json = nlohmann::json;
using std::string;
using std::vector;
using namespace std;

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
  
  int lane =1;
  
  //ref vel
  double ref_vel = 0.0;
  
  
  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          //previous path size
          
          int previous_path_size = previous_path_x.size();
          
          //storing end_path_s to car_s ,to get the continuity and avoid collision
          
          if (previous_path_size > 0 ){
            car_s = end_path_s;
          }

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          //
          bool front_car = false;
          bool right_car = false;
          bool left_car = false;
          
          for(int i = 0; i < sensor_fusion.size();i++){
            float d = sensor_fusion[i][6];
            int car_lane = -1;
            // to get the lane in which nearby car[i] is in
            if(d > 0 && d <4){
              car_lane = 0;
            }
            else if( d >4 && d<8){
              car_lane =1;
            }
            else if(d >8 && d<12){
              car_lane = 2;
            }
            if(car_lane<0){
              continue;
            }

          //speed of nearby car[i]
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_car_speed = sqrt(vx*vx + vy*vy);
            double check_car_s_val = sensor_fusion[i][5];
            
            check_car_s_val += ((double)previous_path_size*0.02*check_car_speed);

            
            if(car_lane == lane)
            {
              if(check_car_s_val > car_s && (check_car_s_val - car_s) <= 30)
              {
                front_car = true;
              }
            }
            else if(car_lane - lane == -1)
            {
              if(car_s - 17 <= check_car_s_val && (car_s + 27) >= check_car_s_val)
              {
                left_car = true;
              }
            }
            else if( car_lane - lane == 1)
            {
              if(car_s - 17 <= check_car_s_val && (car_s + 27) >= check_car_s_val)
              {
                right_car = true;
              }
            }
          }
            
           
            // Behavious planner
          double vel_diff = 0.0;
          const double max_vel = 49.5;
          const double max_acc = .224;
            
          // fronf car = true if there is a car at front
          if(front_car){
            if(!left_car && lane > 0)
            {
              // when there is no car to the left and it is not the left most lane
              lane -- ;
            }
            else if(!right_car && lane != 2)
            {
              // when there is no car to the right and it is not the right most lane
              lane ++;
            }
            else
            {
              //when there is cars i both lanes , slow down in the current lane
              vel_diff = -max_acc;
            }
          }
          else{
            //when there is no cars ahead
            if(lane!= 1)
            {
              if((lane == 0 && !right_car)|| (lane == 2 && !left_car))
              {
                lane = 1;
              }
            }
            if(ref_vel < max_vel)
            {
              vel_diff = max_acc;
            }
          }
            
          vector<double> ptsx;
          vector<double> ptsy;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(previous_path_size < 2){
            //using car as the starting point if previous size is almost empty
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            //adding x coordinates to ptsx
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            //adding y coordinates to ptsy
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else{
            //using previous points as references
            ref_x = previous_path_x[previous_path_size -1];
            ref_y =  previous_path_y[previous_path_size -1];
            double ref_x_prev = previous_path_x[previous_path_size - 2];
            double ref_y_prev = previous_path_y[previous_path_size - 2];
            
            ref_yaw = atan2(ref_y - ref_y_prev , ref_x - ref_x_prev);
            
            //using last points in the previous path
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          //adding 30m spaced waypoint with calculated origin
          vector<double> next_waypoint_0 = getXY(car_s + 30, (2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
          vector<double> next_waypoint_1 = getXY(car_s + 60, (2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
          vector<double> next_waypoint_2 = getXY(car_s + 90, (2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_waypoint_0[0]);
          ptsx.push_back(next_waypoint_1[0]);
          ptsx.push_back(next_waypoint_2[0]);
          
          ptsy.push_back(next_waypoint_0[1]);
          ptsy.push_back(next_waypoint_1[1]);
          ptsy.push_back(next_waypoint_2[1]);
          
          for( int i = 0 ; i < ptsx.size() ; i++){
            //changing to local oordinates for using in spline library
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x*cos(0- ref_yaw) - shift_y *sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0- ref_yaw) + shift_y *cos(0-ref_yaw));
            
          }
          
          
          
          //creating spline
          tk::spline s;
          //fitting points into the spline
          s.set_points(ptsx,ptsy);
          
          for(int i = 0; i < previous_path_size; i++){
            //pushin back previous path points 
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
                                  
          double add_on = 0;
            //limiting jerk by slowly accelarating and not exceeding max_accelaration             
          for(int i = 0; i <= 50- previous_path_size;i++){
            ref_vel += vel_diff;
            if( ref_vel > max_vel){
              ref_vel = max_vel ;
            }
            else if(ref_vel<= max_acc){
              ref_vel += max_acc;
            }
            
            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = add_on + target_x/N;
            double y_point = s(x_point);
            
            add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
              
          }   
    
          json msgJson;

          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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