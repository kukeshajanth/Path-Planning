# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Rubric



- The code complies when the cmake and make is used.

-  The car drives at least 4.5 miles without any collision.

- The car doesn't cross the maximum speed limit i.e. 50MPH

- The car doesn't cross the max accleration and jerk limit.

- The car doesn't collides any car or avoid any traffic rules.

- The car stayed on the lane apart from the lane changing time.

- The car changes lane when car in front of it is slower and a relative smooth lane change can take place



## Prediction and Decision

This step analyzes the localization and sensor fusion data for all cars on the same side of the track, including our vehicle.

```
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
          
```


The positions of all the other vehicles are analyzed relative to our vehicle. If the our vehicle is within 27 meters of the vehicle in front, front vehicle is flagged true. If vehicles are within that margin on the left or right, car_left or car_right are flagged true, respectively.



```
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
          
 ```


Decisions are made on how to adjust speed and change lanes. If a car is ahead within the gap, the lanes to the left and right are checked. If one of them is empty, the car will change lanes. Otherwise it will slow down.

The car will move back to the center lane when it becomes clear. This is because a car can move both left and right from the center lane, and it is more likely to get stuck going slowly if on the far left or right.

If the area in front of the car is clear, no matter the lane, the car will speed up.

## Trajectory Generation

Compute the trajectory of the vehicle from the decisions made above, the vehicle's position, and historical path points. The last two points in the already-covered terrain are computed. If the vehicle previous path points are less than 2, the vehicle's current position is used instead of the historical waypoints. In addition, the Frenet helper function getXY() is used to generate three points spaced evenly at 30 meters in front of the car. 

```
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
          
```

Because splines are the method used to generate the trajectory, a shift and rotate transform is applied.

```
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
              
```

The computed waypoints are transformed using a spline. The spline makes it relatively easy to compute a smooth trajectory in 2D space while taking into account acceleration and velocity. 50 waypoints are generated in total. Because the length of the generated trajectory is variable, after the vehicle has assumed the correct position, the rest of the waypoints are generated to keep the vehicle in the target lane. This can be observed by watching the green trajectory line in front of the vehicle as a lane change occurs