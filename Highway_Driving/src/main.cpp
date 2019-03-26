#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // have a reference velocity to target
  double ref_vel = 0; //mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          auto prev_size = previous_path_x.size();

          // SENSOR FUSION
					bool too_close = false;
					bool tooCloseMiddle = false;
					//detect collision in middle lane
					for (int i = 0; i < sensor_fusion.size(); ++i) {
						// car is in my lane
						float d = sensor_fusion[i][6];
						if (d < 8 && d > 4 && lane == 1) {           //my lane is 1 and d vlue in the map is 4-8
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx * vx + vy * vy);
							double check_car_s = sensor_fusion[i][5];

							check_car_s += (static_cast<double>(prev_size) * .02 * check_speed);
							if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
								cout << "Too close condition in middle lane reached" << endl;
								tooCloseMiddle = true;
								

							}

						}
					}
					bool tooCloseRight = false;
					//detect collision in right lane
					for (int i = 0; i < sensor_fusion.size(); ++i) {
						// car is in my lane
						float d = sensor_fusion[i][6];

						if (d < 12 && d > 8 && lane == 2) {          //my lane is 2 and d vlue in the map is 8-12
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx * vx + vy * vy);
							double check_car_s = sensor_fusion[i][5];

							check_car_s += (static_cast<double>(prev_size) * .02 * check_speed);
							if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
								cout << "Too close condition in right lane reached" << endl;
								tooCloseRight = true;
								
							}

						}
					}

					bool tooCloseleft = false;
					//detect collision in left lane
					for (int i = 0; i < sensor_fusion.size(); ++i) {
						// car is in my lane
						float d = sensor_fusion[i][6];

						if (d < 4 && d > 0 && lane == 0) {           ////my lane is 0 and d vlue in the map is 0-4
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx * vx + vy * vy);
							double check_car_s = sensor_fusion[i][5];

							check_car_s += (static_cast<double>(prev_size) * .02 * check_speed);
							if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
								cout << "Too close condition in left lane reached" << endl;
								tooCloseleft = true;
								break;

							}

						}

					}
					too_close = tooCloseLeft | tooCloseMiddle | tooCloseRight;
					// Check if left lane is free
					bool leftChange = false;
					for (int i = 0; (i < sensor_fusion.size()) && too_close == true; ++i) {
						// car is in my lane
						float d = sensor_fusion[i][6];
						if (d < 4 && d > 0 && tooCloseLeft != true) {
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx * vx + vy * vy);
							double check_car_s = sensor_fusion[i][5];

							check_car_s += (static_cast<double>(prev_size) * .02 * check_speed); // if using previous points ca project s value out
							if ((abs(check_car_s - car_s)) > 30) {
								leftChange = true;
								cout << "Left change possible" << endl;
								//break;
							}
						}
					}
					// Check if right lane is free
					bool rightChange = false;
					for (int i = 0; (i < sensor_fusion.size()) && too_close == true; ++i) {
						// car is in my lane
						float d = sensor_fusion[i][6];

						if (d < 12 && d > 8 && tooCloseRight != true) {

							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx * vx + vy * vy);
							double check_car_s = sensor_fusion[i][5];

							check_car_s += (static_cast<double>(prev_size) * .02 * check_speed);
							if ((abs(check_car_s - car_s)) > 30) {
								rightChange = true;
								cout << "Right change possible" << endl;
								//break;
							}
						}
					}
					// Check if middle lane is free
					bool middleChange = false;
					for (int i = 0; (i < sensor_fusion.size()) && too_close == true; ++i) {
						// car is in my lane
						float d = sensor_fusion[i][6];

						if (d < 8 && d > 4 && tooCloseMiddle != true) {

							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx * vx + vy * vy);
							double check_car_s = sensor_fusion[i][5];

							check_car_s += (static_cast<double>(prev_size) * .02 * check_speed);
							if ((abs(check_car_s - car_s)) > 30) {
								middleChange = true;
								cout << "middle change possible" << endl;
								//break;
							}
						}
					}
					cout << "Lane Change Status Left Free | " << leftChange << " Middle Free | " << middleChange << " Right Free  | " << rightChange << endl;
					bool lanechanged = false;
					if (too_close == true && lane == 0 && middleChange == true) {
						cout << "Condition suitable for lance change left to 1" << endl;
						lane = 1;
						lanechanged = true;
						//too_close = false;
					}

					if (too_close == true && lane == 1 && leftChange == true && lanechanged != true) {
						cout << "Condition suitable for lance change left to 0" << endl;
						lane = 0;
						lanechanged = true;
						//too_close = false;
					}

					if (too_close == true && lane == 1 && rightChange == true && lanechanged != true) {
						cout << "Condition suitable for lance change right to 2" << endl;
						lane = 2;
						lanechanged = true;
						//too_close = false;
					}

					if (too_close == true && lane == 2 && middleChange == true && lanechanged != true) {
						cout << "Condition suitable for lance change left to 2" << endl;
						lane = 1;
						lanechanged = true;
						//too_close = false;
					}

					if (too_close) {
						ref_vel -= .25;
						cout << "Car speed reducing" << endl;
					}
					else if (ref_vel < 49.5) {
						ref_vel += .25;
						cout << "Car speed increasing" << endl;
					}


 
          // INTERPOLATION
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
          vector<double> ptsx, ptsy;

          // reference x,y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if(prev_size < 2)
          {
              // use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(ref_yaw);
              double prev_car_y = car_y - sin(ref_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          }
          // use the previous path's end point as starting reference
          else
          {
              // Redefine reference state as previous path end point
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Use two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); ++i)
          {
              //shift car reference angle to 0 degrees
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx,ptsy);

          //Define the actual (x,y) points we will use for the planner
          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points, here will always output 50 points
          for (int i = 0; i <= 50-previous_path_x.size() ; ++i)
          {
              double N = (target_dist/(.02*ref_vel/2.24));
              double x_point = x_add_on+(target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back to normal after rotating it earlier
              x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }


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
