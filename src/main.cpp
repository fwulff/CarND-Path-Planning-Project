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
#include "helper.h"
#include "vehicle.h"

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

    /*
     * states:
     * 0 - drive straight
     * 1 - prepare lane shift
     * 2 - do lane shift left
     * 3 - do lane shift right
    */

    int curr_state = 0; //initialise state machine in state straight
    int curr_lane = 1; //initialise lane
    double ref_vel = 0; //initialise standing

    //initialise the car
    vehicle car;
    printf("New car created\n");

    //initialise map for car
    car.UpdateMap(map_waypoints_x, map_waypoints_y, map_waypoints_s);
    printf("Car Map update created\n");

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &curr_state, &curr_lane, &ref_vel, &car](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = helper::hasData(data);

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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	int prev_size = previous_path_x.size();

            if(prev_size > 0){
                car_s = end_path_s;
            }

            //State Drive straight until approaching a vehicle
            if (curr_state == 0){

                //parameters for vehicles in front
                double front_car_distance = 1000;
                double front_car_s = 55.;
                double front_car_v = 55.;

                //check all cars in sensor fusion
                for(int i = 0; i < sensor_fusion.size(); i++){

                    //check lane of current other car (d value)
                    float check_d = sensor_fusion[i][6];

                    //check for cars in ego lane
                    if((check_d < (2+4*curr_lane+2)) && (check_d > (2+4*curr_lane-2))){

                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_v = sqrt(vx*vx+vy*vy);
                        double check_s = sensor_fusion[i][5];

                        check_s += ((double)prev_size * 0.02 * check_v);

                        if((check_s > car_s) && ((check_s - car_s) < 30.)){

                            if(front_car_v > (check_v)){
                                front_car_v = check_v * 2.237;
                                printf("new speed of vehicle front is %f\n", front_car_v);
                            }
                        }
                    }
                }

                //braking
                if(ref_vel > (front_car_v - 5.)){
                    ref_vel -= 1.0;
                    printf("Vehicle in front, braking to %f to reach car %f\n", ref_vel, front_car_v - 5.);
                    if (ref_vel < (front_car_v - 3.)){
                        printf("Equal speed to vehicle in front, prepare lane shift %f\n", ref_vel);
                        //Go to prepare lane shift state
                        curr_state = 1;
                    }
                }

                //accelerate
                else if(ref_vel < (front_car_v - 5.) && (ref_vel < 49.0)){
                    ref_vel += 0.5;
                    printf("Accelerationg to %f\n", ref_vel);
                }

                //stay at max ref_vel
                else{
                    printf("Driving at max. speed %f\n", ref_vel);
                }
            }

            //prepare lane change
            else if (curr_state == 1) {
                printf("State 1, prepare lane change with %f\n", ref_vel);

                bool left_lane_safe = true;
                bool right_lane_safe = true;
                bool changed_lanes = false;

                for (int i = 0; i < sensor_fusion.size(); i++) {

                    //check lane of current other car (d value)
                    float check_d = sensor_fusion[i][6];

                    //check for cars in left lane (if existing)
                    if (curr_lane >= 1) {
                        if (check_d < (2 + 4 * (curr_lane - 1) + 2) && check_d > (2 + 4 * (curr_lane - 1) - 2)) {

                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_v = sqrt(vx * vx + vy * vy);
                            double check_s = sensor_fusion[i][5];

                            check_s += ((double) prev_size * .02 * check_v);
                            double dist_s = check_s - car_s;

                            if (dist_s < 31 && dist_s > -20) {
                                //vehicle found nearby on left lane
                                left_lane_safe = false;
                            }
                        }
                    }

                    //check for cars in right lane
                    if (curr_lane <= 1) {
                        if (check_d < (2 + 4 * (curr_lane + 1) + 2) && check_d > (2 + 4 * (curr_lane + 1) - 2)) {

                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_v = sqrt(vx * vx + vy * vy);
                            double check_s = sensor_fusion[i][5];

                            check_s += ((double) prev_size * .02 * check_v);
                            double dist_s = check_s - car_s;
                            if (dist_s < 31 && dist_s > -20) {
                                right_lane_safe = false;
                            }
                        }
                    }


                    //check if surrounding vehicles do exist otherwise do lane change
                    if (curr_lane == 0) {
                        if (right_lane_safe) {
                            printf("right lane safe %i\n", right_lane_safe);;
                            curr_state = 3;
                        } else {
                            curr_state = 0;
                        }
                    } else if (curr_lane == 2) {
                        if (left_lane_safe) {
                            printf("left lane safe %i\n", left_lane_safe);
                            curr_state = 2;
                        } else {
                            curr_state = 0;
                        }
                    } else if (curr_lane == 1) {
                        if (left_lane_safe) {
                            printf("left lane safe %i\n", left_lane_safe);
                            curr_state = 2;
                        } else if (right_lane_safe) {
                            printf("right lane safe %i\n", right_lane_safe);
                            curr_state = 3;
                        } else {
                            curr_state = 0;
                        }

                    } else {
                        printf("Wait for lane change, vehicles present, %f\n", ref_vel);
                        //go back to straight (adjust speed eventually)
                        curr_state = 0;
                    }
                }
            }

            //Execute the lane shift left
            else if (curr_state == 2){
                printf("State 2, execute left lane change with %f\n", ref_vel);
                //check for vehicles in front
                bool too_close = false;

                if(curr_lane >= 1){
                    curr_lane -= 1;
                }

                curr_state = 0;
            }

                //Execute the lane shift
            else if (curr_state == 3){
                printf("State 3, execute right lane change with %f\n", ref_vel);
                //check for vehicles in front
                bool too_close = false;

                if (curr_lane <= 1){
                    curr_lane += 1;
                }
                curr_state = 0;
            }

            car.UpdatePosition(car_x, car_y, car_s, car_d, car_yaw, car_speed, ref_vel, curr_lane);

            car.UpdatePath(previous_path_x, previous_path_y);

            car.CreatePath();

            json msgJson;

          	// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = car.next_x_vals;
          	msgJson["next_y"] = car.next_y_vals;

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
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
















































































