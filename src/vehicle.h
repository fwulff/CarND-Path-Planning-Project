//
// Created by florian on 8/15/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

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

class vehicle {

public:
    /*
     * vehicle parameter
     */
    double curr_x; //cartesian coordinates
    double curr_y; //cartesian coordinates
    double curr_s; //frenet coordinates
    double curr_d; //frenet coordinates
    double curr_yaw; //yaw acc
    double curr_speed; //speed

    /*
    * vehicle path
    */
    int prev_size;
    std::vector<double> prev_path_x;
    std::vector<double> prev_path_y;
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    int curr_lane  = 1;
    double curr_vel = 0; //mph

    /*
     * map informations
     */
    std::vector<double> maps_x;
    std::vector<double> maps_y;
    std::vector<double> maps_s;

    /*
    * Constructor
    */
    vehicle();

    /*
    * Destructor.
    */
    virtual ~vehicle();

    /*
    * Update vehicle informations
    */
    void UpdatePosition(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double ref_vel, int car_lane);

    void UpdateMap(std::vector<double> map_waypoints_x, std::vector<double> map_waypoints_y, std::vector<double> map_waypoints_s);

    void UpdatePath(std::vector<double> previous_path_x, std::vector<double> previous_path_y);

    /*
     * Create a straight path
     */
    void Straight(double dist_inc);

    /*
    * Follow the lane or change lanes path
    */
    void CreatePath();

protected:

private:




};


#endif //PATH_PLANNING_VEHICLE_H
