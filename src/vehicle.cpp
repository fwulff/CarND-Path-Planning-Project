//
// Created by florian on 8/15/17.
//

#include "vehicle.h"

vehicle::vehicle() {}

vehicle::~vehicle() {}

void vehicle::UpdatePosition(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double ref_vel, int car_lane) {
    curr_x = car_x;
    curr_y = car_y;
    curr_s = car_s;
    curr_d = car_d;
    curr_yaw = car_yaw;
    curr_speed = car_speed;
    curr_vel = ref_vel;
    curr_lane = car_lane;
}

void vehicle::UpdateMap(std::vector<double> map_waypoints_x, std::vector<double> map_waypoints_y, std::vector<double> map_waypoints_s){
    maps_x = map_waypoints_x;
    maps_y = map_waypoints_y;
    maps_s = map_waypoints_s;
}

void vehicle::UpdatePath(std::vector<double> previous_path_x, std::vector<double> previous_path_y){
    prev_path_x = previous_path_x;
    prev_path_y = previous_path_y;
    prev_size = prev_path_x.size();
}

void vehicle::Straight(double dist_inc){

    next_x_vals.clear();
    next_y_vals.clear();

    for(int i = 0; i < 50; i++)
    {
        next_x_vals.push_back(curr_x+(dist_inc*i)*cos(helper::deg2rad(curr_yaw)));
        next_y_vals.push_back(curr_y+(dist_inc*i)*sin(helper::deg2rad(curr_yaw)));
    }

}

void vehicle::CreatePath(){

    // Create a list of widely spaced (x,y) points
    vector<double> ptsx;
    vector<double> ptsy;

    //define ref values
    double ref_x = curr_x;
    double ref_y = curr_y;
    double ref_yaw = helper::deg2rad(curr_yaw);

    next_x_vals.clear();
    next_y_vals.clear();

    //check size of prev_points
    if(prev_size < 2){
        //create two initial points tangent to the car
        double prev_curr_x = curr_x - cos(curr_yaw);
        double prev_curr_y = curr_y - sin(curr_yaw);

        //add initial points to pts
        ptsx.push_back(prev_curr_x);
        ptsy.push_back(prev_curr_y);

        //add curr points to pts
        ptsx.push_back(curr_x);
        ptsy.push_back(curr_y);

    }
    else{
        //get previous path end points as initial points
        ref_x = prev_path_x[prev_size-1];
        ref_y = prev_path_y[prev_size-1];

        double ref_x_prev = prev_path_x[prev_size-2];
        double ref_y_prev = prev_path_y[prev_size-2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        //add points to pts
        ptsx.push_back(ref_x_prev);
        ptsy.push_back(ref_y_prev);

        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y);
    }

    //add additional points in 30m steps to the vector
    vector<double> next_wp0 = helper::getXY(curr_s + 30, (2+4*curr_lane), maps_s, maps_x, maps_y);
    vector<double> next_wp1 = helper::getXY(curr_s + 60, (2+4*curr_lane), maps_s, maps_x, maps_y);
    vector<double> next_wp2 = helper::getXY(curr_s + 90, (2+4*curr_lane), maps_s, maps_x, maps_y);

    ptsx.push_back(next_wp0[0]);
    ptsy.push_back(next_wp0[1]);

    ptsx.push_back(next_wp1[0]);
    ptsy.push_back(next_wp1[1]);

    ptsx.push_back(next_wp2[0]);
    ptsy.push_back(next_wp2[1]);

    for(int i = 0; i < ptsx.size(); i++){

        //rotate and translate to cars reference angle
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_y * cos(0 - ref_yaw) + shift_x * sin(0 - ref_yaw));
    }

    //initialise the spline
    tk::spline s;

    //fit spline to points
    s.set_points(ptsx, ptsy);

    next_x_vals.clear();
    next_y_vals.clear();

    //get remaining points from prev_path
    for(int i = 0; i < prev_path_x.size(); i++){
        next_x_vals.push_back(prev_path_x[i]);
        next_y_vals.push_back(prev_path_y[i]);
    }

    //Calculate target distance and point spacing for ref_vel
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x*target_x+target_y*target_y));

    double x_add_on = 0;

    //calculate new points to add to fill up prev_points
    for(int i = 1; i <= 50 - prev_path_x.size(); i++){
        double N = (target_dist / (0.02 * curr_vel / 2.24));
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}
