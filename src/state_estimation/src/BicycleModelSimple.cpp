#include "state_estimation/BicycleModelSimple.hpp"
#include <iostream>
// Constructor
BicycleModelSimple::BicycleModelSimple(std::vector<double> params) 
    : params_(params), robot_state_(3, 0), com_state_(3,0){
        robot_state_[0] = -0.20;
    }

// Update the state : steering angle in radians
void BicycleModelSimple::update(double velocity, double steering_angle, double dt) {

    double x = robot_state_[0];
    double y = robot_state_[1];
    double theta = robot_state_[2];
    double L = params_[0] + params_[1];

    x += velocity * cos(theta) * dt;
    y += velocity * sin(theta) * dt;
    theta += velocity / L * tan(steering_angle) * dt;
 
    robot_state_[0] = x;
    robot_state_[1] = y;
    robot_state_[2] = theta; 

}

std::vector<double> BicycleModelSimple::get_state(){
    com_state_[0] = robot_state_[0] + params_[0] * cos(robot_state_[2]);
    com_state_[1] = robot_state_[1] + params_[0] * sin(robot_state_[2]);
    com_state_[2] = robot_state_[2];
    return com_state_; 
}


