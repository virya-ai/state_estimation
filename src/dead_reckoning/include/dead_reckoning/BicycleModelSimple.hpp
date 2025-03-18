#ifndef BICYCLE_MODEL_HPP
#define BICYCLE_MODEL_HPP

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

class BicycleModelSimple {
public:
    BicycleModelSimple(std::vector<double> params);
    void update(double velocity, double steering_angle, double dt);
    std::vector<double> get_state();


private:

    // Robot State: {x, y. theta}
    std::vector<double> robot_state_;
    
    // Com State: {x, y. theta}
    std::vector<double> com_state_;

    // Params: {lf, lr}
    std::vector<double> params_;
};

#endif // BICYCLE_MODEL_HPP
