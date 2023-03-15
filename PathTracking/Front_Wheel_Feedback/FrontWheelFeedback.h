//
// Created by gary on 2023/03/08.
//

#ifndef GARY_CPP_FRONTWHEELFEEDBACK_H
#define GARY_CPP_FRONTWHEELFEEDBACK_H
#include <iostream>
#include <vector>
#include<cmath>
#include <algorithm>

#define PI 3.1415926

class FrontWheelFeedback {
private:
    double k;
public:
    FrontWheelFeedback(double k);
    double calTargetIndex(std::vector<double> robot_state, std::vector<std::vector<double>> refer_path);
    static double normalizeAngle(double angle);
    std::vector<double> frontWheelFeedbackControl(std::vector<double> robot_state, std::vector<std::vector<double>> refer_path);
    };


#endif// GARY_CPP_FRONTWHEELFEEDBACK_H
