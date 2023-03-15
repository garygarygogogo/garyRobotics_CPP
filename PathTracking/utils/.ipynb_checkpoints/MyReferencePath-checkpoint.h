//
// Created by gary on 2023/03/08.
//

#ifndef GARY_CPP_MYREFERENCEPATH_H
#define GARY_CPP_MYREFERENCEPATH_H
#include <iostream>
#include <vector>
#include<cmath>
#include <algorithm>
#define PI 3.1415926

class MyReferencePath {
public:
    // 4 dimensional refer_path：x, y, tangent, k
    std::vector<std::vector<double>>refer_path;
    std::vector<double>refer_x,refer_y;
public:
    MyReferencePath();
    std::vector<double> calTrackError(std::vector<double>robot_state);
    double normalizeAngle(double angle);
};


#endif //GARY_CPP_MYREFERENCEPATH_H
