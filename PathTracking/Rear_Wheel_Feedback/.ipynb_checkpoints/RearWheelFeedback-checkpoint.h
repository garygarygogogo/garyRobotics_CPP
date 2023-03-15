//
// Created by gary on 2023/03/08.
//

#ifndef GARY_CPP_REARWHEELFEEDBACK_H
#define GARY_CPP_REARWHEELFEEDBACK_H
#include <iostream>
#include <vector>
#include<cmath>
#include <algorithm>

#define PI 3.1415926

class RearWheelFeedback {
private:
    double Kpsi,  K2,  L ;// Lyapunov coeff
public:
    RearWheelFeedback(double Kpsi_, double K2_, double L_);
    static double normalizeAngle(double angle);
    double rearWheelFeedbackControl(std::vector<double>robot_state, double e, double k, double ref_psi);

};


#endif // GARY_CPP_REARWHEELFEEDBACK_H
