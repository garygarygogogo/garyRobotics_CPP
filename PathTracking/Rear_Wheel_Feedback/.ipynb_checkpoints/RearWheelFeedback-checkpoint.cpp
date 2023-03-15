//
// Created by gary on 2023/03/08.
//

#include "RearWheelFeedback.h"

/**
 * Normalize to [-PI,PI]
 * @param angle
 * @return
 */
double RearWheelFeedback::normalizeAngle(double angle) {
    while(angle>PI){
        angle-=2.0*PI;
    }
    while(angle<-PI){
        angle+=2.0*PI;
    }
    return angle;
}

/**
 * Rear Wheel Feedback Control
 * @param robot_state x, y, yaw, v
 * @param e
 * @param k
 * @param ref_psi tangent angle of reference point
 * @return
 */
double RearWheelFeedback::rearWheelFeedbackControl(std::vector<double> robot_state, double e, double k, double ref_psi) {
    double psi = robot_state[2], v = robot_state[3];
    double psi_e = normalizeAngle(psi-ref_psi);//psi_e=yaw-ref_yaw
    // formular17
    double psi_dot = v * k * cos(psi_e) / (1.0 - k * e)  - K2 * v * sin(psi_e) * e / psi_e- Kpsi * abs(v) * psi_e;

    if(psi_e == 0.0 || psi_dot == 0.0) return 0.0;

    // formular21
    double delta = atan2(L * psi_dot, v);

    return delta;

}

/**
 * Constructor
 * @param Kpsi
 * @param K2
 * @param L
 */
RearWheelFeedback::RearWheelFeedback(double Kpsi_, double K2_, double L_) : Kpsi(Kpsi_), K2(K2_), L(L_) {}


