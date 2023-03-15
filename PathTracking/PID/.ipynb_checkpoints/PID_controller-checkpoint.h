//
// Created by Gary on 2023/03/11.
//

#ifndef __GARY_CPP_PID_CONTROLLER_H__
#define __GARY_CPP_PID_CONTROLLER_H__

#include<iostream>
/**
 * Implement of pid
 */
class PID_controller{
private:
    double kp, ki, kd, target, upper, lower;
    double error=0.0, pre_error=0.0, sum_error=0.0;
public:
    PID_controller(double kp, double ki, double kd, double target, double upper, double lower);
    void setTarget(double target);
    void setK(double kp, double ki, double kd);
    void setBound(double upper, double lower);
    double calOutput(double state);
    void setSumError(double sum_error);
    void reset();
};


#endif //__GARY_CPP_PID_CONTROLLER_H__
