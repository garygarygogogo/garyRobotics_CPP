//
// Created by Gary on 2023/03/11.
//

#include "PID_controller.h"

PID_controller::PID_controller(double kp, double ki, double kd, double target, double upper, double lower) : kp(kp), ki(ki), kd(kd), target(target), upper(upper), lower(lower) {}

/**
 * set target value
 * @param target
 */
void PID_controller::setTarget(double target){
    this->target = target;
}

/**
 * set pid params
 * @param kp
 * @param ki
 * @param kd
 */
void PID_controller::setK(double kp, double ki, double kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

/**
 * set control bound
 * @param upper
 * @param lower
 */
void PID_controller::setBound(double upper, double lower){
    this->upper = upper;
    this->lower = lower;
}
    
/**
 * calculate output
 * @param state current state
 * @return
 */

double PID_controller::calOutput(double state){
    this->error = this->target - state;
    double cur = this->error*this->kp + this->sum_error*this->ki + (this->error-this->pre_error)*this->kd;
    if(cur<this->lower) cur = this->lower;
    else if(cur>this->upper) cur = this->upper;
    this->pre_error = this->error;
    this->sum_error += this->error;
    return cur;
}

/**
 * set sum_error
 * @param sum_error
 */
void PID_controller::setSumError(double sum_error){
    this->sum_error = sum_error;
}

/**
 * reset
 */
void PID_controller::reset(){
    error = 0.0, pre_error = 0.0, sum_error = 0.0;
}