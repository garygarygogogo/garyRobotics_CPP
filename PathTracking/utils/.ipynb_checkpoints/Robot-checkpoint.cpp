//
// Created by gary on 2023/03/08.
//

#include "Robot.h"
#include <iostream>
Robot::Robot()
{
    double y0 = 0;
    double phi0 = 0;
    xState(0) = y0;
    xState(1) = 0;
    xState(2) = phi0;
    xState(3) = 0;
    speed = 0.01;
    acc = 0;
    steer = 0;
}
double Robot::getX() const {
    return x;
}

double Robot::getY() const {
    return y;
}

double Robot::getHeading() const {
    return xState(2);
}

double Robot::getReferenceHeading(double k) const {
    return atan2((Lf+Lr)*k,1); //Lf+Lr
}
double Robot::getHeadingRate() const {
    return xState(3);
}
double Robot::getSpeed() const {
    return speed;
}
double Robot::getAcc() const {
    return acc;
}

std::vector<double> Robot::getState() const{
    return {x,y,xState(2),speed};
}

std::vector<Eigen::MatrixXd> Robot::getStateSpace(double ref_delta, double ref_yaw) const{
    Eigen::MatrixXd A(3,3),B(3,2);
    A<<1.0,0.0,-speed*dT*sin(ref_yaw),
        0.0,1.0,speed*dT*cos(ref_yaw),
        0.0,0.0,1.0;
    B<<dT*cos(ref_yaw),0,
        dT*sin(ref_yaw),0,
        dT*tan(ref_delta)/(Lr+Lf),speed*dT/((Lf+Lr)*cos(ref_delta)*cos(ref_delta));
    return {A,B};
}

void Robot::setAcc(double acc) {
    Robot::acc = acc;
}
void Robot::setSteer(double steer) {
    Robot::steer = steer;
}
void Robot::setL(double lf_, double lr_) {
    Lf = lf_;
    Lr = lr_;
}
void Robot::setSimDt(double dt)
{
    dT = dt;
}
void Robot::setInitSpeed(double speed_)
{
    speed = speed_;
}
void Robot::setInitPose(double x_, double y_, double heading_)
{
    x = x_;
    y = y_;
    xState(2) = heading_;
}
void Robot::update()
{
    if(usekinematic)
    {
        update_kinematic();
    }else{
        update_dynamics();
    }
}

void Robot::update_kinematic()
{
    double dT1 = dT/10.0;
    for(int i = 0; i < 10; ++i)
    {
       update_kinematic_private(dT1);
    }
}
    
void Robot::update_kinematic_private(double dT1)
{
    double beta=0;
    if(simplify!=true) double beta = atan(Lr*tan(steer)/(Lr+Lf));
    double dphi = speed*cos(beta)/(Lf+Lr)*tan(steer);
    xState(2) = xState(2) + dphi*dT1;
    double dx = speed*cos(xState(2)+beta)*dT1;
    double dy = speed*sin(xState(2)+beta)*dT1;
    x = x + dx;
    y = y + dy;
}

void Robot::update_dynamics() {
    speed += acc*dT;
    if(speed > 30.0)speed = 30.0;
    if(speed < 0.01)speed = 0.01;
    double phi = xState(2);
    double vy  = xState(1);
    x = x + speed*dT*cos(phi) +vy*dT*cos(phi-M_PI/2);
    y = y + speed*dT*sin(phi) +vy*dT*sin(phi-M_PI/2);

    Eigen::Matrix<double, 4,4> A = Eigen::Matrix<double, 4,4>::Zero(4,4);
    A(0,1) = 1.0;
    A(1,1) = -2.0*(Cf+Cr)/(m*speed);
    A(1,3) = -speed - (2.0*Cf*Lf - 2.0*Cr*Lr)/(m*speed);
    A(2,3) = 1.0;
    A(3,1) = -(2*Cf*Lf - 2*Cr*Lr)/(Iz*speed);
    A(3,3) = -(2*Cf*Lf*Lf + 2*Cr*Lr*Lr)/(Iz*speed);

    Eigen::Matrix<double, 4,1> B = Eigen::Matrix<double, 4,1>::Zero(4,1);
    B(1) = 2*Cf/m;
    B(3) = 2*Lf*Cf/Iz;
    //Eigen::Matrix<double, 4,1> dx = A*xState + B*steer;
    //xState = xState + dx*dT;
    Eigen::Matrix<double, 4,4> I = Eigen::Matrix<double, 4,4>::Identity(4,4);
    Eigen::Matrix<double, 4,4> Ad = A*dT + I;
    Eigen::Matrix<double, 4,1> Bd = B*dT;

    xState = Ad * xState + Bd *steer;
}
void Robot::UseKinematicMode(bool flag)
{
    usekinematic = flag;
}
void Robot::UseSimplify(bool sim)
{
    simplify = sim;
}