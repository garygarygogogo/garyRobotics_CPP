//
// Created by gary on 2023/03/08.
//

#ifndef GARY_ROBOT_H
#define GARY_ROBOT_H
#include <Eigen/Dense>
#include <vector>

struct ControlCommand
{
    double steer;
    double acc;
};

class Robot {
private:
    double dT = 0.01;
    double x;
    double y;
    double speed;
    double acc;
    double steer;
    Eigen::Matrix<double, 4,1> xState;
private:
    double Cr = 90000;
    double Cf = 150000;
    double m  = 2500;
    double Lr = 1.5;
    double Lf = 0.8;
    double Iz = 3000;
    void update_kinematic_private(double dT1);
    void update_kinematic();
    void update_dynamics();
    bool usekinematic = false;
    bool simplify = false;
public:
    Robot();
    double getX() const;
    double getY() const;
    double getHeading() const;
    double getReferenceHeading(double k) const;
    double getHeadingRate() const;
    double getSpeed() const;
    double getAcc() const;
    std::vector<Eigen::MatrixXd> getStateSpace(double ref_yaw, double ref_delta) const;
    std::vector<double> getState() const;
    void setAcc(double acc);
    void setSteer(double steer);
    void setL(double lf_, double lr_);
    void setInitPose(double x_, double y_, double heading_);
    void update();
    void setSimDt(double dt);
    void setInitSpeed(double speed_);
    void UseKinematicMode(bool flag);
    void UseSimplify(bool sim);
};


#endif //GARY_ROBOT_H
