//
// Created by gary on 2023/03/08.
//

#ifndef CHHROBOTICS_CPP_LQRCONTROL_H
#define CHHROBOTICS_CPP_LQRCONTROL_H

#define EPS 1.0e-4
#include <Eigen/Dense>
#include <vector>
#include <iostream>


class LQRControl {
private:
    int N;

public:
    LQRControl(int n);

    Eigen::MatrixXd calRicatti(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R);
    double lqrControl(std::vector<double>robot_state,std::vector<std::vector<double>>refer_path, double s0, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R);
};


#endif //CHHROBOTICS_CPP_LQRCONTROL_H
