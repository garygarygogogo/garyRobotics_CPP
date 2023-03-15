//
// Created by gary on 2023/03/08.
//

#include "MPCControler.h"
#include "mpc_solver/mpc_solver.h"
#include <cmath>
#include "Eigen/LU"
#include "common/log.h"
#include <iostream>
using Matrix = Eigen::MatrixXd;

MPCController::MPCController()
{
    double thetar = 0;
    double vr = 0;
    double deltar = 0;

    wheelbase_ = lf_ + lr_;
    // Matrix init operations.
    matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_(0, 0) = 1.0;
    matrix_ad_(0, 2) = -vr*sin(thetar)*ts_;
    matrix_ad_(1, 1) = 1.0;
    matrix_ad_(1, 2) = vr*cos(thetar)*ts_;
    matrix_ad_(2, 2) = 1.0;
    matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
    matrix_bd_(0, 0) = cos(thetar)*ts_;
    matrix_bd_(1, 0) = sin(thetar)*ts_;
    matrix_bd_(2, 0) = tan(deltar)*ts_/wheelbase_;
    matrix_bd_(2, 1) = vr*ts_/wheelbase_/cos(deltar)/cos(deltar);
    matrix_cd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_state_ = Matrix::Zero(basic_state_size_, 1);
    matrix_r_ = Matrix::Identity(controls_, controls_);
    matrix_q_ = Matrix::Identity(basic_state_size_, basic_state_size_);
    double qarray[] = {5.0, 5.0, 1.0};
    for (int i = 0; i < 3; ++i) {
        matrix_q_(i, i) = qarray[i];
    }
}
void MPCController::ComputeStateErrors(double t, std::vector<double> robot_state, DesireTrajectory* traj)
{
    reference_point = traj->QueryPathPointAtDistance(robot_state[3]*t);
    double xTarget = reference_point.x;
    double yTarget = reference_point.y;
    
    x_error   = robot_state[0] - xTarget;
    y_error   =robot_state[1] - yTarget;
    phi_error = robot_state[2]  - reference_point.phi;
}
void MPCController::UpdateStateAnalyticalMatching()
{
    matrix_state_(0, 0) = x_error;
    matrix_state_(1, 0) = y_error;
    matrix_state_(2, 0) = phi_error;
}
void MPCController::UpdateMatrix(std::vector<double> robot_state) {
    double vr = robot_state[3];
    double deltar = reference_point.Curvature*wheelbase_; // deltar = L/R = L*k
    double thetar = reference_point.phi;
    matrix_ad_(0, 2) = -vr*sin(thetar)*ts_;
    matrix_ad_(1, 2) = vr*cos(thetar)*ts_;
    matrix_bd_(0, 0) = cos(thetar)*ts_;
    matrix_bd_(1, 0) = sin(thetar)*ts_;
    matrix_bd_(2, 0) = tan(deltar)*ts_/wheelbase_;
    matrix_bd_(2, 1) = vr*ts_/wheelbase_/cos(deltar)/cos(deltar);
}
void MPCController::ComputeControlCommand(double t, std::vector<double> robot_state, DesireTrajectory* traj, ControlCommand *cmd)
{
    ComputeStateErrors(t, robot_state, traj);
    UpdateStateAnalyticalMatching();
    UpdateMatrix(robot_state);
    Eigen::MatrixXd control_matrix(controls_, 1);
    control_matrix << 0, 0;

    Eigen::MatrixXd reference_state(basic_state_size_, 1);
    reference_state << 0, 0, 0;

    std::vector<Eigen::MatrixXd> reference(horizon_, reference_state);

    Eigen::MatrixXd lower_bound(controls_, 1);
    lower_bound << -100, -10;

    Eigen::MatrixXd upper_bound(controls_, 1);
    upper_bound << 100, 10;

    std::vector<Eigen::MatrixXd> control(horizon_, control_matrix);

    if (SolveLinearMPC(
            matrix_ad_, matrix_bd_, matrix_cd_, matrix_q_,
            matrix_r_, lower_bound, upper_bound, matrix_state_, reference,
            mpc_eps_, mpc_max_iteration_, &control) != true) {
        AERROR << "MPC solver failed";
    } else {
        //AINFO << "MPC problem solved! ";
    }
    cmd->steer = control[0](1, 0) + reference_point.Curvature*wheelbase_;
    cmd->acc = 0;
}