//
// Created by gary on 2023/03/08.
//

#ifndef MPC_CONTROL_MPCCONTROLER_H
#define MPC_CONTROL_MPCCONTROLER_H
#include "planning/DesireTrajectory.h"
#include "Eigen/Core"
#include "../utils/Robot.h"

class MPCController {
private:
    double x_error = 0.0;
    double y_error = 0.0;
    double phi_error = 0.0;
    // control time interval
    double ts_ = 0.01;
    // corner stiffness; front
    double cf_ = 150000;
    // corner stiffness; rear
    double cr_ = 90000;
    // distance between front and rear wheel center
    double wheelbase_ = 2.3;
    // mass of the vehicle
    double mass_ = 2500;
    // distance from front wheel center to COM
    double lf_ = 0.8;
    // distance from rear wheel center to COM
    double lr_ = 1.5;
    // rotational inertia
    double iz_ = 3000;
    // number of states, includes
    // lateral error, lateral error rate, heading error, heading error rate,
    // station error, velocity error,
    const int basic_state_size_ = 3;
    const int controls_ = 2;
    const int horizon_ = 5;

    Eigen::MatrixXd matrix_ad_;
    Eigen::MatrixXd matrix_bd_;
    Eigen::MatrixXd matrix_cd_;
    Eigen::MatrixXd matrix_r_;
    // state weighting matrix
    Eigen::MatrixXd matrix_q_;
    // 4 by 1 matrix; state matrix
    Eigen::MatrixXd matrix_state_;
    // parameters for mpc solver; number of iterations
    int mpc_max_iteration_ = 150;
    // parameters for mpc solver; threshold for computation
    double mpc_eps_ = 0.01;
    TrajectoryPoint reference_point;
public:
    MPCController();
    void ComputeStateErrors(double t, std::vector<double> robot_state, DesireTrajectory* traj);
    void UpdateStateAnalyticalMatching();
    void UpdateMatrix(std::vector<double> robot_state);
    void ComputeControlCommand(double t, std::vector<double> robot_state, DesireTrajectory* traj, ControlCommand *cmd);
};

#endif //MPC_CONTROL_MPCCONTROLER_H
