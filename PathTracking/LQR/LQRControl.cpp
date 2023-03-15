//
// Created by gary on 2023/03/08.
//

#include "LQRControl.h"


LQRControl::LQRControl(int n) : N(n) {}

/**
 * solve Riccati Equation
 * @param A 
 * @param B
 * @param Q the bigger Q, the faster error approach 0；
 * @param R the bigger R, the smaller control response;
 * @return
 */
Eigen::MatrixXd LQRControl::calRicatti(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    Eigen::MatrixXd Qf= Q;
    Eigen::MatrixXd P = Qf;
    Eigen::MatrixXd P_;
    for(int i=0;i<N;i++){
        P_ = Q+A.transpose()*P*A-A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
        // return if marigin smaller than threshold
        if((P_-P).maxCoeff()<EPS&&(P-P_).maxCoeff()<EPS)break;
        //if((P_-P).cwiseAbs().maxCoeff()<EPS)break;

        P = P_;

    }
    return P_;
}



/**
 * LQR
 * @param robot_state
 * @param refer_path
 * @param s0
 * @param A
 * @param B
 * @param Q
 * @param R
 * @return
 */
double LQRControl::lqrControl(std::vector<double> robot_state, std::vector<std::vector<double>> refer_path, double s0, Eigen::MatrixXd A, Eigen::MatrixXd B,
                       Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    Eigen::MatrixXd X(3,1);
    X<<robot_state[0]-refer_path[s0][0],robot_state[1]-refer_path[s0][1],robot_state[2]-refer_path[s0][2];
    Eigen::MatrixXd P = calRicatti(A,B,Q,R);
    Eigen::MatrixXd K = -(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
    Eigen::MatrixXd u = K*X; //[v-ref_v,delta-ref_delta]

    return u(1,0);

}
