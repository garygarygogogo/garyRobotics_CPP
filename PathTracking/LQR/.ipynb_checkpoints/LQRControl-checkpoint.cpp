//
// Created by gary on 2023/03/08.
//

#include "LQRControl.h"


LQRControl::LQRControl(int n) : N(n) {}

/**
 * 解代数里卡提方程
 * @param A 状态矩阵A
 * @param B 状态矩阵B
 * @param Q Q为半正定的状态加权矩阵, 通常取为对角阵；Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
 * @param R R为正定的控制加权矩阵，R矩阵元素变大意味着希望控制输入能够尽可能小。
 * @return
 */
Eigen::MatrixXd LQRControl::calRicatti(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    Eigen::MatrixXd Qf= Q;
    Eigen::MatrixXd P = Qf;
    Eigen::MatrixXd P_;
    for(int i=0;i<N;i++){
        P_ = Q+A.transpose()*P*A-A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
        //小于预设精度时返回
        if((P_-P).maxCoeff()<EPS&&(P-P_).maxCoeff()<EPS)break;
        //if((P_-P).cwiseAbs().maxCoeff()<EPS)break;

        P = P_;

    }
    return P_;
}



/**
 * LQR控制器
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
