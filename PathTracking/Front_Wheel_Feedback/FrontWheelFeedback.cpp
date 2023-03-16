//
// Created by gary on 2023/03/08.
//
#include "FrontWheelFeedback.h"

FrontWheelFeedback::FrontWheelFeedback(double k) {
    k = k;
}

/**
 * search nearest point
 * @param robot_state
 * @param refer_path
 * @return
 */
double FrontWheelFeedback::calTargetIndex(std::vector<double> robot_state, std::vector<std::vector<double>> refer_path) {
    std::vector<double> dists;
    for (auto xy:refer_path) {
        double dist = sqrt(pow(xy[0]-robot_state[0],2)+pow(xy[1]-robot_state[1],2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(),dists.end())-dists.begin(); 
}

/**
 * Normalize to [-PI,PI]
 * @param angle
 * @return
 */
double FrontWheelFeedback::normalizeAngle(double angle) {
    while(angle>PI){
        angle-=2.0*PI;
    }
    while(angle<-PI){
        angle+=2.0*PI;
    }
    return angle;
}


/**
 * Front Wheel Feedback Control
 * @param robot_state including x, y, yaw, v
 * @param refer_path x, y, tangent angle
 * @return control feedback + target point index
 */
std::vector<double> FrontWheelFeedback::frontWheelFeedbackControl(std::vector<double> robot_state, std::vector<std::vector<double>> refer_path) {
    double current_target_index = calTargetIndex(robot_state, refer_path);
    std::vector<double>  current_ref_point;

    // if target point index is greater than that of last index
    if(current_target_index>=refer_path.size()){
        current_target_index = refer_path.size()-1;
        current_ref_point = refer_path[refer_path.size()-1];
    }
    else{
        current_ref_point = refer_path[current_target_index];
    }
    double e_y;
    double psi_t = current_ref_point[2];

    // obtain lateral error e_y
    // https://blog.csdn.net/renyushuai900/article/details/98460758
    if((robot_state[0]-current_ref_point[0])*psi_t-(robot_state[1]-current_ref_point[1])>0){
        e_y = sqrt(pow(current_ref_point[0]-robot_state[0],2)+pow(current_ref_point[1]-robot_state[1],2));
    }
    else{
        e_y = -sqrt(pow(current_ref_point[0]-robot_state[0],2)+pow(current_ref_point[1]-robot_state[1],2));
    }
   
    double psi = robot_state[2];
    double v = robot_state[3];
    double theta_e = psi_t - psi;
    double delta_e = atan2(k*e_y, v);
    double delta = normalizeAngle(delta_e+theta_e);

    return {delta,current_target_index};
}
