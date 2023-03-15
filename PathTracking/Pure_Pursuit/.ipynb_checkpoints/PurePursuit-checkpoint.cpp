//
// Created by gary on 2023/03/08.
//

#include "PurePursuit.h"

/**
 * obtain nearest point
 * @param robot_state
 * @param refer_path
 * @param l_d forward distance
 * @return
 */
double PurePursuit::calTargetIndex(std::vector<double> robot_state, std::vector<std::vector<double>> refer_path, double l_d) {
    std::vector<double> dists;
    for (auto xy : refer_path) {
        double dist = sqrt(pow(xy[0]-robot_state[0],2)+pow(xy[1]-robot_state[1],2));
        dists.push_back(dist);
    }
    double min_ind = min_element(dists.begin(),dists.end())-dists.begin(); //return index

    double delta_l = sqrt(pow(refer_path[min_ind][0]-robot_state[0],2)+pow(refer_path[min_ind][1]-robot_state[1],2));

    while (l_d>delta_l && min_ind<refer_path.size()-1){
        delta_l = sqrt(pow(refer_path[min_ind+1][0]-robot_state[0],2)+pow(refer_path[min_ind+1][1]-robot_state[1],2));
        min_ind+=1;
    }
    return min_ind;
}

/**
 * purePursuitControl
 * @param robot_state current position
 * @param current_ref_point
 * @param l_d
 * @param psi
 * @param L
 * @return steer angle
 */
double PurePursuit::purePursuitControl(std::vector<double> robot_state, std::vector<double> current_ref_point, double l_d, double L) {
    double alpha = atan2(current_ref_point[1]-robot_state[1],current_ref_point[0]-robot_state[0])-robot_state[2];

    double delta = atan2(2*L*sin(alpha),l_d);

    return delta;

}


