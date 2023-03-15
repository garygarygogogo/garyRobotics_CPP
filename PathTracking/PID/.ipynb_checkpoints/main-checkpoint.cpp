//
// Created by Gary on 2023/03/11.
//

#include "../utils/Robot.h"
#include "PID_controller.h"
#include <algorithm>
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

/**
 * PID controller for trajectory tracking
 */


#define PI 3.1415926

/**
 * Obtain index of the nearest point on the reference trajectory
 * @param robot_state (x,y)
 * @param refer_path
 * @return the index of the nearest point on the refer_path
 */

double calTargetIndex(std::vector<double> robot_state, std::vector<std::vector<double>> refer_path){
    std::vector<double> dists;
    for(auto xy:refer_path){
        double dist = sqrt(pow(xy[0]-robot_state[0],2)+pow(xy[1]-robot_state[1],2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(),dists.end())-dists.begin(); // return index
}

int main(){
    std::vector<std::vector<double>> refer_path(1000, std::vector<double>(2));
    std::vector<double> refer_x, refer_y; // for plot
    // generate trajectory
    for(int i=0; i<1000; ++i){
        refer_path[i][0] = 0.1*i;
        refer_path[i][1] = 2*sin(refer_path[i][0]/3.0);
        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
    }
    
    bool useKinematic = true;
    bool noBeta = true;
    double dT = 0.1;
    // kinematic model
    Robot ugv;
    // set time step,initial state
    ugv.setSimDt(dT);
    ugv.setInitPose(0.0, -1.0, 0.5);
    ugv.setInitSpeed(2.0);
    ugv.setL(1.0,1.0);
    ugv.UseKinematicMode(useKinematic);
    ugv.UseSimplify(noBeta);
    
    // PID controller
    PID_controller PID(2,0.01,30,0.,PI/6,-PI/6);
    
    // save trajectory
    std::vector<double> x_, y_;
    // save robot state
    std::vector<double> robot_state;
    // run 500 epochs
    for(int i=0; i<500; ++i){
        plt::clf();
        robot_state = ugv.getState();
        // calculate pid output
        double min_ind = calTargetIndex(robot_state,refer_path);
        double alpha = atan2(refer_path[min_ind][1]- robot_state[1],refer_path[min_ind][0]- robot_state[0]);
        double ld = sqrt(pow(refer_path[min_ind][0]- robot_state[0],2)+pow(refer_path[min_ind][1]- robot_state[1],2));
        double theta_e = alpha -  robot_state[2];
        double e_y = -ld * sin(theta_e);
        double delta_f = PID.calOutput(e_y);
        // update robot state
        ugv.setAcc(0);
        ugv.setSteer(delta_f);
        ugv.update();
        x_.push_back(ugv.getX());
        y_.push_back(ugv.getY());
        // plot trajectory
        plt::plot(refer_x,refer_y,"b--");
        plt::plot(x_,y_,"r");
        plt::grid(true);
        plt::ylim(-2.5,2.5);
        plt::pause(0.01);
    }
    
    // save figure
    const char* figname = "./pid_demo.png";
    std::cout << "Saving result to: " << figname << std::endl;
    plt::save(figname);
    plt::show();
    return 0;
}