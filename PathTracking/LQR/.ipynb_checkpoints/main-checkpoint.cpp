//
// Created by gary on 2023/03/08.
//

#include "LQRControl.h"
#include "../utils/MyReferencePath.h"
#include "../utils/NormalizeAngle.hpp"
#include "../../matplotlibcpp.h"
#include "../utils/Robot.h"
namespace plt = matplotlibcpp;

int main(){
    double dt=0.1; // time step，unit：s
    double L=2; // wheel base, unit：m
    double v = 2; // initial speed
    double x_0=0; // inital xpos
    double y_0=-3; // inital ypos
    double psi_0=0; // inital yaw
    int N = 100;// iterations

    Eigen::MatrixXd Q(3,3);
    Q<<3,0,0,
            0,3,0,
            0,0,3;
    Eigen::MatrixXd R(2,2);
    R<<2.0,0.0,
            0.0,2;

    // save actual trajectory
    std::vector<double> x_,y_;
    MyReferencePath referencePath;
    bool useKinematic = true;
    bool noBeta = true;

    // kinematic model
    Robot ugv;
    // set time step,initial state
    ugv.setSimDt(dt);
    ugv.setInitPose(x_0,y_0,psi_0);
    ugv.setInitSpeed(v);
    ugv.setL(L/2,L/2);
    ugv.UseKinematicMode(useKinematic);
    ugv.UseSimplify(noBeta);
    LQRControl lqr(N);
    // save robot state
    std::vector<double> robot_state;

    for(int i=0; i<500; ++i){
        plt::clf();
        // obtain current state
        robot_state = ugv.getState();
        std::vector<double> one_trial = referencePath.calTrackError(robot_state);
        double k = one_trial[1], ref_yaw = one_trial[2], s0=one_trial[3];

        double ref_delta = ugv.getReferenceHeading(k);
        std::vector<Eigen::MatrixXd> state_space = ugv.getStateSpace(ref_delta,ref_yaw);
        
        // control value
        double delta = lqr.lqrControl(robot_state, referencePath.refer_path, s0, state_space[0], state_space[1], Q, R);
        delta = delta+ref_delta;

        // update robot state
        ugv.setAcc(0);
        ugv.setSteer(delta);
        ugv.update();
        
        // save trajectory
        x_.push_back(ugv.getX());
        y_.push_back(ugv.getY());
        
        // plot reference trajectory
        plt::plot(referencePath.refer_x,referencePath.refer_y,"b--");
        plt::grid(true);
        plt::ylim(-5,5);
        // plot actual trajectory
        plt::plot(x_, y_,"r");
        plt::pause(0.01);
    }
    // save figure
    const char* filename = "./lqr_demo.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}