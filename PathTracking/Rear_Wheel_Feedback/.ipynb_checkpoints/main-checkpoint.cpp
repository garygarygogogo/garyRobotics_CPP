//
// Created by gary on 2023/03/08.
//
#include "RearWheelFeedback.h"
#include "../utils/MyReferencePath.h"
#include "../../matplotlibcpp.h"
#include "../utils/Robot.h"
namespace plt = matplotlibcpp;

#define PI 3.1415926

int main(){
    double dt=0.1; // time step，unit：s
    double L=2; // wheel base, unit：m
    double v = 2; // initial speed
    double x_0=0; // inital xpos
    double y_0=-3; // inital ypos
    double psi_0=0; // inital yaw

    double Kpsi=3;
    double K2=1.5;// Lyapunov coeff

    // define controller
    RearWheelFeedback rwf(Kpsi,K2,L);
    
    // vector to save actual trajectory
    std::vector<double> x_,y_;
    MyReferencePath referencePath;
    
    // define robot model
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
    // save robot state
    std::vector<double> robot_state;

    for(int i=0; i<500; ++i){
        plt::clf();
        // obtain current state
        robot_state = ugv.getState();
        std::vector<double> one_trial = referencePath.calTrackError(robot_state);
        double e = one_trial[0], k = one_trial[1], ref_psi = one_trial[2];
        double delta = rwf.rearWheelFeedbackControl(robot_state, e, k, ref_psi);

        // update robot state
        ugv.setAcc(0);
        ugv.setSteer(delta);
        ugv.update();
        
        // save actural trajectory
        x_.push_back(ugv.getX());
        y_.push_back(ugv.getY());
     
        // plot reference trajectory
        plt::plot(referencePath.refer_x, referencePath.refer_y, "b--");
        plt::grid(true);
        plt::ylim(-5,5);
        // plot actual trajectory
        plt::plot(x_, y_, "r");
        plt::pause(0.01);
    }
    // save figure
    const char* filename = "./rear_demo.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}
