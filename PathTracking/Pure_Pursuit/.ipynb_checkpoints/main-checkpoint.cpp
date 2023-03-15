//
// Created by gary on 2023/03/08.
//

#include "PurePursuit.h"
#include "../../matplotlibcpp.h"
#include "../utils/Robot.h"
namespace plt = matplotlibcpp;

#define PI 3.1415926

int main(){

    double dt=0.1; // time step，unit：s
    double L=2; // wheel base, unit：m
    double v = 2; // initial speed
    double x_0=0.0; // inital xpos
    double y_0=-1.0; // inital ypos
    double psi_0=0.5; // inital yaw
    double lam = 0.1; // coeff of forward dist
    double c=2; // forward dist
    
    std::vector<std::vector<double>> refer_path(1000, std::vector<double>(2));
    std::vector<double>refer_x,refer_y; // save for plot
    // reference trajectory
    for(int i=0;i<1000;i++){
        refer_path[i][0]=0.1*i;
        refer_path[i][1]=2*sin(refer_path[i][0]/3.0)+2.5*cos(refer_path[i][0]/2.0);
        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
//        cout<<refer_path[i][0]<<" ,"<<refer_path[i][1]<<endl;
    }

    // save for plot
    std::vector<double> x_,y_;
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
    
    //robot state
    std::vector<double> robot_state;
    // purepursuit controller
    PurePursuit pp;
    
    for(int i=0; i<600; ++i){
        plt::clf();
        // obtain current state
        robot_state = ugv.getState();

        double l_d = lam*robot_state[3]+c;
        double min_ind = pp.calTargetIndex(robot_state, refer_path, l_d);
        double delta = pp.purePursuitControl(robot_state, refer_path[min_ind], l_d, L);
        
        // update robot state
        ugv.setAcc(0);
        ugv.setSteer(delta);
        ugv.update();
        
        // save trajectory
        x_.push_back(ugv.getX());
        y_.push_back(ugv.getY());

        // plot
        plt::plot(refer_x,refer_y,"b--");
        plt::plot(x_, y_,"r");
        plt::grid(true);
        plt::ylim(-5,5);
        plt::pause(0.01);
    }
    // save figure
    const char* filename = "./pp_demo.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}