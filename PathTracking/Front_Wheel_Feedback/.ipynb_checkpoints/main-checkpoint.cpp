//
// Created by gary on 2023/03/08.
//
#include "FrontWheelFeedback.h"
#include "../../matplotlibcpp.h"
#include "../utils/Robot.h"
namespace plt = matplotlibcpp;

#define PI 3.1415926

int main(){
    double dt=0.1; // time step，unit：s
    double L=2; // wheel base, unit：m
    double v = 2; // initial speed
    double x_0=0; // inital xpos
    double y_0=-1.0; // inital ypos
    double psi_0=0.5; // inital yaw
    
    double k = 3.; // gain coeff
    
    std::vector<std::vector<double>> refer_path(1000, std::vector<double>(3));
    std::vector<double> refer_x,refer_y; // save for plot
    // obtain reference trajectory
    for(int i=0; i<1000; ++i){
        refer_path[i][0] = 0.1*i;
        refer_path[i][1] = 2 * sin(refer_path[i][0] / 3.0) + 2.5*cos(refer_path[i][0] / 2.0);
        for(int i=0; i<999; ++i){
            refer_path[i][2]= atan2((refer_path[i+1][1] - refer_path[i][1]),(refer_path[i+1][0]-refer_path[i][0]));
        }
        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
//        cout<<refer_path[i][0]<<" ,"<<refer_path[i][1]<<endl;
    }

    // vector to save actual trajectory
    std::vector<double> x_, y_;
    
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
    
    FrontWheelFeedback fwf(k);

    for(int i=0; i<600; ++i){
        plt::clf();
        robot_state = ugv.getState();
  
        std::vector<double> delta_index = fwf.frontWheelFeedbackControl(robot_state,refer_path);

        // update robot state
        ugv.setAcc(0);
        ugv.setSteer(delta_index[0]);
        ugv.update();
        
        // save actural trajectory
        x_.push_back(ugv.getX());
        y_.push_back(ugv.getY());
     
        // plot reference trajectory
        plt::plot(refer_x,refer_y, "b--");
        plt::grid(true);
        plt::ylim(-5,5);
        // plot actual trajectory
        plt::plot(x_, y_, "r");
        plt::pause(0.01);
        
        // determine if end of trajectory
        if(delta_index[1] >= refer_path.size() - 1) break;
    }
    // save figure
    const char* filename = "./front_demo.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}