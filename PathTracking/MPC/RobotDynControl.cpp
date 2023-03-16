//
// Created by gary on 2023/3/8.
//

#include "mpc_solver/mpc_solver.h"
#include <iostream>
#include <algorithm>
#include "planning/DesireTrajectory.h"
#include "MPCControler.h"
#include "../utils/Robot.h"
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main() {
    DesireTrajectory traj;
    Robot ugv;
    std::vector<double> refer_x, refer_y; // for plot

    MPCController mpc;
    ControlCommand cmd;
    std::vector<double> x_;
    std::vector<double> y_;
    // polynomial trajectory
    traj.SetDemoTrajData();
    // save robot state
    std::vector<double> robot_state;
    
    for(int i = 0; i < traj.path.size(); ++i)
    {
        refer_x.push_back(traj.path[i].x);
        refer_y.push_back(traj.path[i].y);
    }
    
    double dT = 0.01;
    bool useKinematic = false;
    bool noBeta = false;
    // set time step,initial state
    ugv.setSimDt(dT);
    ugv.setInitPose(0, 0, 0);
    ugv.setInitSpeed(10.0);
    ugv.UseKinematicMode(useKinematic);
    ugv.UseSimplify(noBeta);
    double t = 0.0;
    for(int i = 0; i < 1150; ++i)
    {
        plt::clf();
        // Get Vehicle State
        robot_state = ugv.getState();
        // Compute control outputs
        mpc.ComputeControlCommand(t, robot_state, &traj, &cmd);
        // Save data
        x_.push_back(robot_state[0]);
        y_.push_back(robot_state[1]);
        // Simulation
        ugv.setAcc(cmd.acc);
        ugv.setSteer(cmd.steer);
        ugv.update();
        // update time
        t += dT;
    }
    // plot trajectory
    plt::plot(refer_x,refer_y,"b--");
    plt::plot(x_,y_,"r");
    plt::grid(true);
    plt::ylim(-2.5,5.0);
    plt::pause(0.01);
    // save figure
    const char* figname = "./dyn_mpc_demo.png";
    std::cout << "Saving result to: " << figname << std::endl;
    plt::save(figname);
    plt::show();
    return 0;
}

