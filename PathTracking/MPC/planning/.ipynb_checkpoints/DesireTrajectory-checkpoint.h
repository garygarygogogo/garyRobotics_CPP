//
// Created by yuanyancheng on 2020/11/11.
//

#ifndef MPC_CONTROL_DESIRETRAJECTORY_H
#define MPC_CONTROL_DESIRETRAJECTORY_H
#include <vector>
struct TrajectoryPoint
{
    double x;
    double y;
    double phi;
    double Curvature;
    double v;
    double t;
};

class DesireTrajectory {
public:
    std::vector<TrajectoryPoint> path;
    std::vector<double> diss;
public:
    DesireTrajectory(){};
    void SetDemoTrajData();
    TrajectoryPoint QueryPathPointAtTime(double time);
    TrajectoryPoint QueryPathPointAtDistance(double dis);
};


#endif //MPC_CONTROL_DESIRETRAJECTORY_H
