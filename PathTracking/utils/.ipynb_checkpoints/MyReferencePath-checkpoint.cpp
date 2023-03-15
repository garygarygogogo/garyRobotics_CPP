//
// Created by gary on 2023/03/08.
//

#include "MyReferencePath.h"

/**
 * obtian k
 */
MyReferencePath::MyReferencePath() {
    refer_path = std::vector<std::vector<double>>(1000, std::vector<double>(4));
    // obtain reference trajectory
    for (int i = 0; i < 1000; i++) {
        refer_path[i][0] = 0.1 * i;
        refer_path[i][1] = 2 * sin(refer_path[i][0] / 3.0) + 2.5 * cos(refer_path[i][0] / 2.0);

        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
    }
    double dx,dy,ddx,ddy;
    for(int i=0;i<refer_path.size();i++){
        if (i==0){
             dx = refer_path[i+1][0] - refer_path[i][0];
             dy = refer_path[i+1][1] - refer_path[i][1];
             ddx = refer_path[2][0] + refer_path[0][0] - 2*refer_path[1][0];
             ddy = refer_path[2][1] + refer_path[0][1] - 2*refer_path[1][1];
        }else if(i==refer_path.size()-1){
             dx = refer_path[i][0] - refer_path[i-1][0];
             dy = refer_path[i][1] - refer_path[i-1][1];
             ddx = refer_path[i][0] + refer_path[i-2][0] - 2*refer_path[i-1][0];
             ddy = refer_path[i][1] + refer_path[i-2][1] - 2*refer_path[i-1][1];
        }else{
             dx = refer_path[i+1][0] - refer_path[i][0];
             dy = refer_path[i+1][1] - refer_path[i][1];
             ddx = refer_path[i+1][0] + refer_path[i-1][0] - 2*refer_path[i][0];
             ddy = refer_path[i+1][1] + refer_path[i-1][1] - 2*refer_path[i][1];
        }
        refer_path[i][2] = atan2(dy,dx);//yaw
        // if r(t) =(x(t),y(t)),then k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
        refer_path[i][3]= (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3 / 2) ;
    }
}

/**
 * obtain tracking error
 * @param robot_state 
 * @return
 */
std::vector<double> MyReferencePath::calTrackError(std::vector<double> robot_state) {
    double x = robot_state[0],y=robot_state[1];
    std::vector<double> d_x(refer_path.size()),d_y(refer_path.size()),d(refer_path.size());
    for(int i=0;i<refer_path.size();i++) {
        d_x[i]=refer_path[i][0]-x;
        d_y[i]=refer_path[i][1]-y;
        d[i] = sqrt(d_x[i]*d_x[i]+d_y[i]*d_y[i]);

    }
    double min_index = min_element(d.begin(),d.end())-d.begin();
    //cout<<min_index<<endl;
    double yaw = refer_path[min_index][2];
    double k = refer_path[min_index][3];
    double angle = normalizeAngle(yaw- atan2(d_y[min_index],d_x[min_index]));
    double error = d[min_index];// tracking error
    if(angle<0)error*=-1;
    return {error,k,yaw,min_index};
}

/**
 * angle normalization
 * @param angle
 * @return
 */
double MyReferencePath::normalizeAngle(double angle) {
    while(angle>PI){
        angle-=2.0*PI;
    }
    while(angle<-PI){
        angle+=2.0*PI;
    }
    return angle;
}


