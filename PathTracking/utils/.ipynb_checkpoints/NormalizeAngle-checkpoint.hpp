//
// Created by gary on 2023/03/08.
//

#ifndef CHHROBOTICS_CPP_NORMALIZEANGLE_HPP
#define CHHROBOTICS_CPP_NORMALIZEANGLE_HPP

#define PI 3.1415926

/**
 * angle normalization
 * @param angle
 * @return
 */
double normalizeAngle(double angle) {
    while(angle>PI){
        angle-=2.0*PI;
    }
    while(angle<-PI){
        angle+=2.0*PI;
    }
    return angle;
}
#endif //CHHROBOTICS_CPP_NORMALIZEANGLE_HPP
