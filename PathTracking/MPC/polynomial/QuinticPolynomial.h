//
// Created by yyc on 2020/4/25.
//

#ifndef VEHSIM_QUINTICPOLYNOMIAL_H
#define VEHSIM_QUINTICPOLYNOMIAL_H
#include <Eigen/Dense>

class QuinticPolynomial {
public:
    QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T);
    double evalValue(double t);
    double evalDValue(double t);
private:
    Eigen::Matrix<double, 1, 6> lat_qp;
};


#endif //VEHSIM_QUINTICPOLYNOMIAL_H
