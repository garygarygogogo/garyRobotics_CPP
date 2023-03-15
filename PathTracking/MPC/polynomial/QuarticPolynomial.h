//
// Created by yyc on 2020/4/25.
//

#ifndef VEHSIM_QUARTICPOLYNOMIAL_H
#define VEHSIM_QUARTICPOLYNOMIAL_H
#include <Eigen/Dense>

class QuarticPolynomial {
public:
    QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double T);
    double evalValue(double t);
    double evalDValue(double t);
private:
    Eigen::Matrix<double, 1, 5> lon_qp;
};


#endif //VEHSIM_QUARTICPOLYNOMIAL_H
