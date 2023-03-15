//
// Created by yyc on 2020/4/25.
//

#include "QuinticPolynomial.h"
QuinticPolynomial::QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T)
{
    double a0 = xs;
    double a1 = vxs;
    double a2 = axs / 2;
    Eigen::MatrixXd A(3, 3);
    A(0, 0) = T*  T*  T;
    A(0, 1) = T*  T*  T*  T;
    A(0, 2) = T * T*  T*  T*  T;

    A(1, 0) = 3*T * T;
    A(1, 1) = 4*T * T*  T;
    A(1, 2) = 5*T * T*  T*  T;

    A(2, 0) = 6*T ;
    A(2, 1) = 12*T * T;
    A(2, 2) = 20*T * T*  T;
    Eigen::MatrixXd b(3, 1);
    b(0, 0) = xe - a0 - a1 * T - a2 * T *T;
    b(1, 0) = vxe - a1 - 2 * a2*T;
    b(2, 0) = axe - 2 * a2;
    Eigen::MatrixXd x = A.inverse()*b;

    lat_qp(0, 0) = a0;
    lat_qp(0, 1) = a1;
    lat_qp(0, 2) = a2;
    lat_qp(0, 3) = x(0,0);
    lat_qp(0, 4) = x(1,0);
    lat_qp(0, 5) = x(2,0);
}
double QuinticPolynomial::evalValue(double t)
{
    return lat_qp(0, 0) + lat_qp(0, 1)*t + lat_qp(0, 2)*t*t + lat_qp(0, 3)*t*t*t + lat_qp(0, 4)*t*t*t*t + lat_qp(0, 5)*t*t*t*t*t;;
}
double QuinticPolynomial::evalDValue(double t)
{
    return lat_qp(0, 1) + lat_qp(0, 2)*t * 2 + lat_qp(0, 3)*t*t * 3 + lat_qp(0, 4)*t*t*t * 4 + lat_qp(0, 5)*t*t*t*t * 5;
}
