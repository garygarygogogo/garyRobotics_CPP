//
// Created by yyc on 2020/4/25.
//

#include "QuarticPolynomial.h"
QuarticPolynomial::QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double T)
{
    double a0 = xs;
    double a1 = vxs;
    double a2 = axs / 2;
    Eigen::MatrixXd A(2, 2);
    A(0, 0) = 3*T * T;
    A(0, 1) = 4*T * T*  T;

    A(1, 0) = 6 * T;
    A(1, 1) = 12 * T * T;
    Eigen::MatrixXd b(2, 1);
    b(0, 0) = vxe - a1 - 2 * a2*T;
    b(1, 0) = axe - 2 * a2;
    Eigen::MatrixXd x = A.inverse()*b;
    Eigen::MatrixXd m(1, 5);
    lon_qp(0, 0) = a0;
    lon_qp(0, 1) = a1;
    lon_qp(0, 2) = a2;
    lon_qp(0, 3) = x(0, 0);
    lon_qp(0, 4) = x(1, 0);
}

double QuarticPolynomial::evalValue(double t)
{
    return lon_qp(0, 0) + lon_qp(0, 1)*t + lon_qp(0, 2)*t*t + lon_qp(0, 3)*t*t*t + lon_qp(0, 4)*t*t*t*t;
}
double QuarticPolynomial::evalDValue(double t)
{
    return lon_qp(0, 1) + lon_qp(0, 2)*t * 2 + lon_qp(0, 3)*t*t * 3 + lon_qp(0, 4)*t*t*t * 4;
}
