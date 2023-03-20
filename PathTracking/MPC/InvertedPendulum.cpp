#include "mpc_solver/mpc_solver.h"
#include <iostream>
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

// inverted pendulum system
void display(double* x_data, double* y_data, int len, double* x_data1, double* y_data1, int len1);
int main() {
    double M = 0.5;
    double m  = 0.2;
    double b = 0.1;
    double I = 0.018;
    double g = 9.8;
    double L = 0.3;
    double q = (M+m )*(I+m *L*L)-(m *L)*(m *L);
    double p = I*(m +M)+M*m*L*L;
    double A22 = -(I+m*L*L)*b/p;
    double A23 = m*m*g*L*L/p;
    double A42 =  -m *L*b/p;
    double A43 = m *g*L*(m +M)/p;
    double B2 = (I+m *L*L)/p;
    double B4 = m*L/p;
    // number of state
    const int STATES = 4;
    // number of control input
    int CONTROLS = 1;
    // horizon
    const int HORIZON = 10;
    const double EPS = 0.001;
    const int MAX_ITER = 1000;
    const double Ts = 0.1;
    Eigen::MatrixXd A(STATES, STATES);
    A << 0, 1, 0, 0, 0, A22, A23, 0, 0, 0, 0, 1, 0, A42, A43, 0;
    Eigen::MatrixXd B(STATES, CONTROLS);
    B << 0, B2, 0, B4;
    Eigen::MatrixXd C(STATES, 1);
    C << 0, 0, 0, 0;
    Eigen::MatrixXd Q(STATES, STATES);
    Q << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0 ,0, 1;
    Eigen::MatrixXd R(CONTROLS, CONTROLS);
    R << 0.1;
    // discrete state space
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(STATES, STATES) + A*Ts;
    Eigen::MatrixXd Bd = B * Ts;
    Eigen::MatrixXd lower_bound(CONTROLS, 1);
    lower_bound << -100;
    Eigen::MatrixXd upper_bound(CONTROLS, 1);
    upper_bound << 100;


    // initial state
    Eigen::MatrixXd initial_state(STATES, 1);
    initial_state << 0, 0, 0, 0;
    // reference state
    Eigen::MatrixXd reference_state(STATES, 1);
    reference_state << 1, 0, 0, 0;

    std::vector<Eigen::MatrixXd> reference(HORIZON, reference_state);
    Eigen::MatrixXd control_matrix(CONTROLS, 1);
    control_matrix << 0;
    std::vector<Eigen::MatrixXd> control(HORIZON, control_matrix);

    // save for plot
    std::vector<double> ts;
    std::vector<double> ys;
    std::vector<double> us;
    
    double t = 0.0;
    Eigen::MatrixXd X = initial_state;
    
    for(int i = 0; i < 200; ++i)
    {
        // MPC
        SolveLinearMPC(Ad, Bd, C, Q, R, lower_bound, upper_bound, X,
                       reference, EPS, MAX_ITER, &control);
        double u = control[0](0, 0);
        us.push_back(u);
        // predict next state
        X = Ad*X +Bd*u;
        ys.push_back(X(0));
        ts.push_back(t);
        t += Ts;
    }
    
    // plot trajectory
    plt::plot(ts,us,"b--");
    plt::plot(ts,ys,"r");
    plt::grid(true);
    plt::ylim(-2.5,5.0);
    plt::pause(0.01);
    // save figure
    const char* figname = "./invertedpendulum_mpc_demo.png";
    std::cout << "Saving result to: " << figname << std::endl;
    plt::save(figname);
    plt::show();
    return 0;
}
