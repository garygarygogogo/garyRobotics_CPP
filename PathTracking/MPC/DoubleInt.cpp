#include "mpc_solver/mpc_solver.h"
#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;
// double integral system
// d2y = u
// d [ x ] = [0  1] [x ] + [0] u
//   [dx ]   [0  0] [dx]   [1]
//   y   =  [ 1  0][ x]
//                 [dx]
void display(double* x_data, double* y_data, int len, double* x_data1, double* y_data1, int len1);
int main() {
    // number of state
    const int STATES = 2;
    // number of control input
    int CONTROLS = 1;
    // horizon
    const int HORIZON = 10;
    const double EPS = 0.01;
    const int MAX_ITER = 100;
    const double Ts = 0.1;
    Eigen::MatrixXd A(STATES, STATES);
    A << 0, 1, 0, 0;
    Eigen::MatrixXd B(STATES, CONTROLS);
    B << 0, 1;
    Eigen::MatrixXd C(STATES, 1);
    C << 0, 0;
    Eigen::MatrixXd Q(STATES, STATES);
    Q << 1, 0, 0, 1;
    Eigen::MatrixXd R(CONTROLS, CONTROLS);
    R << 1;
    // discrete state space
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(STATES, STATES) + A*Ts;
    Eigen::MatrixXd Bd = B * Ts;
    Eigen::MatrixXd lower_bound(CONTROLS, 1);
    lower_bound << -100;
    Eigen::MatrixXd upper_bound(CONTROLS, 1);
    upper_bound << 100;
    // initial state
    Eigen::MatrixXd initial_state(STATES, 1);
    initial_state << 0, 0;
    // reference state
    Eigen::MatrixXd reference_state(STATES, 1);
    reference_state << 1, 0;

    std::vector<Eigen::MatrixXd> reference(HORIZON, reference_state);
    Eigen::MatrixXd control_matrix(CONTROLS, 1);
    control_matrix << 0;
    std::vector<Eigen::MatrixXd> control(HORIZON, control_matrix);

    // safe for plot
    std::vector<double> ts;
    std::vector<double> ys;
    std::vector<double> us;
    
    double t = 0.0;
    Eigen::MatrixXd X = initial_state;
    for(int i = 0; i < 100; ++i)
    {
        ys.push_back(X(0));
        ts.push_back(t);

        // apllo MPC solver
        SolveLinearMPC(Ad, Bd, C, Q, R, lower_bound, upper_bound, X,
                       reference, EPS, MAX_ITER, &control);
        double u = control[0](0, 0);
        us.push_back(u);
        // obtain next state
        X = Ad*X +Bd*u;
        t += Ts;
    }

    // plot trajectory
    plt::plot(ts,us,"b--");
    plt::plot(ts,ys,"r");
    plt::grid(true);
    plt::ylim(-2.5,5.0);
    plt::pause(0.01);
    // save figure
    const char* figname = "./doubleint_mpc_demo.png";
    std::cout << "Saving result to: " << figname << std::endl;
    plt::save(figname);
    plt::show();
    return 0;
}
