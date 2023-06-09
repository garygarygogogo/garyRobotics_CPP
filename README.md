# Table of Contents
  * [Path Tracking](#path-tracking)
    * [PID](#PID)
    * [Rear wheel feedback control](#rear-wheel-feedback-control)
    * [Front wheel feedback control](#front-wheel-feedback-control)
    * [Linear–quadratic regulator (LQR) steering control](#linearquadratic-regulator-lqr-steering-control)
    * [Model predictive speed and steering control](#model-predictive-speed-and-steering-control)
    * [Pure pursuit control](#pure-pursuit-control)


# Dependencies:
OS: Ubuntu 20.04

```markdown
- python3
- matplotlib
- cmake
- Eigen
- glog
- qpOASES
```

# How to build

build through Cmake

```shell
mkdir build
cd build
cmake ../
make
```

# Path Tracking
## Rear wheel feedback control
Path tracking simulation with rear wheel feedback steering control and PID speed control.

Ref:

- [A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles](https://arxiv.org/abs/1604.07446)
## Front wheel feedback control
Path tracking simulation with Stanley steering control and PID speed control.

Ref:


- [Automatic Steering Methods for Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
## Linear–quadratic regulator (LQR) steering control
Path tracking simulation with LQR speed and steering control.


Ref:

- [Towards fully autonomous driving: Systems and algorithms \- IEEE Conference Publication](http://ieeexplore.ieee.org/document/5940562/)
## Model predictive speed and steering control
Path tracking simulation with iterative linear model predictive speed and steering control.

Ref:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/model_predictive_speed_and_steer_control/Model_predictive_speed_and_steering_control.ipynb)

- [Real\-time Model Predictive Control \(MPC\), ACADO, Python \| Work\-is\-Playing](http://grauonline.de/wordpress/?page_id=3244)
## Pure pursuit control
Path tracking simulation with pure pursuit steering control.

Ref:

- [Implementation of the Pure Pursuit Path Tracking Algorithm](https://www.semanticscholar.org/paper/Implementation-of-the-Pure-Pursuit-Path-Tracking-Coulter/ee756e53b6a68cb2e7a2e5d537a3eff43d793d70)


