cmake_minimum_required(VERSION 3.21)
project(garyRobotics_CPP)

set(CMAKE_CXX_STANDARD 14)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OsqpEigen REQUIRED)

find_package(OpenGL REQUIRED)
include_directories(./MPC)
include_directories(MPC/extern/imgui)
include_directories(MPC/extern/implot)
include_directories(MPC/extern/glfw/include)
include_directories(MPC/extern/glad/include)

add_subdirectory(MPC/extern/glad)
add_subdirectory(MPC/extern/glfw)
add_subdirectory(MPC/extern/imgui)
add_subdirectory(MPC/extern/implot)

# PID
add_executable(pid_demo PID/main.cpp PID/PID_controller.cpp utils/Robot.cpp)
target_link_libraries(pid_demo PRIVATE garyRobotics_CPP)

# lqr
add_executable(lqr_demo LQR/main.cpp  LQR/LQRControl.cpp utils/MyReferencePath.cpp utils/Robot.cpp)
target_link_libraries(lqr_demo PRIVATE garyRobotics_CPP)

# MPC -- double integral
add_executable(doubleint_mpc_demo MPC/DoubleInt.cpp
                                   MPC/mpc_solver/mpc_solver.cc
                                   MPC/qp_solver/qp_solver.cc
                                   MPC/qp_solver/active_set_qp_solver.cc)
target_link_libraries(doubleint_mpc_demo PRIVATE libglog.so)
target_link_libraries(doubleint_mpc_demo PRIVATE libqpOASES.so)
target_link_libraries(doubleint_mpc_demo PRIVATE glfw imgui glad  implot ${OPENGL_LIBRARIES})
target_link_libraries(doubleint_mpc_demo PRIVATE garyRobotics_CPP)

# MPC -- inverted pendulum
add_executable(invertedpendulum_mpc_demo MPC/InvertedPendulum.cpp
        MPC/mpc_solver/mpc_solver.cc
        MPC/qp_solver/qp_solver.cc
        MPC/qp_solver/active_set_qp_solver.cc)
target_link_libraries(invertedpendulum_mpc_demo PRIVATE libglog.so )
target_link_libraries(invertedpendulum_mpc_demo PRIVATE libqpOASES.so)
target_link_libraries(invertedpendulum_mpc_demo PRIVATE glfw imgui glad  implot ${OPENGL_LIBRARIES})
target_link_libraries(invertedpendulum_mpc_demo PRIVATE garyRobotics_CPP)

# MPC -- kinematic
add_executable(kine_mpc_demo MPC/RobotKinematicControl.cpp
        MPC/mpc_solver/mpc_solver.cc
        MPC/qp_solver/qp_solver.cc
        MPC/qp_solver/active_set_qp_solver.cc
        MPC/planning/DesireTrajectory.cpp
        MPC/polynomial/QuinticPolynomial.cpp
        MPC/polynomial/QuarticPolynomial.cpp
        MPC/MPCControler.cpp
        utils/Robot.cpp
        )
target_link_libraries(kine_mpc_demo PRIVATE libglog.so )
target_link_libraries(kine_mpc_demo PRIVATE libqpOASES.so)
target_link_libraries(kine_mpc_demo PRIVATE glfw imgui glad  implot ${OPENGL_LIBRARIES})
target_link_libraries(kine_mpc_demo PRIVATE garyRobotics_CPP)

# MPC -- dynamic
add_executable(dyn_mpc_demo MPC/RobotDynControl.cpp
        MPC/mpc_solver/mpc_solver.cc
        MPC/qp_solver/qp_solver.cc
        MPC/qp_solver/active_set_qp_solver.cc
        MPC/planning/DesireTrajectory.cpp
        MPC/polynomial/QuinticPolynomial.cpp
        MPC/polynomial/QuarticPolynomial.cpp
        MPC/MPCControler.cpp
        utils/Robot.cpp
        )
target_link_libraries(dyn_mpc_demo PRIVATE libglog.so )
target_link_libraries(dyn_mpc_demo PRIVATE libqpOASES.so)
target_link_libraries(dyn_mpc_demo PRIVATE glfw imgui glad  implot ${OPENGL_LIBRARIES})
target_link_libraries(dyn_mpc_demo PRIVATE garyRobotics_CPP)

# pure pursuit
add_executable(pure_pursuit_demo Pure_Pursuit/main.cpp Pure_Pursuit/PurePursuit.cpp utils/Robot.cpp)
target_link_libraries(pure_pursuit_demo PRIVATE garyRobotics_CPP)

# front wheel feedback control
add_executable(front_wheel_feedback_demo Front_Wheel_Feedback/main.cpp Front_Wheel_Feedback/FrontWheelFeedback.cpp utils/Robot.cpp)
target_link_libraries(front_wheel_feedback_demo PRIVATE garyRobotics_CPP)

# real wheel feedback control
add_executable(rear_wheel_feedback_demo Rear_Wheel_Feedback/main.cpp  Rear_Wheel_Feedback/RearWheelFeedback.cpp utils/MyReferencePath.cpp utils/Robot.cpp)
target_link_libraries(rear_wheel_feedback_demo PRIVATE garyRobotics_CPP)