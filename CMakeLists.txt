cmake_minimum_required(VERSION 3.21)
project(garyRobotics_CPP)

set(CMAKE_CXX_STANDARD 14)

#include(GNUInstallDirs)
set(PACKAGE_NAME garyRobotics_CPP)

# Library target
add_library(garyRobotics_CPP INTERFACE)


# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(garyRobotics_CPP INTERFACE
        Python3::Python
        Python3::Module
        )
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(garyRobotics_CPP INTERFACE
            Python3::NumPy
            )
else()
    target_compile_definitions(garyRobotics_CPP INTERFACE WITHOUT_NUMPY)
endif()
install(
        TARGETS garyRobotics_CPP
        EXPORT install_targets
)



add_executable(main main.cpp)
target_link_libraries(main PRIVATE garyRobotics_CPP)


add_subdirectory(PathTracking bin)

