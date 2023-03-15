//
// Created by gary on 2023/03/08.
//

#ifndef GARY_CPP_PUREPURSUIT_H
#define GARY_CPP_PUREPURSUIT_H
#include <iostream>
#include <vector>
#include<cmath>
#include <algorithm>

class PurePursuit {
public:
    double calTargetIndex(std::vector<double> robot_state, std::vector<std::vector<double>> refer_path, double l_d);

    double purePursuitControl(std::vector<double> robot_state, std::vector<double> current_ref_point, double l_d, double L);
};


#endif // GARY_CPP_PUREPURSUIT_H
