//
// Created by bddwy on 2021/12/10.
//

#ifndef SRC_FORMATION_H
#define SRC_FORMATION_H

#include <iostream>
#include <cmath>

#include "Params.h"
#include "Position&Control.h"

extern Eigen::VectorXd star_form_x(robot_num);
extern Eigen::VectorXd star_form_y(robot_num);

extern Eigen::VectorXd circ_form_x(robot_num);
extern Eigen::VectorXd circ_form_y(robot_num);

extern Eigen::VectorXd thro_form_x(robot_num);
extern Eigen::VectorXd thro_form_y(robot_num);

void FormationChoose();
double targetCost(double target[][2], double addr[][2]);

#endif //SRC_FORMATION_H
