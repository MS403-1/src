//
// Created by bddwy on 2021/12/10.
//

#ifndef SRC_FORMATION_H
#define SRC_FORMATION_H

#include <iostream>
#include <cmath>
#include <swarm_robot_control.h>

#include "Params.h"
#include "Position&Control.h"

typedef Eigen::VectorXd* (form_info_t)[2];

extern Eigen::VectorXd expectedX;
extern Eigen::VectorXd expectedY;

void FormationChoose();
double targetCost(form_info_t);

#endif //SRC_FORMATION_H
