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

void FormationChoose();
double targetCost(double target[][2], double addr[][2]);

#endif //SRC_FORMATION_H
