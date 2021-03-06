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

typedef struct{
    double theta;
    double cost;
}formation_cost_t;

typedef Eigen::VectorXd* (form_info_t)[2];

extern Eigen::VectorXd expectedX;
extern Eigen::VectorXd expectedY;

void FormationChoose();
void FormationChoose(int formationIndex);
void FormationChoose(int formationIndex, double theta);
void FormationChooseDirect(int formationIndex);

#endif //SRC_FORMATION_H
