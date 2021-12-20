//
// Created by bddwy on 2021/12/16.
//

#ifndef SRC_PATHPLANNING_H
#define SRC_PATHPLANNING_H

#include <cmath>

#include "Position&Control.h"
#include "Formation.h"

typedef enum {
    SIMPLE_PLAN = 0,
}plan_type_e;

typedef struct {
    double x;
    double y;
    double theta;
} path_plan_t;

void PathInitialization(plan_type_e planType, path_plan_t target);
void PathExec();

#endif //SRC_PATHPLANNING_H
