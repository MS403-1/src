#ifndef SRC_OBSTACLE_H
#define SRC_OBSTACLE_H

#include <vector>
#include <swarm_robot_control.h>

#include "Params.h"
#include "Position&Control.h"

void ObstacleAvoidance(double& velocity, double& omega, int robotId);

#endif //SRC_OBSTACLE_H