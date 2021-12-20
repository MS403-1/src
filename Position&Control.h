//
// Created by bddwy on 2021/12/11.
//

#ifndef SRC_POSITION_CONTROL_H
#define SRC_POSITION_CONTROL_H

#include <vector>
#include <swarm_robot_control.h>

#include "Params.h"

void PositionRefresh(SwarmRobot& swarm_robot);
void CenterPositionRefresh();

const Eigen::VectorXd& PositionGetX();
const Eigen::VectorXd& PositionGetY();
const Eigen::VectorXd& PositionGetTheta();

Eigen::VectorXd& ControlGetX();
Eigen::VectorXd& ControlGetY();
Eigen::VectorXd& ControlGetTheta();

const Eigen::VectorXd& ObstacleGetX();
const Eigen::VectorXd& ObstacleGetY();
const Eigen::VectorXd& ObstacleGetTheta();

const Eigen::VectorXd& CenterGetX();
const Eigen::VectorXd& CenterGetY();

extern Eigen::VectorXd positionToCenter[2];

void ControlX(const Eigen::VectorXd& vx);
void ControlY(const Eigen::VectorXd& vy);
void ControlTheta(const Eigen::VectorXd& w);

#endif //SRC_POSITION_CONTROL_H
