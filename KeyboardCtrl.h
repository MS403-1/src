//
// Created by majiayue on 2021/12/17.
//

#ifndef GAZEBO_SWARM_ROBOT_TB3_KEYBOARDCTRL_H
#define GAZEBO_SWARM_ROBOT_TB3_KEYBOARDCTRL_H

#include <swarm_robot_control.h>
#include <cmath>
#include <iostream>

#include "Position&Control.h"
#include "Hungary.h"
#include "Formation.h"
#include "Params.h"

void KeyboardCtrl(ros::NodeHandle* _nh,SwarmRobot* _swRobot,int _id);


#endif //GAZEBO_SWARM_ROBOT_TB3_KEYBOARDCTRL_H
