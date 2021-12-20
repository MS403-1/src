//
// Created by bddwy on 2021/12/11.
//

#ifndef SRC_PARAMS_H
#define SRC_PARAMS_H

constexpr int N = 11;

/* First: Set ids of swarm robot based on Aruco marker */
constexpr int swarm_robot_id[] = {1, 2, 3, 4, 5};
constexpr int obstacle_id[] = {7, 8};

constexpr int ROBOT_NUM = sizeof(swarm_robot_id) / sizeof(int);
constexpr int OBSTACLE_NUM = sizeof(obstacle_id) / sizeof(int);

#endif //SRC_PARAMS_H
