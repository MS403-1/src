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

/* Convergence threshold */
constexpr double conv_th = 0.2;   // Threshold of angle, in rad
constexpr double dis_th = 0.05;    // Threshold of distance, in m
constexpr double angle_th = 0.05;  // Threshold of angle, in rad

/* Velocity scale and threshold */
constexpr double MAX_W = 1;       // Maximum angle velocity (rad/s)
constexpr double MIN_W = 0.05;    // Minimum angle velocity(rad/s)
constexpr double MAX_V = 0.2;     // Maximum linear velocity(m/s)
constexpr double MIN_V = 0.01;    // Minimum linear velocity(m/s)
constexpr double k_w = 0.12;       // Scale of angle velocity
constexpr double k_v = 0.1;       // Scale of linear velocity

#endif //SRC_PARAMS_H
