
#ifndef SRC_RVO_H
#define SRC_RVO_H


#include <iostream>
#include <cmath>
#include "Position&Control.h"
#include "Params.h"//need the number of obstacles 'Obstacle_NUM' in this


#define robotR 0.2
#define Pi 3.1415926
//the following varibles all defined in Position&Control.h
extern Eigen::VectorXd cur_x;
extern Eigen::VectorXd cur_y;

extern Eigen::VectorXd swarmVelocity;
extern Eigen::VectorXd swarmOmega;

extern Eigen::VectorXd Obstacle_x;
extern Eigen::VectorXd Obstacle_y;

extern Eigen::VectorXd swarmOmegaout;//the output of rvo


double angleConstraint(double omega);
bool testAllow(double targetw, double v0, double w0, double v1, double w1, double x, double y, bool staticobj);
bool testAllowInWorld(double targetw, int index, bool moveStop[]);
bool rvo_solve();//just use this in main()

#endif //SRC_rvo_H