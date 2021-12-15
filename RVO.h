//
// Created by bddwy on 2021/12/15.
//

#ifndef SRC_RVO_H
#define SRC_RVO_H

#include <cmath>

#include "Position&Control.h"

void RVO(const Eigen::VectorXd& x,const Eigen::VectorXd& y,const Eigen::VectorXd& v, Eigen::VectorXd& theta);
void VO(const Eigen::VectorXd& obstaclex,const Eigen::VectorXd& obstacley,const Eigen::VectorXd& x2,const Eigen::VectorXd& y2,const Eigen::VectorXd& v, Eigen::VectorXd& theta);

#endif //SRC_RVO_H
