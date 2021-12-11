//
// Created by bddwy on 2021/12/11.
//

#include "Position&Control.h"

extern std::vector<int> swarm_robot_id;

/* Mobile robot poses and for next poses */
Eigen::VectorXd cur_x(swarm_robot_id.size());
Eigen::VectorXd cur_y(swarm_robot_id.size());
Eigen::VectorXd cur_theta(swarm_robot_id.size());
Eigen::VectorXd del_x(swarm_robot_id.size());
Eigen::VectorXd del_y(swarm_robot_id.size());
Eigen::VectorXd del_theta(swarm_robot_id.size());

std::vector<std::vector<double>> current_robot_pose(swarm_robot_id.size());

extern SwarmRobot swarm_robot;

void PositionRefresh(SwarmRobot& swarm_robot){
    /* Get swarm robot poses firstly */

    swarm_robot.getRobotPose(current_robot_pose);


    /* x,y,theta */
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_x(i) = current_robot_pose[i][0];
        cur_y(i) = current_robot_pose[i][1];
        cur_theta(i) = current_robot_pose[i][2];
    }
}

Eigen::VectorXd& PositionGetX(){
    return cur_x;
}

Eigen::VectorXd& PositionGetY(){
    return cur_y;
}

Eigen::VectorXd& PositionGetTheta(){
    return cur_theta;
}

Eigen::VectorXd& ControlGetX(){
    return del_x;
}

Eigen::VectorXd& ControlGetY(){
    return del_y;
}

Eigen::VectorXd& ControlGetTheta(){
    return del_theta;
}

/** Todo：右值引用 */
void ControlX(const Eigen::VectorXd& vx){
    del_x = vx;
}

void ControlY(const Eigen::VectorXd& vy){
    del_y = vy;
}
void ControlTheta(const Eigen::VectorXd& w){
    del_theta = w;
}

std::vector<std::vector<double>>& GetCurrentPose(){
    return current_robot_pose;
}
