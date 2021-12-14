//
// Created by bddwy on 2021/12/11.
//

#include "Position&Control.h"

extern std::vector<int> swarm_robot_id;

/* Mobile robot poses and for next poses */
Eigen::VectorXd cur_x(ROBOT_NUM);
Eigen::VectorXd cur_y(ROBOT_NUM);
Eigen::VectorXd cur_theta(ROBOT_NUM);
Eigen::VectorXd del_x(ROBOT_NUM);
Eigen::VectorXd del_y(ROBOT_NUM);
Eigen::VectorXd del_theta(ROBOT_NUM);

/* Obstacle information */
Eigen::VectorXd obstacle_x(OBSTACLE_NUM);
Eigen::VectorXd obstacle_y(OBSTACLE_NUM);
Eigen::VectorXd obstacle_theta(OBSTACLE_NUM);

/**
 * Warning: The following vector is assigned with static memory size, this may cause exception when index is over ROBOT_NUM
 */
std::vector<std::vector<double>> current_robot_pose(ROBOT_NUM + OBSTACLE_NUM);

extern SwarmRobot swarm_robot;

void PositionRefresh(SwarmRobot& swarm_robot){

    /* Get swarm robot poses firstly */
    swarm_robot.getRobotPose(current_robot_pose);

    /* x,y,theta */
    for(auto i = 0; i < ROBOT_NUM; i++) {
        cur_x(i) = current_robot_pose[i][0];
        cur_y(i) = current_robot_pose[i][1];
        cur_theta(i) = current_robot_pose[i][2];
    }

    /* Obstacle */
    for(auto i = ROBOT_NUM; i < ROBOT_NUM + OBSTACLE_NUM; i++){
        obstacle_x(i) = current_robot_pose[i][0];
        obstacle_y(i) = current_robot_pose[i][1];
        obstacle_theta(i) = current_robot_pose[i][2];
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

Eigen::VectorXd& ObstacleGetX(){
    return obstacle_x;
}

Eigen::VectorXd& ObstacleGetY(){
    return obstacle_y;
}

Eigen::VectorXd& ObstacleGetTheta(){
    return obstacle_theta;
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
