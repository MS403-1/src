//
// Created by bddwy on 2021/12/11.
//

#include "Position&Control.h"

/** Mobile robot poses and for next poses */
Eigen::VectorXd cur_x(ROBOT_NUM);
Eigen::VectorXd cur_y(ROBOT_NUM);
Eigen::VectorXd cur_theta(ROBOT_NUM);
Eigen::VectorXd del_x(ROBOT_NUM);
Eigen::VectorXd del_y(ROBOT_NUM);
Eigen::VectorXd del_theta(ROBOT_NUM);

/** Obstacle information */
Eigen::VectorXd obstacle_x(OBSTACLE_NUM);
Eigen::VectorXd obstacle_y(OBSTACLE_NUM);
Eigen::VectorXd obstacle_theta(OBSTACLE_NUM);

/** Center information */
Eigen::VectorXd centerPosition[2];
Eigen::VectorXd positionToCenter[2];

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
    if(OBSTACLE_NUM == 0) return;
    for(auto i = ROBOT_NUM; i < ROBOT_NUM + OBSTACLE_NUM; i++){
        obstacle_x(i) = current_robot_pose[i][0];
        obstacle_y(i) = current_robot_pose[i][1];
        obstacle_theta(i) = current_robot_pose[i][2];
    }
}

void CenterPositionRefresh(){
    /**
     * 减少IO次数，借用两个变量临时存储
     */
    positionToCenter[0] = PositionGetX();
    positionToCenter[1] = PositionGetY();

    centerPosition[0] = centerPosition[0].setOnes(ROBOT_NUM) * (positionToCenter[0].sum() / ROBOT_NUM);
    centerPosition[1] = centerPosition[1].setOnes(ROBOT_NUM) * (positionToCenter[1].sum() / ROBOT_NUM);
    positionToCenter[0] -= centerPosition[0];
    positionToCenter[1] -= centerPosition[1];
}

const Eigen::VectorXd& PositionGetX(){
    return cur_x;
}

const Eigen::VectorXd& PositionGetY(){
    return cur_y;
}

const Eigen::VectorXd& PositionGetTheta(){
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

const Eigen::VectorXd& ObstacleGetX(){
    return obstacle_x;
}

const Eigen::VectorXd& ObstacleGetY(){
    return obstacle_y;
}

const Eigen::VectorXd& ObstacleGetTheta(){
    return obstacle_theta;
}

const Eigen::VectorXd& CenterGetX(){
    return centerPosition[0];
}

const Eigen::VectorXd& CenterGetY(){
    return centerPosition[1];
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
