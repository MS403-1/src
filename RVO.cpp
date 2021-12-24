//
// Created by bddwy on 2021/12/15.
//

#include "RVO.h"

constexpr double ROBOT_RADIUS = 0.5;
constexpr double DETECTION_LENGTH = 1;

inline int sign(double x)
{
    if(x > 0.0) return 1;
    else return -1;
}

inline double cot(double x){
    return cos(x) / sin(x);
}

/**
 * @brief Reciprocal Velocity Obstacle Algorithm
 * @param v The target velocity of all robots, generated by previous planning algorithm.
 * @param theta The target angle of all robots, generated by previous planning algorithm.
 * @return None. The input parameter theta has been modified.
 */
void RVO(const Eigen::VectorXd& v, Eigen::VectorXd& theta){

    double z[ROBOT_NUM][ROBOT_NUM][2] = {0}; //The relative position
    double u[ROBOT_NUM][ROBOT_NUM][2] = {0}; //The relative velosity
    double d[ROBOT_NUM][ROBOT_NUM] = {0}; //The minimum distance of center

    /**
     * Initialize the tables
     */
    for(auto i = 0; i < ROBOT_NUM; i++){
        for(auto j = 0; j < ROBOT_NUM; j++){
            z[i][j][0] = PositionGetX()(j) - PositionGetX()(i);
            z[i][j][1] = PositionGetY()(j) - PositionGetY()(i);
            u[i][j][0] = v(j) * cot(theta(j)) - v(i) * cot(theta(i));
            u[i][j][1] = v(j) * tan(theta(j)) - v(i) * tan(theta(i));
            d[i][j] = (u[i][j][0] * z[i][j][1] - u[i][j][1] * z[i][j][0]) / sqrt(u[i][j][0] * u[i][j][0] + u[i][j][1] * u[i][j][1]);
        }
    }

    /**
     *
     */
    for(auto i = 0; i < ROBOT_NUM; i++){
        for(auto j = 0; j < ROBOT_NUM; j++){
            if( (ROBOT_RADIUS > std::abs(d[i][j]))\
                && \
                (u[i][j][0] * z[i][j][0] + u[i][j][1] * z[i][j][1]<0) \
                && \
                (sqrt(z[i][j][0] * z[i][j][0] + z[i][j][1] * z[i][j][1]) < DETECTION_LENGTH)){
                theta(i) += sign(d[i][j]) * (ROBOT_RADIUS - d[i][j]) / 2 / sqrt(z[i][j][0] * z[i][j][0] + z[i][j][1] * z[i][j][1]);
            }
        }
    }
}

/**
 * @brief Velocity Obstacle Algorithm
 * @param v The target velocity of all robots, generated by previous planning algorithm.
 * @param theta The target angle of all robots, generated by previous planning algorithm.
 * @return None. The input parameter theta has been modified.
 */
void VO(const Eigen::VectorXd& v, Eigen::VectorXd& theta){

    if(OBSTACLE_NUM == 0) return;

    double z[ROBOT_NUM][OBSTACLE_NUM][2]; //The relative position
    double d[ROBOT_NUM][OBSTACLE_NUM]; //The minimum distance of center

    /**
     * Initialize the tables
     */
    for(auto i = 0; i < ROBOT_NUM; i++){
        for(auto j = 0; j < OBSTACLE_NUM; j++){
            z[i][j][0] = ObstacleGetX()(j) - PositionGetX()(i);
            z[i][j][1] = ObstacleGetY()(j) - PositionGetY()(i);
            d[i][j] = (v(i) * cot(theta(j)) * z[i][j][1] - v(i) * tan(theta(i)) * z[i][j][0]) / v(i);
        }
    }

    for(auto i = 0; i < ROBOT_NUM; i++){
        for(auto j = 0; j < OBSTACLE_NUM; j++){
            if( (ROBOT_RADIUS > std::abs(d[i][j]))\
                && \
                (v(i) * cot(theta(j)) * z[i][j][0] + v(i) * tan(theta(j)) * z[i][j][1] < 0) \
                && \
                (sqrt(z[i][j][0] * z[i][j][0] + z[i][j][1] * z[i][j][1]) < DETECTION_LENGTH)){
                theta(i) +=  (ROBOT_RADIUS * sign(d[i][j]) - d[i][j]) / v(i);
            }
        }
    }
}
