/* 
 * Date: 2021-11-29
 * Description: To a line
 */

#include <swarm_robot_control.h>
#include <cmath>
#include <iostream>

#include "Position&Control.h"
#include "Hungary.h"
#include "Formation.h"
#include "RVO.h"

using namespace std;

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

bool StopCondition() {
    static int judge_cnt = 0;
    for (int i = 0; i < ROBOT_NUM; i++) {
        if (std::fabs(ControlGetY()(i)) < dis_th && std::fabs(ControlGetX()(i)) < dis_th) {
            judge_cnt++;
        }
        if (judge_cnt == 5) {
            return true;
        }
    }
    return false;
}

/* Main function */
int main(int argc, char** argv) {

    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;

    /**
     * April detection initialization
     */
    std::vector<int> agent_id(ROBOT_NUM + OBSTACLE_NUM);
/*    for(auto it : swarm_robot_id){
        agent_id.push_back(it);
    }*/
    for(auto i = 0; i < ROBOT_NUM; i++){
        agent_id[i] = swarm_robot_id[i];
    }

    for(auto it : obstacle_id){
        agent_id.push_back(it);
    }

    ROS_INFO("%zu %d %d\r\n", agent_id.size(), ROBOT_NUM, OBSTACLE_NUM);

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, agent_id);

    /* Set L Matrix */
    Eigen::MatrixXd lap(ROBOT_NUM, ROBOT_NUM);
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;

/*    lap <<  5, -1, -1, -1, -1, -1,
            -1, 5, -1, -1, -1, -1,
            -1, -1, 5, -1, -1, -1,
            -1, -1, -1, 5, -1, -1,
            -1, -1, -1, -1, 5, -1,
            -1, -1, -1, -1, -1, 5;*/

    Eigen::VectorXd d(ROBOT_NUM);
    Eigen::VectorXd d_(ROBOT_NUM);

    /* Convergence sign */
//    bool is_angled = false;    // Convergence sign of angle
//    bool is_shaped = false;    // Convergence sign of shape
//    bool is_conv = false;      // Convergence sign of agents

    /* While loop */
    while(true) {

        /* Get swarm robot poses */
        PositionRefresh(swarm_robot);

        static int cnt = 10;
        cnt++;
        if(cnt > 10){
            cnt = 0;
            FormationChooseDirect(0);
        }

        /* Judge whether reached */
        ControlTheta(-lap * PositionGetTheta());
        ControlY(-lap * (PositionGetY() - expectedY));
        ControlX(-lap * (PositionGetX() - expectedX));
        //std::cout << "cur_x" << "= " << cur_x << std::endl;
        //std::cout << "del_x" << "= " << del_x << std::endl;

        if(StopCondition()) break;

        /* Swarm robot move */
        vector<double> v_x(ROBOT_NUM), v_y(ROBOT_NUM), v_direction(ROBOT_NUM);

        for(int i = 0; i < ROBOT_NUM; i++) {
            //determine the velocity
            v_x[i] = ControlGetX()(i) * k_v;
            v_y[i] = ControlGetY()(i) * k_v;
            v_direction[i] = atan2(v_y[i], v_x[i]);

            d(i) = sqrt(v_x[i] * v_x[i] + v_y[i] * v_y[i]);
            d_(i) = v_direction[i];

            //std::cout << 'w' << i << "= "  << w << std::endl;
        }
            //avoid face to face crash
            //ObstacleAvoidance(v, w, i);
            RVO(d, d_);
            //VO(d, d_);
            //move the robot

        for(int i = 0; i < ROBOT_NUM; i++) {
            double dire_w = d_(i) - PositionGetTheta()(i);
            double v = d(i) * cos(dire_w);
            //std::cout << 'v' << i << "= " << v << std::endl;

            //determine the omega
            //if (i==1){std::cout<<"vx = "<<v_x<<"; vy = "<<v_y<<"; dire_w = "<<dire_w<<"; cur_theta = "<<cur_theta(i)<<"; v_direction = "<<v_direction<<std::endl;}
            double w = (20*sqrt(v_x[i]*v_x[i] + v_y[i]*v_y[i])*dire_w)*k_w;
            w = swarm_robot.checkVel(w, MAX_W, MIN_W);
            v = swarm_robot.checkVel(v, MAX_V, MIN_V);
            swarm_robot.moveRobot(i, v, w);
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();
    }

    /* Stop all robots */
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}
