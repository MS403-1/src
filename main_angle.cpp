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
#include "KeyboardCtrl.h"
#include "RVO.h"
#include "PathPlanning.h"

using namespace std;

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

    for(auto i = 0; i < OBSTACLE_NUM; i++){
        agent_id[i + ROBOT_NUM] = obstacle_id[i];
    }


/*    for(auto it : obstacle_id){
        agent_id.push_back(it);
    }*/

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

    PositionRefresh(swarm_robot);

    CenterPositionRefresh();

    PathInitialization(SIMPLE_PLAN, {12, 12, 0});



    /* While loop */
    while(true) {

        /* Get swarm robot poses */
        PositionRefresh(swarm_robot);


        static int cnt = 10;
        cnt++;
        if(cnt > 10){
            cnt = 0;
            FormationChooseDirect(0);
            PathExec();
        }

        /* Judge whether reached */
        //ControlTheta(-lap * PositionGetTheta());
        ControlY(-lap * (PositionGetY() - expectedY));
        ControlX(-lap * (PositionGetX() - expectedX));
        //std::cout << "cur_x" << "= " << cur_x << std::endl;
        //std::cout << "del_x" << "= " << del_x << std::endl;

        //if(StopCondition()) break;

        /* Swarm robot move */
        vector<double> v_x(ROBOT_NUM), v_y(ROBOT_NUM), v_direction(ROBOT_NUM);

        for(int i = 0; i < ROBOT_NUM; i++) {
            //determine the velocity
            v_x[i] = ControlGetX()[i] * k_v + (expectedCenter[0] - CenterGetX()[0]) * k_v;
            v_y[i] = ControlGetY()[i] * k_v + (expectedCenter[1] - CenterGetY()[0]) * k_v;
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
            if(dire_w < -3.14) dire_w += 6.28;
            if(dire_w > 3.14) dire_w -= 6.28;

            double v = d(i) * cos(dire_w);
            //std::cout << 'v' << i << "= " << v << std::endl;

            //determine the omega
            //if (i==1){std::cout<<"vx = "<<v_x<<"; vy = "<<v_y<<"; dire_w = "<<dire_w<<"; cur_theta = "<<cur_theta(i)<<"; v_direction = "<<v_direction<<std::endl;}
            double w = (20*sqrt(v_x[i]*v_x[i] + v_y[i]*v_y[i])*dire_w)*k_w;
            w = swarm_robot.checkVel(w, MAX_W, MIN_W);
            v = swarm_robot.checkVel(v, MAX_V, MIN_V);
            swarm_robot.moveRobot(i, v, w);
        }

      //  KeyboardCtrl(&nh,&swarm_robot,4);

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();
    }

    /* Stop all robots */
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}


/*std::cout << "=================check111============================" << std::endl;
std::cout << "=================check222============================" << std::endl;
std::cout << "=================check333============================" << std::endl;
std::cout << "=================check444============================" << std::endl;
std::cout << "=================check555============================" << std::endl;*/