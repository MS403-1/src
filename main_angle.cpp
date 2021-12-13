/* 
 * Date: 2021-11-29
 * Description: To a line
 */

#include <swarm_robot_control.h>
#include <cmath>
#include <iostream>

//double arctan(double y, double x);

#include "Position&Control.h"
#include "Hungary.h"
#include "Formation.h"


using namespace std;



/* First: Set ids of swarm robot based on Aruco marker */
std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};

/* Main function */
int main(int argc, char** argv) {

    ros::init(argc, argv, "swarm_robot_control_formation");

    ros::NodeHandle nh;

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);

    /* Set L Matrix */
    Eigen::MatrixXd lap(ROBOT_NUM, ROBOT_NUM);
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;


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





//    std::cout <<"star_form_x" << star_form_x[0] << star_form_x[1] << star_form_x[2] << star_form_x[3] << star_form_x[4] << std::endl;
//    std::cout <<"star_form_y" << star_form_y[0] << star_form_y[1] << star_form_y[2] << star_form_y[3] << star_form_y[4] << std::endl;

    Eigen::VectorXd d(ROBOT_NUM);
    Eigen::VectorXd d_(ROBOT_NUM);




    /* Convergence sign */
    bool is_angled = false;    // Convergence sign of angle
    bool is_shaped = false;    // Convergence sign of shape
    bool is_conv = false;      // Convergence sign of agents

    double theta_sum;
    int judge_cnt = 0;

    PositionRefresh(swarm_robot);
    FormationChoose();

    /* While loop */
    while(! is_conv) {

        static int cnt = 0;
        cnt++;
        if(cnt > 10){
            cnt = 0;
            FormationChoose();
        }

        /* Judge whether reached */
        ControlTheta(-lap * PositionGetTheta());
        ControlY(-lap * (PositionGetY() - expectedY));
        ControlX(-lap * (PositionGetX() - expectedX));
        //std::cout << "cur_x" << "= " << cur_x << std::endl;
        //std::cout << "del_x" << "= " << del_x << std::endl;

        judge_cnt = 0;
        for (int i = 0; i < ROBOT_NUM; i++){
            if(/*std::fabs(del_theta(i)) < angle_th && */std::fabs(ControlGetY()(i)) < dis_th && std::fabs(ControlGetX()(i)) < dis_th){
                judge_cnt++;
            }
            else if(i == 2){
                std::cout<<"del_x = "<<ControlGetX()(i)<<"; del_y = "<<ControlGetY()(i)<<std::endl;
            }
            if(judge_cnt == 5){
                is_conv = true;
            }
        }


        /* Swarm robot move */
        for(int i = 0; i < ROBOT_NUM; i++) {
            //determine the velocity
            double v_x = ControlGetX()(i) * k_v;
            double v_y = ControlGetY()(i) * k_v;
            double v_direction = atan2(v_y, v_x);
            double dire_w = v_direction - PositionGetTheta()(i);
            double v = sqrt(v_x*v_x + v_y*v_y) * cos(dire_w);
            //std::cout << 'v' << i << "= " << v << std::endl;

            //determine the omega
            //if (i==1){std::cout<<"vx = "<<v_x<<"; vy = "<<v_y<<"; dire_w = "<<dire_w<<"; cur_theta = "<<cur_theta(i)<<"; v_direction = "<<v_direction<<std::endl;}
            double w = (20*sqrt(v_x*v_x + v_y*v_y)*dire_w)*k_w;
            //std::cout << 'w' << i << "= "  << w << std::endl;

            //avoid face to face crash
            for(int j = 0; j < ROBOT_NUM; j++){
                if(i == j){
                    continue;
                }
                double face_angle = atan2((PositionGetY()(j) - PositionGetY()(i)), (PositionGetX()(j) - PositionGetX()(i))) - PositionGetTheta()(i);
                double y_distance = std::fabs((PositionGetY()(j) - PositionGetY()(i)));
                double x_distance = std::fabs((PositionGetX()(j) - PositionGetX()(i)));
                double distance = sqrt(x_distance*x_distance + y_distance*y_distance);
                //ROS_INFO_STREAM(i << " to " << j <<" face angle: " << face_angle << " y distance: " << y_distance);
                //ROS_INFO_STREAM("this is test cpp file");
                if (face_angle < 0.05/distance && face_angle < 0.1 && distance < 0.4){
                    v = 0.3*v;
                    w = w + 0.05*sqrt(ControlGetX()(i) * ControlGetX()(i) + ControlGetY()(i) * ControlGetY()(i)) / distance;
                }
            }

            //move the robot
            w = swarm_robot.checkVel(w, MAX_W, MIN_W);
            v = swarm_robot.checkVel(v, MAX_V, MIN_V);
            swarm_robot.moveRobot(i, v, w);
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();

        /* Get swarm robot poses */
        PositionRefresh(swarm_robot);
    }

    /* Stop all robots */
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}
