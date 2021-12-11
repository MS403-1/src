/* 
 * Date: 2021-11-29
 * Description: To a line
 */

#include <swarm_robot_control.h>
#include <cmath>
#include <math.h>
#include <iostream>

//double arctan(double y, double x);

#include "Hungary.h"
//#include "Formation.h"


using namespace std;

double Map[N][N]; //邻接矩阵存图




double polarTar[robot_num][2];
double polaradd[robot_num][2];
void map_init(double target[][2], double addr[][2], double theta)
{

    for (int i = 0; i < robot_num; i++)
    {
        polarTar[i][0] = sqrt(target[i][0] * target[i][0] + target[i][1] * target[i][1]);
        polarTar[i][1] = atan2(target[i][1], target[i][0]);
        polaradd[i][0] = sqrt(addr[i][0] * addr[i][0] + addr[i][1] * addr[i][1]);
        polaradd[i][1] = atan2(addr[i][1], addr[i][0]);
    }

    for (int i = 0; i < robot_num; i++)
        for (int j = 0; j < robot_num; j++)
        {
            Map[i + 1][j + 1] = sqrt(polarTar[j][0] * polarTar[j][0] + polaradd[i][0] * polaradd[i][0] - 2 * polarTar[j][0] * polaradd[i][0] * cos(polaradd[i][1] - (polarTar[j][1] + theta)));
        }
}

void storeP()
{
    for (int i = 0; i < robot_num; i++)
        nowp[i] = p[i];
}

double dtheta(double theta)
{
    double d = 0;
    for (int i = 0; i < robot_num; i++)
    {
        double tmp = sqrt(polarTar[p[i]][0] * polarTar[p[i]][0] + polaradd[i][0] * polaradd[i][0] - 2 * polarTar[p[i]][0] * polaradd[i][0] * cos(polaradd[i][1] - (polarTar[p[i]][1] + theta)));
        if (tmp == 0)
            d += sqrt(polarTar[i][0] * polarTar[p[i]][0]);
        else
            d += 2 * polaradd[i][0] * polarTar[p[i]][0] * sin(theta + polarTar[p[i]][1] - polaradd[i][1]) / tmp;
    }
    return d;
}

double besttheta(double theta_ini)
{
 
    double r = 0.02;
    double min = -3.14;
    double max = 3.14;
    double d = dtheta(theta_ini);;
    while (max - min > 0.0002&&d>0.01)
    {
        d = dtheta(theta_ini);
        if (d < 0)
        {
            if ( theta_ini - r * d < max)
            {
                min = theta_ini;
                theta_ini = theta_ini - r * d;
            }
            else
            {
                theta_ini = theta_ini - r * d;
                max = theta_ini + 6.28;
                r = r * 0.99;
            }
        }
        else
        {
            if ( theta_ini - r * d>min)
            {
                max = theta_ini;
                theta_ini = theta_ini - r * d;
            }
            else
            {
                theta_ini = theta_ini - r * d;
                max = theta_ini + 6.28;
                r = r * 0.99;
            }
        }
    }
    return (min + max) / 2;
}

double targetCost(double target[][2], double addr[][2])
{

    double centerX[2]={0,0};
    double centerY[2]={0,0};
    for (int i = 0; i < robot_num; i++)
    {
        centerX[0] += addr[i][0];
        centerY[0] += addr[i][1];
        centerX[1] += target[i][0];
        centerY[1] += target[i][1];
    
    }
    centerX[0] /= robot_num;
    centerY [0]/= robot_num;
    centerX[1] /= robot_num;
    centerY [1]/= robot_num;
    for (int i = 0; i < robot_num; i++)
    {
        addr[i][0] -= centerX[0];
        addr[i][1] -= centerY[0];
        target[i][0] -= centerX[1];
        target[i][1] -= centerY[1];
    }
    double theta = 0;
    double cost = 0;



    map_init(target, addr, theta);

    cost = calc(Map, robot_num);
    storeP();

    for (int i = -20; i < 20; i++)
    {

        map_init(target, addr, 3.14 * i / 20);
        

        double tmp  = calc(Map, robot_num);
std::cout<<tmp<<" ";
        if (tmp < cost)
        {
            cost = tmp;
            theta = 3.14 * i / 20;

            storeP();
        }

    }

    for (int i = 0; i < 5; i++)
    {
        std::cout << nowp[i] << " ";
    }

std::cout<<endl;
    double cost2;
    while (1)
    {
        double thetanew = besttheta(theta);
        map_init(target, addr, thetanew);
        cost2 = calc(Map, robot_num);
        if (cost2 < cost)
        {
    theta=thetanew;
            storeP();
            cost = cost2;
        }
        else{
for(int i=0;i<5;i++)
std::cout<<target[i][0]<<" "<<target[i][1]<<std::endl;
    for (int i = 0; i < 5; i++)
    {
        std::cout << nowp[i] << " ";

    double a=cos(theta)*target[i][0]-sin(theta)*target[i][1];
    double b=sin(theta)*target[i][0]+cos(theta)*target[i][1];
target[i][0]=a;
target[i][1]=b;
    }

std::cout<<theta<<std::endl;
            return cost;}
    }
}


/* Main function */
int main(int argc, char** argv) {

    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;
    
    /* First: Set ids of swarm robot based on Aruco marker */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);


    /* Set L Matrix */
    Eigen::MatrixXd lap(robot_num, robot_num);
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;


    /* Convergence threshold */
    double conv_th = 0.2;   // Threshold of angle, in rad
    double dis_th = 0.05;    // Threshold of distance, in m
    double angle_th = 0.05;  // Threshold of angle, in rad


    /* Velocity scale and threshold */
    double MAX_W = 1;       // Maximum angle velocity (rad/s)
    double MIN_W = 0.05;    // Minimum angle velocity(rad/s)
    double MAX_V = 0.2;     // Maximum linear velocity(m/s)
    double MIN_V = 0.01;    // Minimum linear velocity(m/s)
    double k_w = 0.12;       // Scale of angle velocity
    double k_v = 0.1;       // Scale of linear velocity


    /* Mobile robot poses and for next poses */
    Eigen::VectorXd cur_x(swarm_robot_id.size());
    Eigen::VectorXd cur_y(swarm_robot_id.size());
    Eigen::VectorXd cur_theta(swarm_robot_id.size());
    Eigen::VectorXd del_x(swarm_robot_id.size());
    Eigen::VectorXd del_y(swarm_robot_id.size());
    Eigen::VectorXd del_theta(swarm_robot_id.size());

    Eigen::VectorXd star_form_x(robot_num);
    Eigen::VectorXd star_form_y(robot_num);
    star_form_x << 0, 0.8, -0.8, 0, 0;
    star_form_y << 0, 0, 0, 0.8, -0.8;
    Eigen::VectorXd circ_form_x(robot_num);
    Eigen::VectorXd circ_form_y(robot_num);
    circ_form_x << 0, 0.761,0.470, -0.470, -0.761;
    circ_form_y << 0.8,0.247, -0.647, -0.470,0.247;
    Eigen::VectorXd thro_form_x(robot_num);
    Eigen::VectorXd thro_form_y(robot_num);
    thro_form_x << 0, 0.4, 0.4, 0.8, 0.8;
    thro_form_y << 0, 0.2, -0.2, 0.4, -0.4;

    double star_cost,circ_cost,thro_cost;




    std::cout <<"star_form_x" << star_form_x[0] << star_form_x[1] << star_form_x[2] << star_form_x[3] << star_form_x[4] << std::endl;
    std::cout <<"star_form_y" << star_form_y[0] << star_form_y[1] << star_form_y[2] << star_form_y[3] << star_form_y[4] << std::endl;

    Eigen::VectorXd d(robot_num);
    Eigen::VectorXd d_(robot_num);


    /* Get swarm robot poses firstly */
    std::vector<std::vector<double> > current_robot_pose(swarm_robot_id.size());
    swarm_robot.getRobotPose(current_robot_pose);


    /* x,y,theta */
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_x(i) = current_robot_pose[i][0];
        cur_y(i) = current_robot_pose[i][1];
        cur_theta(i) = current_robot_pose[i][2];
    }
    double sstarget[5][2];
    double ssadd[5][2];


    for(int i = 0; i < 5; i++) {
        ssadd[i][0]= current_robot_pose[i][0];
        ssadd[i][1] = current_robot_pose[i][1];
        sstarget[i][0] = star_form_x(i);
        sstarget[i][1] = star_form_y(i);
    }
    star_cost=targetCost(sstarget, ssadd);
    for(int i = 0; i <5; i++) {
        star_form_x(i) = sstarget[nowp[i]][0];
    star_form_y(i) = sstarget[nowp[i]][1];
    }    



    for(int i = 0; i < 5; i++) {
        sstarget[i][0] = circ_form_x(i);
        sstarget[i][1] = circ_form_y(i);
    }
    circ_cost=targetCost(sstarget, ssadd);
    for(int i = 0; i <5; i++) {
        circ_form_x(i) = sstarget[nowp[i]][0];
    circ_form_y(i) = sstarget[nowp[i]][1];
    }    





    for(int i = 0; i < 5; i++) {
        sstarget[i][0] = thro_form_x(i);
        sstarget[i][1] = thro_form_y(i);
    }
    thro_cost=targetCost(sstarget, ssadd);
    for(int i = 0; i <5; i++) {
        thro_form_x(i) = sstarget[nowp[i]][0];
    thro_form_y(i) = sstarget[nowp[i]][1];
    }    




    if(thro_cost<star_cost&&thro_cost<circ_cost)
    for(int i = 0; i <5; i++) {
                star_form_x=thro_form_x;
    star_form_y = thro_form_y;
    }    

    if(circ_cost<star_cost&&circ_cost<thro_cost)
    for(int i = 0; i <5; i++) {
                star_form_x=circ_form_x;
    star_form_y = circ_form_y;
    } 



    /* Convergence sign */
    bool is_angled = false;    // Convergence sign of angle
    bool is_shaped = false;    // Convergence sign of shape
    bool is_conv = false;      // Convergence sign of agents

    double theta_sum;
    int judge_cnt = 0;

    ros::Time begin = ros::Time::now();
    std::cout << begin << std::endl;

    /* While loop */
    while(! is_conv) {

        /* Judge whether reached */
        del_theta = -lap * cur_theta;
        del_y = -lap * (cur_y - star_form_y);
        del_x = -lap * (cur_x - star_form_x);
        //std::cout << "cur_x" << "= " << cur_x << std::endl;
        //std::cout << "del_x" << "= " << del_x << std::endl;

        judge_cnt = 0;
        for (int i = 0; i < robot_num; i++){
            if(/*std::fabs(del_theta(i)) < angle_th && */std::fabs(del_y(i)) < dis_th && std::fabs(del_x(i)) < dis_th){
                judge_cnt++;
            }
            else if(i == 2){
                std::cout<<"del_x = "<<del_x(i)<<"; del_y = "<<del_y(i)<<std::endl;
            }
            if(judge_cnt == 5){
                is_conv = true;
            }
        }      


        /* Swarm robot move */
        for(int i = 0; i < robot_num; i++) {
            //determine the velocity
            double v_x = del_x(i) * k_v;
            double v_y = del_y(i) * k_v;
            double v_direction = atan2(v_y, v_x);
            double dire_w = v_direction - cur_theta(i);
            double v = sqrt(v_x*v_x + v_y*v_y) * cos(dire_w);
            //std::cout << 'v' << i << "= " << v << std::endl;

            //determine the omega
            //if (i==1){std::cout<<"vx = "<<v_x<<"; vy = "<<v_y<<"; dire_w = "<<dire_w<<"; cur_theta = "<<cur_theta(i)<<"; v_direction = "<<v_direction<<std::endl;}
            double w = (20*sqrt(v_x*v_x + v_y*v_y)*dire_w)*k_w;
            //std::cout << 'w' << i << "= "  << w << std::endl;

            //avoid face to face crash
            for(int j = 0; j < robot_num; j++){
                if(i == j){
                    continue;
                }
                double face_angle = atan2((cur_y(j) - cur_y(i)), (cur_x(j) - cur_x(i))) - cur_theta(i);
                double y_distance = std::fabs((cur_y(j) - cur_y(i)));
                double x_distance = std::fabs((cur_x(j) - cur_x(i)));
                double distance = sqrt(x_distance*x_distance + y_distance*y_distance);
                //ROS_INFO_STREAM(i << " to " << j <<" face angle: " << face_angle << " y distance: " << y_distance);
                //ROS_INFO_STREAM("this is test cpp file");
                if (face_angle < 0.05/distance && face_angle < 0.1 && distance < 0.4){
                    v = 0.3*v;
                    w = w + 0.05*sqrt(del_x(i)*del_x(i) + del_y(i)*del_y(i))/distance;
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
        swarm_robot.getRobotPose(current_robot_pose);
        
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[i][2];
            cur_x(i) = current_robot_pose[i][0];
            cur_y(i) = current_robot_pose[i][1];
        }
    }

    ros::Time end = ros::Time::now();
    //ros::Time during = end - begin;
    //std::cout << "--------------------" << "processing time:" << during << "--------------------" << std::endl;

    /* Stop all robots */
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}
