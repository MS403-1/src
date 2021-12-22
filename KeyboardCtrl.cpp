//
// Created by majiayue on 2021/12/17.
//

#include "KeyboardCtrl.h"
constexpr double slope = 2.0;
constexpr int IT_ARR = 5;

float speed_v,speed_w;
int IT_CNT = 0;
void keyboardCallback(geometry_msgs::Twist twist){
    IT_CNT = 0;
    if(twist.linear.x >= 1){
        speed_v += MAX_V/slope;
        if(speed_v<0)speed_v=0;
    }else if(twist.linear.x <= -1){
        speed_v -= MAX_V/slope;
        if(speed_v>0)speed_v=0;
    }else {
        speed_v = 0;
    }
    if(twist.angular.z >= 1){
        speed_w += MAX_W/slope;
        if(speed_w<0)speed_w=0;
    }else if(twist.angular.z <= -1){
        speed_w -= MAX_W/slope;
        if(speed_w>0)speed_w=0;
    }else {
        speed_w = 0;
    }

    cout<<"ITHandle!!!!************"<<endl;
    cout<<speed_v<<"****************"<<speed_w<<endl;
}
/**
 * 此函数用于订阅小乌龟控制节点，并且作常规处理
 * @param _nh       ros::NodeHandle*类型，ros节点句柄
 * @param _swRobot  SwarmRobot*类型
 * @param _id       要控制的机器人id，从0开始，只能控制一台
 */
void KeyboardCtrl(ros::NodeHandle* _nh,SwarmRobot* _swRobot,int _id){
    static int firstEnter = 1;
    if(firstEnter == 1){
        cout<<"test111111111111111111111111111111111"<<endl;
        static ros::Subscriber person_info_sub = _nh->subscribe("/turtle1/cmd_vel", 10, keyboardCallback);
        firstEnter = 0;
    }
    speed_w = _swRobot->checkVel(speed_w, MAX_W, MIN_W);
    speed_v = _swRobot->checkVel(speed_v, MAX_V, MIN_V);
    _swRobot->moveRobot(_id, speed_v, speed_w);
    IT_CNT++;
    if(IT_CNT>=IT_ARR){speed_v = 0;speed_w =0;IT_CNT = 0;}

    cout<<"test2222222222222222222222222222222222"<<endl;
    ros::spinOnce();

}