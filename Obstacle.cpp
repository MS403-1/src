#include "Obstacle.h"
#include "Params.h"
#include "Position&Control.h"

void ObstacleAvoidance(double& velocity, double& omega, int robotId){
    for(int j = 0; j < ROBOT_NUM; j++){
	        if(robotId == j){
	            continue;
	        }
	        double face_angle = atan2((PositionGetY()(j) - PositionGetY()(robotId)), (PositionGetX()(robotId) - PositionGetX()(robotId))) - PositionGetTheta()(robotId);
	        double y_distance = std::fabs((PositionGetY()(j) - PositionGetY()(robotId)));
	        double x_distance = std::fabs((PositionGetX()(j) - PositionGetX()(robotId)));
	        double distance = sqrt(x_distance*x_distance + y_distance*y_distance);
	        //ROS_INFO_STREAM(i << " to " << j <<" face angle: " << face_angle << " y distance: " << y_distance);
			//ROS_INFO_STREAM("this is test cpp file");
	        if ((std::fabs(face_angle) < 0.2/y_distance && std::fabs(face_angle) < 0.45 && y_distance < 0.6) || (distance < 0.15 && std::fabs(face_angle)<1.57)){
	            if (distance < 0.25){
	                velocity = -0.1 * velocity;
	                omega = omega + 0.15;
	            }
	            else{
	                velocity = 0.1 * velocity;
	                omega = omega + 0.15;
	            }
	        }
    }
}