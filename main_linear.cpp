/* 
 * Date: 2021-11-29
 * Description: To a line
 */

#include <swarm_robot_control.h>
#include <cmath>

/* Main function */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;

    /* First: Set ids of swarm robot based on Aruco marker */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);

    /* Set L Matrix */
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size());
    lap << 4, -1, -1, -1, -1,
        -1, 4, -1, -1, -1,
        -1, -1, 4, -1, -1,
        -1, -1, -1, 4, -1,
        -1, -1, -1, -1, 4;

    /* Convergence threshold */
    double conv_th = 0.5; // Threshold of angle, in rad
    double dis_th = 0.1;  // Threshold of distance, in m

    /* Velocity scale and threshold */
    double MAX_W = 1;    // Maximum angle velocity (rad/s)
    double MIN_W = 0.05; // Minimum angle velocity(rad/s)
    double MAX_V = 0.2;  // Maximum linear velocity(m/s)
    double MIN_V = 0.01; // Minimum linear velocity(m/s)
    double k_w = 0.1;    // Scale of angle velocity
    double k_v = 0.1;    // Scale of linear velocity

    /* Mobile robot poses and for next poses */
    Eigen::VectorXd cur_x(swarm_robot_id.size());
    Eigen::VectorXd cur_y(swarm_robot_id.size());
    Eigen::VectorXd cur_theta(swarm_robot_id.size());
    Eigen::VectorXd del_x(swarm_robot_id.size());
    Eigen::VectorXd del_y(swarm_robot_id.size());
    Eigen::VectorXd del_theta(swarm_robot_id.size());

    Eigen::VectorXd tmp1(swarm_robot_id.size());
    Eigen::VectorXd tmp2(swarm_robot_id.size());
    Eigen::VectorXd tmp3(swarm_robot_id.size());
    Eigen::VectorXd tmp4(swarm_robot_id.size());

    /* Set Shape */
    // Eigen::VectorXd target_x(swarm_robot_id.size());
    // Eigen::VectorXd target_y(swarm_robot_id.size());
    // target_x << 0, 1, 0, -1, 0;
    // target_y << 0, 0, 0, 0, 0;

    /* Get swarm robot poses firstly */
    std::vector<std::vector<double>> current_robot_pose(swarm_robot_id.size());
    swarm_robot.getRobotPose(current_robot_pose);

    /* x,y,theta */

    /* Convergence sign */
    bool is_angled = false; // Convergence sign of angle
    bool is_shaped = false; // Convergence sign of shape
    /* While loop */
    while (!(is_shaped && is_angled))
    {
        // while(1) {
        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            cur_x(i) = current_robot_pose[i][0];
            cur_y(i) = current_robot_pose[i][1];
            cur_theta(i) = current_robot_pose[i][2];
        }

        /* Judge whether shape reached */
        del_theta = -lap * cur_theta;
        del_y = -lap * cur_y;
        del_x = -lap * cur_x;
        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            tmp1(i)=(del_x(i)+del_y(i)*tan(cur_theta(i)))
            /((del_x(i)*del_x(i)+del_y(i)*del_y(i))*(1+tan(cur_theta(i))*tan(cur_theta(i))));
        }

        if (del_theta.norm() < conv_th && del_y.norm() < dis_th)
            is_shaped = true;
        else
            is_shaped = false;

        /* Judge whether to same angle */
        if (del_theta.norm() < conv_th)
            is_angled = true;
        else
            is_angled = false;

        
     

        /* Swarm robot move */
        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            double w = del_theta(i) * k_w;
            tmp2(i) = swarm_robot.checkVel(w, MAX_W, MIN_W);
        }



        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            double v = tmp1(i) * k_v;
            v = swarm_robot.checkVel(v, MAX_V, MIN_V);
            swarm_robot.moveRobot(i, v, tmp2(i));
        }
        ros::Duration(0.2).sleep();

        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);
    }

    /* Stop all robots */
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}
