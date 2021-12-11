//
// Created by bddwy on 2021/12/10.
//

#include "Formation.h"
#include "Hungary.h"

double Map[N][N]; //邻接矩阵存图


double polarTar[robot_num][2];
double polaradd[robot_num][2];


Eigen::VectorXd star_form_x(robot_num);
Eigen::VectorXd star_form_y(robot_num);

Eigen::VectorXd circ_form_x(robot_num);
Eigen::VectorXd circ_form_y(robot_num);

Eigen::VectorXd thro_form_x(robot_num);
Eigen::VectorXd thro_form_y(robot_num);


inline void InitFormationParas(){
    star_form_x << 0, 0.8, -0.8, 0, 0;
    star_form_y << 0, 0, 0, 0.8, -0.8;
    circ_form_x << 0, 0.761,0.470, -0.470, -0.761;
    circ_form_y << 0.8,0.247, -0.647, -0.470,0.247;
    thro_form_x << 0, 0.4, 0.4, 0.8, 0.8;
    thro_form_y << 0, 0.2, -0.2, 0.4, -0.4;
}

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

double besttheta(double theta_ini);

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

    std::cout << std::endl;
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

double sstarget[5][2];
double ssadd[5][2];

void FormationChoose(){

    InitFormationParas();

    double star_cost,circ_cost,thro_cost;

    for(int i = 0; i < 5; i++) {
        ssadd[i][0]= GetCurrentPose()[i][0];
        ssadd[i][1] = GetCurrentPose()[i][1];
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

}

