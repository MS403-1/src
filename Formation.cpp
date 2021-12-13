//
// Created by bddwy on 2021/12/10.
//

#include "Formation.h"
#include "Hungary.h"

double Map[N][N]; //邻接矩阵存图


/*double polarTar[ROBOT_NUM][2];
double polaradd[ROBOT_NUM][2];*/

Eigen::VectorXd form_star_x(ROBOT_NUM);
Eigen::VectorXd form_star_y(ROBOT_NUM);
Eigen::VectorXd form_circ_x(ROBOT_NUM);
Eigen::VectorXd form_circ_y(ROBOT_NUM);
Eigen::VectorXd form_thro_x(ROBOT_NUM);
Eigen::VectorXd form_thro_y(ROBOT_NUM);

Eigen::VectorXd* forms[][2] = {
        {&form_star_x, &form_star_y},
        {&form_circ_x, &form_circ_y},
        {&form_thro_x, &form_thro_y},
};

constexpr int FORM_NUM = sizeof(forms) / sizeof(Eigen::VectorXd*) / 2;

inline void InitFormationParas(){
    form_star_x << 0, 0.8, -0.8, 0, 0;
    form_star_y << 0, 0.8, -0.8, 0, 0;
    form_circ_x << 0, 0.761, 0.470, -0.470, -0.761;
    form_circ_y << 0.8, 0.247, -0.647, -0.470, 0.247;
    form_thro_x << 0, 0.4, 0.4, 0.8, 0.8;
    form_thro_y << 0, 0.2, -0.2, 0.4, -0.4;
}

void map_init(Eigen::VectorXd *positionToCenter, form_info_t formInfo){
    for(auto i = 0; i < ROBOT_NUM; i++) {
        for (auto j = 0; j < ROBOT_NUM; j++) {
            Map[i + 1][j + 1] = sqrt(   \
                    (positionToCenter[0](i) - (*formInfo[0])(j)) * (positionToCenter[0](i) - (*formInfo[0])(j)) \
 + \
                    (positionToCenter[1](i) - (*formInfo[1])(j)) * (positionToCenter[1](i) - (*formInfo[1])(j)) \
                );
        }
    }

    for(auto i = 0; i < ROBOT_NUM; i++){
        for(auto j = 0; j < ROBOT_NUM; j++){
            ROS_INFO("%f ", Map[i + 1][j + 1]);
        }
        ROS_INFO("\r\n");
    }
}

//double besttheta(double theta_ini);

Eigen::VectorXd centorPosition[2];
Eigen::VectorXd positionToCenter[2];

void UpdateCenterPosition() {
    /**
     * 减少IO次数，借用两个变量临时存储
     */
    positionToCenter[0] = PositionGetX();
    positionToCenter[1] = PositionGetY();

    centorPosition[0] = centorPosition[0].setOnes(ROBOT_NUM) * (positionToCenter[0].sum() / ROBOT_NUM);
    centorPosition[1] = centorPosition[1].setOnes(ROBOT_NUM) * (positionToCenter[1].sum() / ROBOT_NUM);
    positionToCenter[0] -= centorPosition[0];
    positionToCenter[1] -= centorPosition[1];
}

double targetCost(form_info_t formInfo)
{
//    double theta = 0;

    map_init(positionToCenter, formInfo);

    double cost = calc(Map, ROBOT_NUM);
//    storeP();

/*    for (int i = -20; i < 20; i++)
    {
        map_init(position);

        double tmp  = calc(Map, ROBOT_NUM);
        std::cout<<tmp<<" ";
        if (tmp < cost)
        {
            cost = tmp;
            theta = 3.14 * i / 20;

            storeP();
        }

    }*/

    for (int i = 0; i < ROBOT_NUM; i++)
    {
        std::cout << nowp[i] << " ";
    }

    /*std::cout << std::endl;
    double cost2;
    while (true)
    {
        double thetanew = besttheta(theta);
        map_init();
        cost2 = calc(Map, ROBOT_NUM);
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
    }*/

    ROS_INFO("Cost = %f\r\n", cost);

    return cost;
}

/*double dtheta(double theta)
{
    double d = 0;
    for (int i = 0; i < ROBOT_NUM; i++)
    {
        double tmp = sqrt(polarTar[p[i]][0] * polarTar[p[i]][0] + polaradd[i][0] * polaradd[i][0] - 2 * polarTar[p[i]][0] * polaradd[i][0] * cos(polaradd[i][1] - (polarTar[p[i]][1] + theta)));
        if (tmp == 0)
            d += sqrt(polarTar[i][0] * polarTar[p[i]][0]);
        else
            d += 2 * polaradd[i][0] * polarTar[p[i]][0] * sin(theta + polarTar[p[i]][1] - polaradd[i][1]) / tmp;
    }
    return d;
}*/

/*double besttheta(double theta_ini)
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
}*/

Eigen::VectorXd expectedX(ROBOT_NUM);
Eigen::VectorXd expectedY(ROBOT_NUM);

void FormationChoose(){

    InitFormationParas();

    UpdateCenterPosition();

    double formCostMin = targetCost(forms[0]);
    int formCostMinIndex = 0;

    for(auto formIndex = 1; formIndex < FORM_NUM; formIndex++){
        if(targetCost(forms[formIndex]) < formCostMin){
            formCostMinIndex = formIndex;
            storeP();
        }
    }

    /**
     * Todo: Rotation
     */
    for(auto robotIndex = 0; robotIndex < ROBOT_NUM; robotIndex++){
        expectedX[robotIndex] = centorPosition[0](nowp[robotIndex]) + (*forms[formCostMinIndex][0])(nowp[robotIndex]);
        expectedY[robotIndex] = centorPosition[1](nowp[robotIndex]) + (*forms[formCostMinIndex][1])(nowp[robotIndex]);
    }
}
