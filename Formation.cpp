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

void map_init(Eigen::VectorXd *positionToCenter, form_info_t formInfo, double theta) {
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

Eigen::VectorXd centerPosition[2];
Eigen::VectorXd positionToCenter[2];

void UpdateCenterPosition() {
    /**
     * 减少IO次数，借用两个变量临时存储
     */
    positionToCenter[0] = PositionGetX();
    positionToCenter[1] = PositionGetY();

    centerPosition[0] = centerPosition[0].setOnes(ROBOT_NUM) * (positionToCenter[0].sum() / ROBOT_NUM);
    centerPosition[1] = centerPosition[1].setOnes(ROBOT_NUM) * (positionToCenter[1].sum() / ROBOT_NUM);
    positionToCenter[0] -= centerPosition[0];
    positionToCenter[1] -= centerPosition[1];
}

formation_cost_t targetCost(Eigen::VectorXd *formInfo[2])
{
    map_init(positionToCenter, formInfo, 0);

    double cost = calc(Map, ROBOT_NUM);
    ROS_INFO("Cost = %f\r\n", cost);

    formation_cost_t retVal = {0, cost};
    return retVal;
}

Eigen::VectorXd expectedX(ROBOT_NUM);
Eigen::VectorXd expectedY(ROBOT_NUM);

void FormationChoose(){

    InitFormationParas();

    UpdateCenterPosition();

    double formCostMin = targetCost(forms[0]).cost;
    int formCostMinIndex = 0;
    storeP();

    for(auto formIndex = 1; formIndex < FORM_NUM; formIndex++){
        if(targetCost(forms[formIndex]).cost < formCostMin){
            formCostMinIndex = formIndex;
            storeP();
        }
    }

    /**
     * Todo: Rotation
     */
    for(auto robotIndex = 0; robotIndex < ROBOT_NUM; robotIndex++){
        expectedX[robotIndex] = centerPosition[0](nowp[robotIndex]) + (*forms[formCostMinIndex][0])(nowp[robotIndex]);
        expectedY[robotIndex] = centerPosition[1](nowp[robotIndex]) + (*forms[formCostMinIndex][1])(nowp[robotIndex]);
    }
}
