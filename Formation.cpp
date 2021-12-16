//
// Created by bddwy on 2021/12/10.
//

#include "Formation.h"
#include "Hungary.h"

double Map[N][N]; //邻接矩阵存图

Eigen::VectorXd form_star_x(ROBOT_NUM);
Eigen::VectorXd form_star_y(ROBOT_NUM);
Eigen::VectorXd form_circ_x(ROBOT_NUM);
Eigen::VectorXd form_circ_y(ROBOT_NUM);
Eigen::VectorXd form_thro_x(ROBOT_NUM);
Eigen::VectorXd form_thro_y(ROBOT_NUM);
Eigen::VectorXd form_line_x(ROBOT_NUM);
Eigen::VectorXd form_line_y(ROBOT_NUM);

Eigen::VectorXd* forms[][2] = {
        {&form_star_x, &form_star_y},
        {&form_circ_x, &form_circ_y},
        {&form_thro_x, &form_thro_y},
        {&form_line_x, &form_line_y},
};

constexpr int FORM_NUM = sizeof(forms) / sizeof(Eigen::VectorXd*) / 2;

inline void InitFormationParas(){
    form_star_x << 0, 0.8, -0.8, 0, 0;
    form_star_y << 0, 0, 0, 0.8, -0.8;
    form_circ_x << 0, 0.761, 0.470, -0.470, -0.761;
    form_circ_y << 0.8, 0.247, -0.647, -0.470, 0.247;
    form_thro_x << 0, 0.6, 0.6, 1.2, 1.2;
    form_thro_y << 0, 0.3, -0.3, 0.6, -0.6;
    form_line_x << 0, 0.6, -0.6, 1.2, -1.2;
    form_line_y << 0, 0, 0, 0, 0;
}

void map_init(Eigen::VectorXd *positionToCenter, form_info_t formInfo, double theta) {
    for(auto i = 0; i < ROBOT_NUM; i++) {
        for (auto j = 0; j < ROBOT_NUM; j++) {
            Map[i + 1][j + 1] = sqrt(   \
                    (positionToCenter[0](i) - cos(theta) * (*formInfo[0])(j) - sin(theta) * (*formInfo[1])(j)) * (positionToCenter[0](i) - cos(theta) * (*formInfo[0])(j) - sin(theta) * (*formInfo[1])(j)) \
                        + \
                    (positionToCenter[1](i) + sin(theta) * (*formInfo[0])(j) - cos(theta) * (*formInfo[1])(j)) * (positionToCenter[1](i) + sin(theta) * (*formInfo[0])(j) - cos(theta) * (*formInfo[1])(j)) \
                );
        }
    }
}

/**
 * @brief Calculate the minimum cost of all possible angle
 * @param formInfo The given formation
 * @return minimum cost and the theta
 */
formation_cost_t targetCost(Eigen::VectorXd *formInfo[2])
{
    double minCostTheta = -3.14;
    double minCost = 100000;

    constexpr int ITER_NUM = 20;
    for(auto thetaIter = -ITER_NUM; thetaIter < ITER_NUM; thetaIter++) {
        map_init(positionToCenter, formInfo, 3.14 * thetaIter / 20);
        double cost = calc(Map, ROBOT_NUM);
        if(cost < minCost){
            minCost = cost;
            minCostTheta = 3.14 * thetaIter / 20;
        }
    }

    /**     refresh P     */
    map_init(positionToCenter, formInfo, minCostTheta);
    calc(Map, ROBOT_NUM);

    formation_cost_t retVal = {minCostTheta, minCost};
    return retVal;
}

Eigen::VectorXd expectedX(ROBOT_NUM);
Eigen::VectorXd expectedY(ROBOT_NUM);

void FormationChoose(){

    InitFormationParas();
    CenterPositionRefresh();

    formation_cost_t formCostMin = targetCost(forms[0]);
    int formCostMinIndex = 0;
    storeP();

    for(auto formIndex = 1; formIndex < FORM_NUM; formIndex++){
        formation_cost_t formCost = targetCost(forms[formIndex]);
        if(formCost.cost < formCostMin.cost){
            formCostMinIndex = formIndex;
            formCostMin = formCost;
            storeP();
        }
    }

    for(auto robotIndex = 0; robotIndex < ROBOT_NUM; robotIndex++){
        expectedX[robotIndex] = CenterGetX(nowp[robotIndex]) + cos(formCostMin.theta) * (*forms[formCostMinIndex][0])(nowp[robotIndex]) + sin(formCostMin.theta) * (*forms[formCostMinIndex][1])(nowp[robotIndex]);
        expectedY[robotIndex] = CenterGetY(nowp[robotIndex]) - sin(formCostMin.theta) * (*forms[formCostMinIndex][0])(nowp[robotIndex]) + cos(formCostMin.theta) * (*forms[formCostMinIndex][1])(nowp[robotIndex]);
    }
}

void FormationChoose(int formationIndex){

    InitFormationParas();
    CenterPositionRefresh();

    formation_cost_t formCostMin = targetCost(forms[formationIndex]);
    storeP();

    for(auto robotIndex = 0; robotIndex < ROBOT_NUM; robotIndex++){
        expectedX[robotIndex] = CenterGetX(nowp[robotIndex]) + cos(formCostMin.theta) * (*forms[formationIndex][0])(nowp[robotIndex]) + sin(formCostMin.theta) * (*forms[formationIndex][1])(nowp[robotIndex]);
        expectedY[robotIndex] = CenterGetY(nowp[robotIndex]) - sin(formCostMin.theta) * (*forms[formationIndex][0])(nowp[robotIndex]) + cos(formCostMin.theta) * (*forms[formationIndex][1])(nowp[robotIndex]);
    }
}

void FormationChoose(int formationIndex, double theta){

    InitFormationParas();
    CenterPositionRefresh();

    map_init(positionToCenter, forms[formationIndex], theta);
    calc(Map, ROBOT_NUM);
    storeP();

    for(auto robotIndex = 0; robotIndex < ROBOT_NUM; robotIndex++){
        expectedX[robotIndex] = CenterGetX(nowp[robotIndex]) + cos(theta) * (*forms[formationIndex][0])(nowp[robotIndex]) + sin(theta) * (*forms[formationIndex][1])(nowp[robotIndex]);
        expectedY[robotIndex] = CenterGetY(nowp[robotIndex]) - sin(theta) * (*forms[formationIndex][0])(nowp[robotIndex]) + cos(theta) * (*forms[formationIndex][1])(nowp[robotIndex]);
    }
}

void FormationChooseDirect(int formationIndex){

    InitFormationParas();
    CenterPositionRefresh();

    for(auto robotIndex = 0; robotIndex < ROBOT_NUM; robotIndex++){
        expectedX[robotIndex] = CenterGetX(robotIndex) + (*forms[formationIndex][0])(robotIndex);
        expectedY[robotIndex] = CenterGetY(robotIndex) + (*forms[formationIndex][1])(robotIndex);
    }
}
