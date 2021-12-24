//
// Created by bddwy on 2021/12/16.
//

#include "PathPlanning.h"

/**
 * Part 1: Planning
 */

constexpr double PLAN_INTERVAL = 0.8;

std::vector<path_plan_t> centerPath;

double expectedCenter[2] = {0};

int PathNodeAdd(double x0, double y0, double x1, double y1){
    
    double distance = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    int cnt = 1 + int(distance / PLAN_INTERVAL);
    for(auto i = 1; i <= cnt; i++){
        
        centerPath.push_back({x0 + i * (x1 - x0) / cnt, y0 + i * (y1 - y0) / cnt, 0});
        
    }
    return cnt;
}

void PathInitializationSimple(path_plan_t target){
    
    centerPath.push_back({CenterGetX()[0], CenterGetY()[0], 0});
    centerPath.push_back({3, 3, 0});
    centerPath.push_back({3, 5, 0});
    centerPath.push_back({3, 7, 0});
    centerPath.push_back({3, 9, 0});
    
    /**
     * Case 1: The desired path is parallel to x axis
     */
    
    //if((ObstacleGetY()[1] - ObstacleGetY()[0])*(ObstacleGetY()[1] - ObstacleGetY()[0]) < 0.05*0.05){
        
//        /** x, y -> x, (y0 + y1) / 2 */
//        PathNodeAdd(CenterGetX()[0], CenterGetY()[0], CenterGetX()[0], (ObstacleGetY()[0] + ObstacleGetY()[1]) / 2);
//
//        /** x, y -> (x0 + x1) / 2, (y0 + y1) / 2 */
//        PathNodeAdd(CenterGetX()[0], (ObstacleGetY()[0] + ObstacleGetY()[1]) / 2, (ObstacleGetX()[0] + ObstacleGetX()[1]) / 2, (ObstacleGetY()[0] + ObstacleGetY()[1]) / 2);
//
//        /** x, y -> x*, y* */
//        PathNodeAdd((ObstacleGetX()[0] + ObstacleGetX()[1]) / 2, (ObstacleGetY()[0] + ObstacleGetY()[1]) / 2, target.x, target.y);
//
   // }


    /**
     * Case 2: General cases
     */
    //else{
        /** x, y -> x, (y0 + y1) / 2 */

        /** x, y -> (x0 + x1) / 2, (y0 + y1) / 2 */

        /** x, y -> x*, y* */
    //}

}

void (*pathPlanFunc[])(path_plan_t) = {
        PathInitializationSimple,
};

void PathInitialization(plan_type_e planType, path_plan_t target){
    pathPlanFunc[planType](target);
}

/**
 * Part 2: Execution
 */

constexpr double PATH_EXEC_ERR = 0.8;

bool PathGetToTarget(const std::vector<path_plan_t>::iterator it){
    cout << it->x << ' ' << it->y << endl;
    cout << CenterGetX()[0] << ' ' << CenterGetY()[0] << endl;
    cout << ((*it).x - CenterGetX()[0]) * ((*it).x - CenterGetX()[0]) + ((*it).y - CenterGetY()[0]) * ((*it).y - CenterGetY()[0]) << endl;

    if(((*it).x - CenterGetX()[0]) * ((*it).x - CenterGetX()[0]) + ((*it).y - CenterGetY()[0]) * ((*it).y - CenterGetY()[0]) < PATH_EXEC_ERR * PATH_EXEC_ERR) {
        return true;
    }
    else return false;
}

void PathExec(){
    static auto path_it = centerPath.begin() + 1;
    if(PathGetToTarget(path_it)){
        if(path_it != centerPath.end()) path_it++;
    }

    expectedCenter[0] = path_it->x;
    expectedCenter[1] = path_it->y;

/*    for(auto i = 0; i < ROBOT_NUM; i++){
        expectedX[i] += (*path_it).x - CenterGetX()[i];
        expectedY[i] += (*path_it).y - CenterGetY()[i];

        cout << expectedX[i] << ' ' << expectedY[i] << endl;
    }*/
/*    expectedX += Eigen::VectorXd(ROBOT_NUM).setOnes() * ((*path_it).x - (*(path_it - 1)).x);
    expectedY += Eigen::VectorXd(ROBOT_NUM).setOnes() * ((*path_it).y - (*(path_it - 1)).y);*/
}
