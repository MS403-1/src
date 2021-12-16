//
// Created by bddwy on 2021/12/16.
//

#include "PathPlanning.h"

typedef struct {
    double x;
    double y;
    double theta;
} path_plan_t;

std::vector<path_plan_t> Path;

void PathInitializationSimple(path_plan_t target){

    /**
     * Case 1: The desired path is parallel to x axis
     */
    if(abs(ObstacleGetY()[1] - ObstacleGetY()[0]) < 0.05){
        /** x, y -> x, (y0 + y1) / 2 */

        int cnt = ()

        /** x, y -> (x0 + x1) / 2, (y0 + y1) / 2 */

        /** x, y -> x*, y* */
    }

    /**
     * Case 2: General cases
     */
    else{
        /** x, y -> x, (y0 + y1) / 2 */

        /** x, y -> (x0 + x1) / 2, (y0 + y1) / 2 */

        /** x, y -> x*, y* */
    }
}

void (*pathPlanFunc[])(path_plan_t) = {
        PathInitializationSimple,
};

void PathInitialization(plan_type_e planType, path_plan_t target){
    pathPlanFunc[planType](target);
}


