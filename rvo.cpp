#include "rvo.h"

using namespace std;

// function: testAllow

double angleConstraint(double omega)
{
    while (omega > 2 * Pi)
        omega -= 2 * Pi;
    while (omega < 0)
        omega += 2 * Pi;
    return omega;
}

bool testAllow(double targetw, double v0, double w0, double v1, double w1, double x, double y, bool staticobj)
{
    double w = atan2(y, x);
    double dw = atan(2 * robotR / sqrt(y * y + x * x));
    if (!staticobj)
    {
        double vx = v0 * cos(w1) - v1 * cos(w1);
        double vy = v0 * sin(w1) - v1 * sin(w1);
        double tarx = v0 * cos(targetw);
        double tary = v0 * sin(targetw);
        targetw = atan2(tary - vy / 2, tarx - vx / 2);
    }
    return angleConstraint(targetw - w + dw) >= angleConstraint(2 * dw);
}

bool testAllowInWorld(double targetw, int index, bool moveStop[])
{
    int ensure = 1;
    for (int i = 0; i < ROBOT_NUM; i++)
        if (i != index)
            ensure *= testAllow(targetw, swarmVelocity(index), swarmOmega(index), swarmVelocity(i), swarmOmega(i), cur_x(i) - cur_x(index), cur_y(i) - cur_y(index), moveStop[i]);
    for (int i = 0; i < Obstacle_NUM; i++)
        ensure *= testAllow(targetw, swarmVelocity(index), swarmOmega(index), 0, 0, Obstacle_x(i) - cur_x(index), Obstacle_y(i) - cur_y(index), 1);
    return ensure;
}

bool rvo_solve()
{
    bool moveStop[ROBOT_NUM] = {0};
    bool ans_exist = 1;
    swarmOmegaout = swarmOmega;
    for (int i = 0; i < ROBOT_NUM; i++)
    {
        double dw = 0;
        double dw2 = Pi / 10;
        int count = 0;
        if (swarmVelocity(i) < 0.02)
            moveStop[i] = 1;
        for (count = 0; count < 75; count++)
        {
            dw += dw2;
            if (dw >= Pi)
            {
                dw -= Pi;
                dw2 = dw2 / 2;
            }
            if (testAllowInWorld(swarmOmega[i] + dw, i, moveStop))
            {
                swarmOmegaout[i] = swarmOmega[i] + dw;
                break;
            }
            if (testAllowInWorld(swarmOmega[i] - dw, i, moveStop))
            {
                swarmOmegaout[i] = swarmOmega[i] - dw;
                break;
            }
        }
        if (count == 75)
        {
            moveStop[i] = 1;
            ans_exist = 0;
        }
    }
    if (!ans_exist)
        for (int i = 0; i < ROBOT_NUM; i++)
        {
            double dw = 0;
            double dw2 = Pi / 10;
            int count = 0;
            if (moveStop[i])
            {
                swarmOmegaout[i] = -swarmOmegaout[i];
            }
            else
                for (count = 0; count < 75; count++)
                {
                    dw += dw2;
                    if (dw >= Pi)
                    {
                        dw -= Pi;
                        dw2 = dw2 / 2;
                    }
                    if (testAllowInWorld(swarmOmega[i] + dw, i, moveStop))
                    {
                        swarmOmegaout[i] = swarmOmega[i] + dw;
                        break;
                    }
                    if (testAllowInWorld(swarmOmega[i] - dw, i, moveStop))
                    {
                        swarmOmegaout[i] = swarmOmega[i] - dw;
                        break;
                    }
                }
        }
    return ans_exist;
}