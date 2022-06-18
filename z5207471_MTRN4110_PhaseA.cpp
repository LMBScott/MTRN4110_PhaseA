// File:                 z5207471_MTRN4110_PhaseA.cpp
// Date:                 10/06/2022
// Description:          Main file for the execution of tasks required for Phase A of the MTRN4110 assignment project
// Author:               Lachlan Scott (z5207471)
// Development Platform: MacOS 12.4 Monterey, M1 Pro processor

#include <EPuck.hpp>

const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";

int main(int argc, char **argv)
{
    EPuck ePuck {};

    ePuck.ReadMotionPlan(MOTION_PLAN_FILE_NAME);
    ePuck.SetUpExecutionFile(MOTION_EXECUTION_FILE_NAME);
    ePuck.ExecutePlan();

    return 0;
}
