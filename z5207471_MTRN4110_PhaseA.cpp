// File:          PhaseA.cpp
// Date:
// Description:
// Author:
// Modifications:

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
