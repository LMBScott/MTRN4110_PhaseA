// File:          PhaseA.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <EPuck.hpp>

int main(int argc, char **argv)
{
    EPuck ePuck {};
    int timeStep = ePuck.GetTimeStep();

    ePuck.ReadMotionPlan();
    ePuck.SetUpExecutionFile();

    while (ePuck.step(timeStep) != -1)
    {
        ePuck.Run();
    };

    return 0;
}
