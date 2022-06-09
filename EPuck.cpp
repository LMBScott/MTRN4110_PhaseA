#include <EPuck.hpp>

const std::string EPuck::PLAN_INPUT_FILE = "../../MotionPlan.txt";
const std::string EPuck::EXECUTION_OUTPUT_FILE = "../../MotionExecution.csv";
const std::string EPuck::PRINT_PREFIX = "[z5207471_MTRN4110_PhaseA] ";

const std::array<std::string, EPuck::NUM_DISTANCE_SENSORS> EPuck::distSensorNames =
{
    "dsF",  // Front
    "ps0",  // Front right ~15deg from +x axis
    "ps1",  // Front right ~45deg from +x axis
    "dsR",  // Right
    "ps2",  // Right
    "ps3",  // Rear right ~20deg from -x axis
    "ps4",  // Rear left ~20deg from -x axis
    "dsL",  // Left
    "ps5",  // Left
    "ps6",  // Front right ~45deg from +x axis
    "ps7"   // Front right ~15deg from +x axis
};

const std::map<Direction, std::array<int, 2>> EPuck::MOVEMENT_DELTAS =
{
    { Direction::North, { -1, 0 } },
    { Direction::South, { 1, 0 } },
    { Direction::East, { 0, 1 } },
    { Direction::West, { 0, -1 } }
};

const std::map<Direction, Direction> EPuck::LEFT_TURN_MAP =
{
    { Direction::North, Direction::West },
    { Direction::South, Direction::East },
    { Direction::East, Direction::North },
    { Direction::West, Direction::South }
};

const std::map<Direction, Direction> EPuck::RIGHT_TURN_MAP =
{
    { Direction::North, Direction::East },
    { Direction::South, Direction::West },
    { Direction::East, Direction::South },
    { Direction::West, Direction::North }
};

const std::map<Direction, double> EPuck::HEADING_YAWS =
{
    { Direction::North, 0.0 },
    { Direction::South, M_PI },
    { Direction::East, -M_PI_2 },
    { Direction::West, M_PI_2 }
};

EPuck::EPuck()
{
    timeStep = (int)getBasicTimeStep();

    planStep = 0;
    stepEndTime = 0.0;
    isPlanComplete = false;

    turnDuration = M_PI_2 * AXLE_LENGTH / (2 * TURN_SPEED * WHEEL_RADIUS);
    forwardDuration = INTER_CELL_DIST / WHEEL_RADIUS / FORWARD_SPEED;

    leftSetSpeed = 0.0;
    leftSetSpeed = 0.0;

    for (int i = 0; i < NUM_DISTANCE_SENSORS; i++)
    {
        distSensors[i] = std::make_unique<webots::DistanceSensor>(*getDistanceSensor(distSensorNames[i]));
        distSensors[i]->enable(timeStep);
    }

    IMU = std::make_unique<webots::InertialUnit>(*getInertialUnit("IMU"));

    IMU->enable(timeStep);

    leftMotor = std::make_unique<webots::Motor>(*getMotor("left wheel motor"));
    rightMotor = std::make_unique<webots::Motor>(*getMotor("right wheel motor"));

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
}

void EPuck::Run()
{
    // Sensor reading updates
    for (int i = 0; i < NUM_DISTANCE_SENSORS; i++)
    {
        distReadings[i] = distSensors[i]->getValue();
    }

    wallVisibility[0] = distReadings[7] <= WALL_DETECTION_THRESHOLD;
    wallVisibility[1] = distReadings[0] <= WALL_DETECTION_THRESHOLD;
    wallVisibility[2] = distReadings[3] <= WALL_DETECTION_THRESHOLD;

    roll = IMU->getRollPitchYaw()[0];
    pitch = IMU->getRollPitchYaw()[1];
    yaw = IMU->getRollPitchYaw()[2];

    // Check if current step has been completed
    if (!isPlanComplete && simTime >= stepEndTime)
    {
        if (planStep < plan->steps.size())
        {
            int yawAdjustSign = (heading == Direction::South && yaw < 0) ? -1 : 1;

            if (numYawSamples < YAW_SAMPLE_SIZE)
            {
                leftSetSpeed = 0.0;
                rightSetSpeed = 0.0;
                yawAvg += yaw * yawAdjustSign;

                numYawSamples++;

                if (numYawSamples == YAW_SAMPLE_SIZE)
                {
                    yawAvg /= YAW_SAMPLE_SIZE;
                }
            }
            else if (yawAvg < yawAdjustSign * (HEADING_YAWS.at(heading) - TURN_ANGLE_TOLERANCE))
            {
                double correctionSpeed = std::min(std::abs(HEADING_YAWS.at(heading) - yawAvg) * AXLE_LENGTH / ((float)timeStep / 250 * WHEEL_RADIUS), MAX_MOTOR_SPEED);
                leftSetSpeed = -correctionSpeed;
                rightSetSpeed = correctionSpeed;

                yawAvg = 0;
                numYawSamples = 0;
            }
            else if (yawAvg > yawAdjustSign * (HEADING_YAWS.at(heading) + TURN_ANGLE_TOLERANCE))
            {
                double correctionSpeed = std::min(std::abs(HEADING_YAWS.at(heading) - yawAvg) * AXLE_LENGTH / ((float)timeStep / 250 * WHEEL_RADIUS), MAX_MOTOR_SPEED);
                leftSetSpeed = correctionSpeed;
                rightSetSpeed = -correctionSpeed;

                yawAvg = 0;
                numYawSamples = 0;
            }
            else
            {
                if (planStep == 0)
                {
                    Print("Executing motion plan...\n");
                }

                PrintPlanState();
                fileHandler.WritePlanState(EXECUTION_OUTPUT_FILE, planStep, row, column, heading, wallVisibility);

                // Move to next step
                switch (plan->steps[planStep])
                {
                    case Movement::Left:
                        stepEndTime = simTime + turnDuration;
                        leftSetSpeed = -TURN_SPEED;
                        rightSetSpeed = TURN_SPEED;

                        heading = LEFT_TURN_MAP.at(heading);
                        break;
                    case Movement::Forward:
                        stepEndTime = simTime + forwardDuration;
                        leftSetSpeed = FORWARD_SPEED;
                        rightSetSpeed = FORWARD_SPEED;

                        row += MOVEMENT_DELTAS.at(heading)[0];
                        column += MOVEMENT_DELTAS.at(heading)[1];
                        break;
                    case Movement::Right:
                        stepEndTime = simTime + turnDuration;
                        leftSetSpeed = TURN_SPEED;
                        rightSetSpeed = -TURN_SPEED;

                        heading = RIGHT_TURN_MAP.at(heading);
                        break;
                    default:
                        throw std::runtime_error(PRINT_PREFIX + "Invalid Movement value encountered.");
                        break;
                }
                numYawSamples = 0;
                yawAvg = 0;
                planStep++;
            }
        }
        else
        {
            // All steps complete, plan is complete
            Print("Motion plan executed!\n");
            leftSetSpeed = 0.0;
            rightSetSpeed = 0.0;
            isPlanComplete = true;
        }
    }

    leftMotor->setVelocity(leftSetSpeed);
    rightMotor->setVelocity(rightSetSpeed);

    simTime = getTime();
}

int EPuck::GetTimeStep() const
{
    return timeStep;
}

void EPuck::ReadMotionPlan()
{
    Print("Reading in motion plan from " + PLAN_INPUT_FILE + "...\n");
    plan = fileHandler.ReadMotionPlan(PLAN_INPUT_FILE);
    Print("Motion Plan: " + plan->line + '\n');

    row = plan->initRow;
    column = plan->initColumn;
    heading = plan->initHeading;

    Print("Motion plan read in!\n");
}

void EPuck::SetUpExecutionFile()
{
    fileHandler.WriteExecutionHeader(EXECUTION_OUTPUT_FILE);
}

void EPuck::Print(std::string message)
{
    std::cout << PRINT_PREFIX << message;
}

void EPuck::PrintPlanState()
{
    std::stringstream output;

    output << "Step: " << std::setfill('0') << std::setw(3) << planStep << ", "
           << "Row: " << row << ", "
           << "Column: " << column << ", "
           << "Heading: " << (char)heading << ", "
           << "Left Wall: " << (wallVisibility[0] ? 'Y' : 'N') << ", "
           << "Front Wall: " << (wallVisibility[1] ? 'Y' : 'N') << ", "
           << "Right Wall: " << (wallVisibility[2] ? 'Y' : 'N') << std::endl;

    Print(output.str());
}

void EPuck::PrintPlanDetails()
{
    std::cout<< "Full plan line: " << plan->line << std::endl;
    std::cout << "Initial position: row: " << plan->initRow << ", column: " << plan->initColumn << std::endl;
    std::cout << "Initial direction: " << (char)(plan->initHeading) << std::endl;
    std::cout << "Movements: ";

    for (auto& step : plan->steps)
    {
        std::cout << (char)step << (&step == &plan->steps.back() ? "\n" : ", ");
    }
}

void EPuck::PrintIMUReadings()
{
    std::stringstream output;

    output << "Yaw: " << yaw << ", "
           << "Pitch: " << pitch << ", "
           << "Roll: " << roll << std::endl;

    Print(output.str());
}

void EPuck::PrintDistanceReadings()
{
    std::stringstream output;

    for (int i = 0; i < NUM_DISTANCE_SENSORS - 1; i++)
    {
        output << distSensorNames[i] << ": " << distReadings[i] << ", ";
    }

    output << distSensorNames.back() << ": " << distReadings.back() << std::endl;

    Print(output.str());
}
