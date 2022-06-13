#include <EPuck.hpp>

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
    planStep = 0;
    stepEndTime = 0.0;
    isStepComplete = false;
    hasSampledWallDistance = false;
    isPlanComplete = false;

    turnDuration = M_PI_2 * AXLE_LENGTH / (2 * TURN_SPEED * WHEEL_RADIUS);
    forwardDuration = INTER_CELL_DIST / WHEEL_RADIUS / FORWARD_SPEED;

    turnPosDelta = AXLE_LENGTH / WHEEL_RADIUS;
    forwardPosDelta = INTER_CELL_DIST / WHEEL_RADIUS;

    leftSetSpeed = 0.0;
    leftSetSpeed = 0.0;

    for (int i = 0; i < NUM_DISTANCE_SENSORS; i++)
    {
        distSensors[i] = std::make_unique<webots::DistanceSensor>(*getDistanceSensor(distSensorNames[i]));
        distSensors[i]->enable(TIME_STEP);
    }

    numWallDistSamples = 0;

    IMU = std::make_unique<webots::InertialUnit>(*getInertialUnit("IMU"));

    IMU->enable(TIME_STEP);

    leftMotor = std::make_unique<webots::Motor>(*getMotor("left wheel motor"));
    rightMotor = std::make_unique<webots::Motor>(*getMotor("right wheel motor"));

    leftWheelSetPos = 0.0;
    rightWheelSetPos = 0.0;

    leftMotor->setPosition(leftWheelSetPos);
    rightMotor->setPosition(rightWheelSetPos);

    leftSetSpeed = 0.0;
    rightSetSpeed = 0.0;

    leftMotor->setVelocity(leftSetSpeed);
    rightMotor->setVelocity(rightSetSpeed);

    leftPosSensor = std::make_unique<webots::PositionSensor>(*getPositionSensor("left wheel sensor"));
    rightPosSensor = std::make_unique<webots::PositionSensor>(*getPositionSensor("right wheel sensor"));

    leftPosSensor->enable(TIME_STEP);
    rightPosSensor->enable(TIME_STEP);
}

void EPuck::ExecutePlan()
{
    Print("Executing motion plan...\n");

    while (step(TIME_STEP) != -1)
    {
        Run();
    }
}

void EPuck::Run()
{
    UpdateSensors();

    isStepComplete = IsWithinTolerance(leftWheelPos, leftWheelSetPos, POSITION_TOLERANCE) &&
                     IsWithinTolerance(rightWheelPos, rightWheelSetPos, POSITION_TOLERANCE);

    // Check if current step has been completed
    if (!isPlanComplete && isStepComplete)
    {
        if (hasSampledWallDistance)
        {
            PrintPlanState();
            fileHandler.WritePlanState(motionExecutionFilePath, planStep, row, column, heading, wallVisibility);

            if (planStep < plan->steps.size())
            {
                // Move to next step
                switch (plan->steps[planStep])
                {
                    case Movement::Left:
                        leftWheelSetPos = leftWheelPos - turnPosDelta;
                        rightWheelSetPos = rightWheelPos + turnPosDelta;
                        leftSetSpeed = TURN_SPEED;
                        rightSetSpeed = TURN_SPEED;

                        heading = LEFT_TURN_MAP.at(heading);
                        break;
                    case Movement::Forward:
                        leftWheelSetPos = leftWheelPos + forwardPosDelta;
                        rightWheelSetPos = rightWheelPos + forwardPosDelta;
                        leftSetSpeed = FORWARD_SPEED;
                        rightSetSpeed = FORWARD_SPEED;

                        row += MOVEMENT_DELTAS.at(heading)[0];
                        column += MOVEMENT_DELTAS.at(heading)[1];
                        break;
                    case Movement::Right:
                        leftWheelSetPos = leftWheelPos + turnPosDelta;
                        rightWheelSetPos = rightWheelPos - turnPosDelta;
                        leftSetSpeed = TURN_SPEED;
                        rightSetSpeed = TURN_SPEED;

                        heading = RIGHT_TURN_MAP.at(heading);
                        break;
                    default:
                        throw std::runtime_error(PRINT_PREFIX + "Invalid Movement value encountered.");
                        break;
                }

                hasSampledWallDistance = false;
                numWallDistSamples = 0;
                planStep++;
            }
            else
            {
                // All steps complete, plan is complete
                Print("Motion plan executed!\n");
                leftWheelSetPos = INFINITY;
                rightWheelSetPos = INFINITY;
                leftSetSpeed = 0.0;
                rightSetSpeed = 0.0;
                isPlanComplete = true;
            }
        }
        else
        {
            leftSetSpeed = 0.0;
            rightSetSpeed = 0.0;

            if (numWallDistSamples == WALL_DIST_SAMPLE_SIZE)
            {
                float leftWallDist = 0.0, frontWallDist = 0.0, rightWallDist = 0.0;

                for (int i = 0; i < WALL_DIST_SAMPLE_SIZE; i++)
                {
                    leftWallDist += wallDistSamples[0][i];
                    frontWallDist += wallDistSamples[1][i];
                    rightWallDist += wallDistSamples[2][i];
                }

                leftWallDist /= (float)WALL_DIST_SAMPLE_SIZE;
                frontWallDist /= (float)WALL_DIST_SAMPLE_SIZE;
                rightWallDist /= (float)WALL_DIST_SAMPLE_SIZE;

                wallVisibility[0] = leftWallDist <= WALL_DETECTION_THRESHOLD;
                wallVisibility[1] = frontWallDist <= WALL_DETECTION_THRESHOLD;
                wallVisibility[2] = rightWallDist <= WALL_DETECTION_THRESHOLD;

                hasSampledWallDistance = true;
            }
            else
            {
                wallDistSamples[0][numWallDistSamples] = distReadings[LEFT_DIST_SENSOR_INDEX];
                wallDistSamples[1][numWallDistSamples] = distReadings[FRONT_DIST_SENSOR_INDEX];
                wallDistSamples[2][numWallDistSamples] = distReadings[RIGHT_DIST_SENSOR_INDEX];

                numWallDistSamples++;
            }
        }
    }

    leftMotor->setPosition(leftWheelSetPos);
    rightMotor->setPosition(rightWheelSetPos);
    leftMotor->setVelocity(leftSetSpeed);
    rightMotor->setVelocity(rightSetSpeed);

    simTime = getTime();
}

int EPuck::GetTimeStep() const
{
    return TIME_STEP;
}

void EPuck::ReadMotionPlan(std::string filePath)
{
    motionPlanFilePath = filePath;
    Print("Reading in motion plan from " + motionPlanFilePath + "...\n");
    plan = fileHandler.ReadMotionPlan(motionPlanFilePath);
    Print("Motion Plan: " + plan->line + '\n');

    row = plan->initRow;
    column = plan->initColumn;
    heading = plan->initHeading;

    Print("Motion plan read in!\n");
}

void EPuck::SetUpExecutionFile(std::string filePath)
{
    motionExecutionFilePath = filePath;
    fileHandler.WriteExecutionHeader(motionExecutionFilePath);
}

void EPuck::UpdateSensors()
{
    for (int i = 0; i < NUM_DISTANCE_SENSORS; i++)
    {
        distReadings[i] = distSensors[i]->getValue();
    }

    roll = IMU->getRollPitchYaw()[0];
    pitch = IMU->getRollPitchYaw()[1];
    yaw = IMU->getRollPitchYaw()[2];

    leftWheelPos = leftPosSensor->getValue();
    rightWheelPos = rightPosSensor->getValue();
}

bool EPuck::IsWithinTolerance(double value, double target, double tolerance)
{
    return (value >= target - tolerance) && (value <= target + tolerance);
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

void EPuck::PrintWheelPositions()
{
    std::stringstream output;

    output << "Left Wheel Position (rad): " << leftWheelPos << ", Right Wheel Position (rad): " << rightWheelPos << std::endl;

    Print(output.str());
}
