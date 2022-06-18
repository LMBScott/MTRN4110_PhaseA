// File:                 EPuck.hpp
// Date:                 10/06/2022
// Description:          Class definition for the EPuck robot class designed to control an e-puck robot in the WeBots simulator
// Author:               Lachlan Scott (z5207471)
// Development Platform: MacOS 12.4 Monterey, M1 Pro processor

#include <array>
#include <FileHandler.hpp>
#include <math.h>
#include <map>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>

class EPuck : public webots::Robot {
public:
    EPuck();
    void ExecutePlan();
    int GetTimeStep() const;                                            // Retrieve the basic time step of the simulation
    void ReadMotionPlan(std::string filePath);
    void SetUpExecutionFile(std::string filePath);
private:
    static constexpr int TIME_STEP {64};
    static constexpr int NUM_DISTANCE_SENSORS {11};                     // Total number of distance sensors on the e-puck robot
    static constexpr int NUM_WALL_DIST_SENSORS {3};                     // Number of wall distance sensors on the e-puck robot
    static constexpr int WALL_DIST_SAMPLE_SIZE {10};                    // Number of samples to be taken when measuring wall distance
    static constexpr int LEFT_DIST_SENSOR_INDEX {7};                    // Index of left distance sensor in distReadings array
    static constexpr int FRONT_DIST_SENSOR_INDEX {0};                   // Index of front distance sensor in distReadings array
    static constexpr int RIGHT_DIST_SENSOR_INDEX {3};                   // Index of right distance sensor in distReadings array
    static constexpr double MAX_MOTOR_SPEED {6.28};                     // Max motor speed of the e-puck robot, in rads/s
    static constexpr double TURN_SPEED {MAX_MOTOR_SPEED / 4};           // Speed of motors during a turn, in rads/s
    static constexpr double FORWARD_SPEED {MAX_MOTOR_SPEED / 2};        // Speed of motors moving forward, in rads/s
    static constexpr double AXLE_LENGTH {0.04489992};                   // Axle length of the e-puck robot, in meters
    static constexpr double WHEEL_RADIUS {0.01999252};                  // Wheel radius of the e-puck robot, in meters
    static constexpr double INTER_CELL_DIST {0.165};                    // Distance between maze cells, in meters
    static constexpr double WALL_THICKNESS {0.015};                     // Thickness of maze walls, in meters
    static constexpr double WALL_DETECTION_THRESHOLD {750.0};           // Minimum proximity sensor reading indicating wall presence
    static constexpr double POSITION_TOLERANCE {0.02};                  // Tolerance of motor position changes, in radians
    static const std::array<std::string, NUM_DISTANCE_SENSORS>
        distSensorNames;                                                // Name of each distance sensor on the e-puck robot
    static const std::string PRINT_PREFIX;                              // Prefix to be printed before each console output message
    static const std::map<Direction, std::array<int, 2>>
        MOVEMENT_DELTAS;                                                // Changes in row & column from movement along each heading
    static const std::map<Direction, Direction> LEFT_TURN_MAP;          // Resultant heading from a left turn starting at each heading
    static const std::map<Direction, Direction> RIGHT_TURN_MAP;         // Resultant heading from a right turn starting at each heading
    int planStep;                                                       // Current step of execution of the motion plan
    int row;                                                            // Current row location within the maze
    int column;                                                         // Current column location within the maze
    int numWallDistSamples;                                             // Current number of wall distance samples taken
    double simTime;                                                     // Stores current simulation time
    double turnPosDelta;                                                // Absolute change in position encoder value for a forward movement
    double forwardPosDelta;                                             // Absolute change in position encoder value for a turn
    double stepEndTime;                                                 // The simulation time at which the current step will be complete
    double leftSetSpeed;                                                // Set speed of the left motor, in rad/s
    double rightSetSpeed;                                               // Set speed of the right motor, in rad/s
    double leftWheelPos;                                                // Current position of the left wheel, in radians
    double rightWheelPos;                                               // Current position of the right wheel, in radians
    double leftWheelSetPos;                                             // Set position of the left wheel, in radians
    double rightWheelSetPos;                                            // Set position of the right wheel, in radians
    double roll;                                                        // Current roll reading of the IMU, in radians
    double pitch;                                                       // Current pitch reading of the IMU, in radians
    double yaw;                                                         // Current yaw reading of the IMU, in radians
    bool isStepComplete;                                                // Whether the current plan step has been completed
    bool hasSampledWallDistance;                                        // Whether the distance to each wall has been sampled
    bool isPlanComplete;                                                // Whether the motion plan has been completed
    FileHandler fileHandler;                                            // File handler for file I/O
    Direction heading;                                                  // The current heading of the e-puck (North, South, East or West)
    std::unique_ptr<MotionPlan> plan;                                   // The current motion plan
    std::unique_ptr<webots::InertialUnit> IMU;
    std::unique_ptr<webots::Motor> leftMotor;
    std::unique_ptr<webots::Motor> rightMotor;
    std::unique_ptr<webots::PositionSensor> leftPosSensor;
    std::unique_ptr<webots::PositionSensor> rightPosSensor;
    std::array<std::array<int, WALL_DIST_SAMPLE_SIZE>, NUM_WALL_DIST_SENSORS> wallDistSamples;
    std::array<bool, NUM_WALL_DIST_SENSORS> wallVisibility;
    std::array<double, NUM_DISTANCE_SENSORS> distReadings;              // Stores current reading values of each built-in distance sensor
    std::array<std::unique_ptr<webots::DistanceSensor>, NUM_DISTANCE_SENSORS>
        distSensors;                                                    // Stores pointers to each built-in distance sensor
    std::string motionPlanFilePath;
    std::string motionExecutionFilePath;
    void Run();                                                         // Run the robot for a single timestep
    void UpdateSensors();
    void Print(std::string message);                                    // General console output function, prepends print prefix
    void PrintPlanState();
    void PrintPlanDetails();
    void PrintIMUReadings();
    void PrintDistanceReadings();
    void PrintWheelPositions();
    bool IsWithinTolerance(double value, double target, double tolerance);
};
