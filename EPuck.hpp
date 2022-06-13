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
    static constexpr int NUM_DISTANCE_SENSORS {11};                     // Number of distance sensors on the e-puck robot
    static constexpr int NUM_WALL_DIST_SENSORS {3};
    static constexpr int WALL_DIST_SAMPLE_SIZE {10};
    static constexpr int LEFT_DIST_SENSOR_INDEX {7};
    static constexpr int FRONT_DIST_SENSOR_INDEX {0};
    static constexpr int RIGHT_DIST_SENSOR_INDEX {3};
    static constexpr double MAX_MOTOR_SPEED {6.28};                     // Max motor speed of the e-puck robot, in rads/s
    static constexpr double TURN_SPEED {MAX_MOTOR_SPEED / 4};           // Speed of motors during a turn, in rads/s
    static constexpr double FORWARD_SPEED {MAX_MOTOR_SPEED / 2};        // Speed of motors moving forward, in rads/s
    static constexpr double AXLE_LENGTH {0.04486};                      // Axle length of the e-puck robot, in meters
    static constexpr double WHEEL_RADIUS {0.02};                      // Wheel radius of the e-puck robot, in meters
    static constexpr double INTER_CELL_DIST {0.165};                    // Distance between maze cells, in meters
    static constexpr double WALL_THICKNESS {0.015};                     // Thickness of maze walls, in meters
    static constexpr double WALL_DETECTION_THRESHOLD {750.0};           // Minimum proximity sensor reading indicating wall presence
    static constexpr double POSITION_TOLERANCE {0.02};
    static constexpr double TURN_ANGLE_TOLERANCE {0.04};                // Tolerance of turn angles, in radians
    static const std::array<std::string, NUM_DISTANCE_SENSORS> distSensorNames;
    static const std::string PRINT_PREFIX;
    static const std::map<Direction, std::array<int, 2>>
        MOVEMENT_DELTAS;                                                // Changes in row & column from movement along each heading
    static const std::map<Direction, Direction> LEFT_TURN_MAP;          // Resultant heading from a left turn starting at each heading
    static const std::map<Direction, Direction> RIGHT_TURN_MAP;         // Resultant heading from a right turn starting at each heading
    static const std::map<Direction, double> HEADING_YAWS;              // Expected yaw angle reading from IMU for each heading
    int planStep;
    int row;
    int column;
    int numWallDistSamples;
    double simTime;                                                     // Stores current simulation time
    double turnDuration;
    double forwardDuration;
    double turnPosDelta;
    double forwardPosDelta;
    double stepEndTime;                                                 // The simulation time at which the current step will be complete
    double leftSetSpeed;
    double rightSetSpeed;
    double leftWheelPos;
    double rightWheelPos;
    double leftWheelSetPos;
    double rightWheelSetPos;
    double roll;
    double pitch;
    double yaw;
    bool isStepComplete;
    bool hasSampledWallDistance;
    bool isPlanComplete;
    FileHandler fileHandler;
    Direction heading;
    std::unique_ptr<MotionPlan> plan;
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
    void Print(std::string message);
    void PrintPlanState();
    void PrintPlanDetails();
    void PrintIMUReadings();
    void PrintDistanceReadings();
    void PrintWheelPositions();
    bool IsWithinTolerance(double value, double target, double tolerance);
};
