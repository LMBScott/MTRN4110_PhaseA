#include <array>
#include <FileHandler.hpp>
#include <math.h>
#include <map>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>

class EPuck : public webots::Robot {
public:
    EPuck();
    void Run();                                                         // Run the robot for a single timestep
    int GetTimeStep() const;                                            // Retrieve the basic time step of the simulation
    void ReadMotionPlan();
    void SetUpExecutionFile();
private:
    static constexpr int NUM_DISTANCE_SENSORS {11};                     // Number of distance sensors on the e-puck robot
    static constexpr int YAW_SAMPLE_SIZE {10};
    static constexpr double MAX_MOTOR_SPEED {6.28};                     // Max motor speed of the e-puck robot, in rads/s
    static constexpr double TURN_SPEED {MAX_MOTOR_SPEED / 4};           // Speed of motors during a turn, in rads/s
    static constexpr double FORWARD_SPEED {MAX_MOTOR_SPEED / 2};        // Speed of motors moving forward, in rads/s
    static constexpr double AXLE_LENGTH {0.052};                        // Axle length of the e-puck robot, in meters
    static constexpr double WHEEL_RADIUS {0.0205};                      // Wheel radius of the e-puck robot, in meters
    static constexpr double INTER_CELL_DIST {0.165};                    // Distance between maze cells, in meters
    static constexpr double WALL_THICKNESS {0.015};                     // Thickness of maze walls, in meters
    static constexpr double WALL_DETECTION_THRESHOLD {1000.0};          // Minimum proximity sensor reading indicating wall presence
    static constexpr double TURN_ANGLE_TOLERANCE {0.04};                // Tolerance of turn angles, in radians
    static const std::string PLAN_INPUT_FILE;
    static const std::string EXECUTION_OUTPUT_FILE;
    static const std::array<std::string, NUM_DISTANCE_SENSORS> distSensorNames;
    static const std::string PRINT_PREFIX;
    static const std::map<Direction, std::array<int, 2>>
        MOVEMENT_DELTAS;                                                // Changes in row & column from movement along each heading
    static const std::map<Direction, Direction> LEFT_TURN_MAP;          // Resultant heading from a left turn starting at each heading
    static const std::map<Direction, Direction> RIGHT_TURN_MAP;         // Resultant heading from a right turn starting at each heading
    static const std::map<Direction, double> HEADING_YAWS;              // Expected yaw angle reading from IMU for each heading
    int timeStep;                                                       // Stores simulation time step
    int planStep;
    int row;
    int column;
    int numYawSamples;
    Direction heading;
    std::array<bool, 3> wallVisibility;
    std::array<double, NUM_DISTANCE_SENSORS> distReadings;              // Stores current reading values of each built-in distance sensor
    std::array<std::unique_ptr<webots::DistanceSensor>, NUM_DISTANCE_SENSORS>
        distSensors;                                                    // Stores pointers to each built-in distance sensor
    std::unique_ptr<webots::InertialUnit> IMU;
    std::unique_ptr<webots::Motor> leftMotor;
    std::unique_ptr<webots::Motor> rightMotor;
    double simTime;                                                     // Stores current simulation time
    double turnDuration;
    double forwardDuration;
    double stepEndTime;                                                 // The simulation time at which the current step will be complete
    double leftSetSpeed;
    double rightSetSpeed;
    double roll;
    double pitch;
    double yaw;
    double yawAvg;
    bool isPlanComplete;
    FileHandler fileHandler;
    std::unique_ptr<MotionPlan> plan;
    void Print(std::string message);
    void PrintPlanState();
    void PrintPlanDetails();
    void PrintIMUReadings();
    void PrintDistanceReadings();
};
