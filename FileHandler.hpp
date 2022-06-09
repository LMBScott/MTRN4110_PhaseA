#include <memory>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <regex>
#include <array>

constexpr int NUM_MAZE_COLS {9};                // Number of grid columns in the maze
constexpr int NUM_MAZE_ROWS {5};                // Number of grid rows in the maze

enum class Direction : char
{
    North = 'N',
    South = 'S',
    East = 'E',
    West = 'W'
};

enum class Movement : char
{
    Forward = 'F',
    Left = 'L',
    Right = 'R'
};

struct MotionPlan
{
    std::string line;
    int initRow;
    int initColumn;
    Direction initHeading;
    std::vector<Movement> steps;
};

class FileHandler {
public:
    FileHandler();
    std::unique_ptr<MotionPlan> ReadMotionPlan(std::string filePath);
    void WriteExecutionHeader(std::string filePath);
    void WritePlanState(std::string filePath, int step, int row, int column, Direction heading, std::array<bool, 3> wallVisibility);
private:
    static constexpr int PLAN_INIT_COND_LENGTH {3};
    std::regex planRegex;
};
