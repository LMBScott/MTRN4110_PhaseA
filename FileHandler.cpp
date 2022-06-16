#include <FileHandler.hpp>

FileHandler::FileHandler()
{
    // Construct the motion plan regex pattern
    std::stringstream pattern;
    pattern << "[0-" << (NUM_MAZE_ROWS - 1) << "][0-" << (NUM_MAZE_COLS - 1) << "][NSEW][FLR]*";

    planRegex = std::regex(pattern.str());
}

std::unique_ptr<MotionPlan> FileHandler::ReadMotionPlan(std::string filePath)
{
    std::ifstream fileStream(filePath, std::ios::in);

    std::string line;

    std::unique_ptr<MotionPlan> plan = nullptr;

    if (fileStream)
    {
        bool lineReadSuccess = (bool)std::getline(fileStream, line);
        fileStream.close();

        if (!lineReadSuccess)
        {
            throw std::runtime_error("FileHandler::ReadMotionPlan: File " + filePath + "does not contain a single line.");
            return plan;
        }
    }
    else
    {
        throw std::runtime_error("FileHandler::ReadMotionPlan: Failed to open instruction file: " + filePath);
        return plan;
    }

    // Ensure that plan matches the regex pattern
    if (std::regex_match(line, planRegex))
    {
        plan = std::make_unique<MotionPlan>();

        plan->line = line;

        // Extract the initial conditions of the instruction set from the line
        std::string initConds = line.substr(0, PLAN_INIT_COND_LENGTH);
        line.erase(0, PLAN_INIT_COND_LENGTH);

        plan->initRow = initConds[0] - '0';
        plan->initColumn = initConds[1] - '0';
        plan->initHeading = static_cast<Direction>(initConds[2]);

        for (char& c : line)
        {
            plan->steps.push_back(static_cast<Movement>(c));
        }
    }
    else
    {
        throw std::runtime_error("FileHandler::ReadMotionPlan: Instructions were not in correct format.");
    }

    return plan;
}

void FileHandler::WriteExecutionHeader(std::string filePath)
{
    std::ofstream fout {filePath, std::ios::out};  // Open file in append mode

    if (fout.is_open()) {
        fout << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall" << std::endl;
        fout.close();
    } else {
        // Throw an error message if file failed to open
        throw std::runtime_error("Writing header line to " + filePath + " failed!");
    }
}

void FileHandler::WritePlanState(std::string filePath, int step, int row, int column, Direction heading, std::array<bool, 3> wallVisibility)
{
    std::ofstream fout {filePath, std::ios::out | std::ios::app};  // Open file in append mode

    if (fout.is_open()) {
        fout << step << ','
             << row << ','
             << column << ','
             << (char)heading << ','
             << (wallVisibility[0] ? 'Y' : 'N') << ','
             << (wallVisibility[1] ? 'Y' : 'N') << ','
             << (wallVisibility[2] ? 'Y' : 'N') << ',' << std::endl;
        fout.close();
    } else {
        // Throw an error message if file failed to open
        throw std::runtime_error("Writing new line to " + filePath + " failed!");
    }
}
