#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "Maze.hpp"
#include "MazeGenerator.hpp"

int main(int, char **)
{
    auto mazeGenerator = aisa::MazeGenerator({100, 100});
    auto maze = mazeGenerator.RandomizedDepthFirstSearchMultiRoot({{0, 0}, {99, 99}}, "iterative");
    cv::imwrite("aisa.png", maze.GetMat());
    cv::waitKey();
}