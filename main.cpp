#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "Maze.hpp"
#include "MazeGenerator.hpp"

int main(int, char **)
{
    auto mazeGenerator = aisa::MazeGenerator({100, 100});
    auto maze = mazeGenerator.RandomizedDepthFirstSearch({0, 0}, "iterative", 0.001);
    cv::imwrite("aisa.png", maze.GetMat());
    cv::waitKey();
}