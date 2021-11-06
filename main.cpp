#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "Maze.hpp"
#include "MazeGenerator.hpp"

int main(int, char **)
{
    auto mazeGenerator = aisa::MazeGenerator({500,500});
    auto maze = mazeGenerator.RandomizedDepthFirstSearch({0,0});
    cv::imshow("aisa", maze.GetMat());
    cv::waitKey();
}