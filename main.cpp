#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include "Maze.hpp"
#include "MazeGenerator.hpp"
#include "MazeSolver.hpp"

int main(int, char **)
{
    cv::Size size(20, 20);
    cv::Point initial(0, 0);
    cv::Point goal(size.width - 1, size.height - 1);

    auto mazeGenerator = aisa::MazeGenerator(size);
    auto maze = mazeGenerator.RandomizedDepthFirstSearch({0, 0}, "iterative", 0.01);
    maze.Show(true);
    cv::imwrite("aisa.png", maze.Mat());

    auto mazeSolver = aisa::MazeSolver();
    std::vector<cv::Point> solution;

    auto start = std::chrono::high_resolution_clock::now();

    // mazeSolver.DfsIterative(maze, initial, goal, solution);
    // mazeSolver.DfsIterativeWıthPath(maze, initial, goal, solution);
    // auto result = mazeSolver.Dls(maze, initial, goal, 10000000, aisa::MazeSolver::Implementation::recursive);
    auto result = mazeSolver.IterativeDeepeningSearch(maze, initial, goal, solution, aisa::MazeSolver::Implementation::recursive);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;

    maze.Trace(solution);
    std::cout << "Length of path: " << maze.LengthOfPath() << std::endl;
    std::cout << "Number of nodes expanded: " << maze.NumberOfExpanded() << std::endl;
    cv::imwrite("solution.png", maze.Mat());
}