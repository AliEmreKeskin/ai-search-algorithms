#ifndef __MAZESOLVER_H__
#define __MAZESOLVER_H__

/**
 * @file MazeSolver.hpp
 * @author Ali Emre Keskin (aliemrekskn@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <opencv2/opencv.hpp>
#include "Maze.hpp"

namespace aisa
{
    class MazeSolver
    {
    public:
        enum class DlsResult
        {
            solution = 1,
            failure = 2,
            cutoff = 3
        };
        enum class Implementation
        {
            recursive = 1,
            iterative = 2
        };
        enum class Heuristic
        {
            EuclideanDistance = 1,
            ManhattanDistance = 2
        };
        MazeSolver();
        ~MazeSolver();
        bool DfsIterativeWithPath(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution);
        bool DfsIterative(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution);
        MazeSolver::DlsResult Dls(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution, size_t limit, Implementation implementation);
        bool IterativeDeepeningSearch(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution, Implementation implementation);
        bool UniformCostSearch(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution);
        bool AStar(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution, Heuristic heuristic);

        double EuclideanDistance(cv::Point current, cv::Point goal);
        double ManhattanDistance(cv::Point current, cv::Point goal);
    private:
        MazeSolver::DlsResult DlsRecursive(aisa::Maze &maze, cv::Point initial, cv::Point goal, size_t limit);
        uchar discovered_ = 191;
        bool show_;
        std::vector<cv::Point> solution_;
    };

} // namespace aisa

#endif // __MAZESOLVER_H__