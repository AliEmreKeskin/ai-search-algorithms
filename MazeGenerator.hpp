/**
 * @file MazeGenerator.hpp
 * @author Ali Emre Keskin (aliemrekskn@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <stddef.h>
#include "Maze.hpp"

namespace aisa
{
    class MazeGenerator
    {
    public:
        MazeGenerator(cv::Size size);
        aisa::Maze RandomizedDepthFirstSearch(cv::Point root, std::string implementation = "recursive");

    private:
        void RandomizedDepthFirstSearchRecursiveImplementation(cv::Point root);
        bool LeftAvailable();
        bool RightAvailable();
        bool TopAvailable();
        bool BottomAvailable();
        cv::Point Left();
        cv::Point Right();
        cv::Point Top();
        cv::Point Bottom();
        void UpdateUnvisitedNeighbours();
        cv::Point RandomUnvisitedNeighbour();
        void RemoveWall();
        cv::Mat GetMat();

        cv::Size size_;
        cv::Mat mat_;
        cv::Point current_;
        std::vector<cv::Point> unvisitedNeighbours_;
        cv::Point selectedNeighbour_;
    };
} // namespace aisa
