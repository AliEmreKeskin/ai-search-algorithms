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
#include <stack>
#include <vector>
#include "Maze.hpp"

namespace aisa
{
    class MazeGenerator
    {
    public:
        MazeGenerator(cv::Size size);
        aisa::Maze RandomizedDepthFirstSearch(cv::Point root, std::string implementation = "recursive");
        aisa::Maze RandomizedDepthFirstSearchMultiRoot(std::vector<cv::Point> roots, std::string implementation = "recursive");

    private:
        void RandomizedDepthFirstSearchRecursiveImplementation(cv::Point root);
        void RandomizedDepthFirstSearchRecursiveImplementationMultiRoot(std::vector<cv::Point> roots);
        void RandomizedDepthFirstSearchIterativeImplementation(cv::Point root);
        void RandomizedDepthFirstSearchIterativeImplementationMultiRoot(std::vector<cv::Point> roots);
        bool LeftAvailable();
        bool LeftAvailable(const cv::Point &current, const cv::Mat &mat);
        bool RightAvailable();
        bool RightAvailable(const cv::Point &current, const cv::Mat &mat);
        bool TopAvailable();
        bool TopAvailable(const cv::Point &current, const cv::Mat &mat);
        bool BottomAvailable();
        bool BottomAvailable(const cv::Point &current, const cv::Mat &mat);
        cv::Point Left();
        cv::Point Left(const cv::Point &current);
        cv::Point Right();
        cv::Point Right(const cv::Point &current);
        cv::Point Top();
        cv::Point Top(const cv::Point &current);
        cv::Point Bottom();
        cv::Point Bottom(const cv::Point &current);
        void UpdateUnvisitedNeighbours();
        void UpdateUnvisitedNeighbours(std::vector<cv::Point> &unvisitedNeighbours, const cv::Point &current, const cv::Mat &mat);
        cv::Point RandomUnvisitedNeighbour();
        cv::Point RandomUnvisitedNeighbour(const std::vector<cv::Point> &unvisitedNeighbours);
        void RemoveWall();
        void RemoveWall(cv::Mat &mat, const cv::Point &current, const cv::Point &selectedNeighbour);
        void MarkVisited(cv::Mat &mat, const cv::Point &current);
        bool AllStacksAreEmpty(std::vector<std::stack<cv::Point>> stackVec);
        cv::Mat GetMat();

        cv::Size size_;
        cv::Mat mat_;

        cv::Point current_;
        std::vector<cv::Point> unvisitedNeighbours_;
        cv::Point selectedNeighbour_;

        std::vector<cv::Point> currentVec_;
        std::vector<cv::Point> availableCurrentVec_;
        std::vector<std::vector<cv::Point>> unvisitedNeighboursVec_;
        std::vector<cv::Point> selectedNeighbourVec_;

        std::stack<cv::Point> stack_;
        std::vector<std::stack<cv::Point>> stackVec_;

        bool show_;
    };
} // namespace aisa
