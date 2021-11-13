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
        aisa::Maze RandomizedDepthFirstSearch(cv::Point root, std::string implementation = "iterative", const double &revisit = 0.0);
        aisa::Maze RandomizedDepthFirstSearchMultiRoot(std::vector<cv::Point> roots, std::string implementation = "iterative");

    private:
        void RandomizedDepthFirstSearchRecursiveImplementation(cv::Point root);
        void RandomizedDepthFirstSearchRecursiveImplementationMultiRoot(std::vector<cv::Point> roots);
        void RandomizedDepthFirstSearchIterativeImplementation(cv::Point root);
        void RandomizedDepthFirstSearchIterativeImplementationWithRevisit(cv::Point root, const double &revisit);
        void RandomizedDepthFirstSearchIterativeImplementationMultiRoot(std::vector<cv::Point> roots);
        bool IsNotVisited(const cv::Point &point, const cv::Mat &mat);
        bool IsNotVisitedWithRevisit(const cv::Point &point, const cv::Mat &mat, const double &revisit);
        bool LeftAvailable(const cv::Point &current, const cv::Mat &mat);
        bool RightAvailable(const cv::Point &current, const cv::Mat &mat);
        bool TopAvailable(const cv::Point &current, const cv::Mat &mat);
        bool BottomAvailable(const cv::Point &current, const cv::Mat &mat);
        bool LeftAvailableWithRevisit(const cv::Point &current, const cv::Mat &mat, const double &revisit);
        bool RightAvailableWithRevisit(const cv::Point &current, const cv::Mat &mat, const double &revisit);
        bool TopAvailableWithRevisit(const cv::Point &current, const cv::Mat &mat, const double &revisit);
        bool BottomAvailableWithRevisit(const cv::Point &current, const cv::Mat &mat, const double &revisit);
        bool HasLeft(const cv::Point &current);
        bool HasRight(const cv::Point &current, const cv::Mat &mat);
        bool HasTop(const cv::Point &current);
        bool HasBottom(const cv::Point &current, const cv::Mat &mat);
        cv::Point Left(const cv::Point &current);
        cv::Point Right(const cv::Point &current);
        cv::Point Top(const cv::Point &current);
        cv::Point Bottom(const cv::Point &current);
        void UpdateUnvisitedNeighbours(std::vector<cv::Point> &unvisitedNeighbours, const cv::Point &current, const cv::Mat &mat);
        void UpdateUnvisitedNeighboursWithRevisit(std::vector<cv::Point> &unvisitedNeighbours, const cv::Point &current, const cv::Mat &mat, const double &revisit);
        cv::Point RandomUnvisitedNeighbour(const std::vector<cv::Point> &unvisitedNeighbours);
        void RemoveWall(cv::Mat &mat, const cv::Point &current, const cv::Point &selectedNeighbour);
        void MarkVisited(cv::Mat &mat, const cv::Point &current);
        bool AllStacksAreEmpty(std::vector<std::stack<cv::Point>> stackVec);
        cv::Mat GetMat();
        bool Probably(const double &revisit);

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
