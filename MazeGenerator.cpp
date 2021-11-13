/**
 * @file MazeGenerator.cpp
 * @author Ali Emre Keskin (aliemrekskn@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <random>
#include "MazeGenerator.hpp"

namespace aisa
{
    MazeGenerator::MazeGenerator(cv::Size size)
    {
        size_ = size;
        mat_ = cv::Mat(size_ * 2 + cv::Size(1, 1), CV_8UC1, cv::Scalar(0));
        unvisitedNeighbours_.reserve(3);
        show_ = true;
    }

    aisa::Maze MazeGenerator::RandomizedDepthFirstSearch(cv::Point root, std::string implementation, const double &revisit)
    {
        root = root * 2 + cv::Point(1, 1);
        if (implementation == "recursive")
        {
            if (revisit > 0.0)
            {
                std::cerr << "Rdfs recursive with revisit is not implemented." << std::endl;
            }
            else
            {
                RandomizedDepthFirstSearchRecursiveImplementation(root);
            }
        }
        else if (implementation == "iterative")
        {
            if (revisit > 0.0)
            {
                RandomizedDepthFirstSearchIterativeImplementationWithRevisit(root, revisit);
            }
            else
            {
                RandomizedDepthFirstSearchIterativeImplementation(root);
            }
        }
        return aisa::Maze(mat_);
    }

    aisa::Maze MazeGenerator::RandomizedDepthFirstSearchMultiRoot(std::vector<cv::Point> roots, std::string implementation)
    {
        for (auto &&root : roots)
        {
            root = root * 2 + cv::Point(1, 1);
        }
        if (implementation == "recursive")
        {
            RandomizedDepthFirstSearchRecursiveImplementationMultiRoot(roots);
        }
        else if (implementation == "iterative")
        {
            RandomizedDepthFirstSearchIterativeImplementationMultiRoot(roots);
        }
        return aisa::Maze(mat_);
    }

    void MazeGenerator::RandomizedDepthFirstSearchRecursiveImplementation(cv::Point root)
    {
        cv::Point localCurrent = root;
        current_ = localCurrent;
        MarkVisited(current_);

        UpdateUnvisitedNeighbours(unvisitedNeighbours_, current_, mat_);
        while (!unvisitedNeighbours_.empty())
        {
            selectedNeighbour_ = RandomUnvisitedNeighbour(unvisitedNeighbours_);
            RemoveWall(mat_, current_, selectedNeighbour_);
            RandomizedDepthFirstSearchRecursiveImplementation(selectedNeighbour_);
            current_ = localCurrent;
            UpdateUnvisitedNeighbours(unvisitedNeighbours_, current_, mat_);
        }
    }

    void MazeGenerator::RandomizedDepthFirstSearchRecursiveImplementationMultiRoot(std::vector<cv::Point> roots)
    {
        std::vector<cv::Point> localCurrentVec = roots;
        currentVec_ = localCurrentVec;
        unvisitedNeighboursVec_.clear();
        availableCurrentVec_.clear();
        for (size_t i = 0; i < currentVec_.size(); i++)
        {
            MarkVisited(currentVec_[i]);
            UpdateUnvisitedNeighbours(unvisitedNeighbours_, currentVec_[i], mat_);
            if (!unvisitedNeighbours_.empty())
            {
                unvisitedNeighboursVec_.push_back(unvisitedNeighbours_);
                availableCurrentVec_.push_back(currentVec_[i]);
            }
        }
        while (!unvisitedNeighboursVec_.empty())
        {
            selectedNeighbourVec_.clear();
            for (size_t i = 0; i < unvisitedNeighboursVec_.size(); i++)
            {
                selectedNeighbourVec_.push_back(RandomUnvisitedNeighbour(unvisitedNeighboursVec_[i]));
                RemoveWall(mat_, availableCurrentVec_[i], selectedNeighbourVec_.back());
            }
            RandomizedDepthFirstSearchRecursiveImplementationMultiRoot(selectedNeighbourVec_);
            currentVec_ = localCurrentVec;
            unvisitedNeighboursVec_.clear();
            availableCurrentVec_.clear();
            for (size_t i = 0; i < currentVec_.size(); i++)
            {
                UpdateUnvisitedNeighbours(unvisitedNeighbours_, currentVec_[i], mat_);
                if (!unvisitedNeighbours_.empty())
                {
                    unvisitedNeighboursVec_.push_back(unvisitedNeighbours_);
                    availableCurrentVec_.push_back(currentVec_[i]);
                }
            }
        }
    }

    void MazeGenerator::RandomizedDepthFirstSearchIterativeImplementation(cv::Point root)
    {
        MarkVisited(mat_, root);
        stack_.push(root);
        while (!stack_.empty())
        {
            current_ = stack_.top();
            stack_.pop();
            UpdateUnvisitedNeighbours(unvisitedNeighbours_, current_, mat_);
            if (!unvisitedNeighbours_.empty())
            {
                stack_.push(current_);
                selectedNeighbour_ = RandomUnvisitedNeighbour(unvisitedNeighbours_);
                RemoveWall(mat_, current_, selectedNeighbour_);
                MarkVisited(mat_, selectedNeighbour_);
                stack_.push(selectedNeighbour_);
            }
        }
    }

    void MazeGenerator::RandomizedDepthFirstSearchIterativeImplementationWithRevisit(cv::Point root, const double &revisit)
    {
        MarkVisited(mat_, root);
        stack_.push(root);
        while (!stack_.empty())
        {
            current_ = stack_.top();
            stack_.pop();
            UpdateUnvisitedNeighboursWithRevisit(unvisitedNeighbours_, current_, mat_, revisit);
            if (!unvisitedNeighbours_.empty())
            {
                stack_.push(current_);
                selectedNeighbour_ = RandomUnvisitedNeighbour(unvisitedNeighbours_);
                RemoveWall(mat_, current_, selectedNeighbour_);
                MarkVisited(mat_, selectedNeighbour_);
                stack_.push(selectedNeighbour_);
            }
        }
    }

    void MazeGenerator::RandomizedDepthFirstSearchIterativeImplementationMultiRoot(std::vector<cv::Point> roots)
    {
        stackVec_.resize(roots.size());
        currentVec_.resize(roots.size());
        unvisitedNeighboursVec_.resize(roots.size());
        selectedNeighbourVec_.resize(roots.size());
        for (size_t i = 0; i < roots.size(); i++)
        {
            MarkVisited(mat_, roots[i]);
            stackVec_[i].push(roots[i]);
        }
        while (!AllStacksAreEmpty(stackVec_))
        {
            for (size_t i = 0; i < stackVec_.size(); i++)
            {
                if (!stackVec_[i].empty())
                {
                    currentVec_[i] = stackVec_[i].top();
                    stackVec_[i].pop();
                    UpdateUnvisitedNeighbours(unvisitedNeighboursVec_[i], currentVec_[i], mat_);
                    if (!unvisitedNeighboursVec_[i].empty())
                    {
                        stackVec_[i].push(currentVec_[i]);
                        selectedNeighbourVec_[i] = RandomUnvisitedNeighbour(unvisitedNeighboursVec_[i]);
                        RemoveWall(mat_, currentVec_[i], selectedNeighbourVec_[i]);
                        MarkVisited(mat_, selectedNeighbourVec_[i]);
                        stackVec_[i].push(selectedNeighbourVec_[i]);
                    }
                }
            }
        }
    }

    bool MazeGenerator::IsNotVisited(const cv::Point &point, const cv::Mat &mat)
    {
        return mat.at<uchar>(point) == 0;
    }

    bool MazeGenerator::IsNotVisitedWithRevisit(const cv::Point &point, const cv::Mat &mat, const double &revisit)
    {
        return mat.at<uchar>(point) == 0 || Probably(revisit);
    }

    bool MazeGenerator::LeftAvailable(const cv::Point &current, const cv::Mat &mat)
    {
        return HasLeft(current) && IsNotVisited(Left(current), mat);
    }

    bool MazeGenerator::RightAvailable(const cv::Point &current, const cv::Mat &mat)
    {
        return HasRight(current, mat) && IsNotVisited(Right(current), mat);
    }

    bool MazeGenerator::TopAvailable(const cv::Point &current, const cv::Mat &mat)
    {
        return HasTop(current) && IsNotVisited(Top(current), mat);
    }

    bool MazeGenerator::BottomAvailable(const cv::Point &current, const cv::Mat &mat)
    {
        return HasBottom(current, mat) && IsNotVisited(Bottom(current), mat);
    }

    bool MazeGenerator::LeftAvailableWithRevisit(const cv::Point &current, const cv::Mat &mat, const double &revisit)
    {
        return HasLeft(current) && IsNotVisitedWithRevisit(Left(current), mat, revisit);
    }

    bool MazeGenerator::RightAvailableWithRevisit(const cv::Point &current, const cv::Mat &mat, const double &revisit)
    {
        return HasRight(current, mat) && IsNotVisitedWithRevisit(Right(current), mat, revisit);
    }

    bool MazeGenerator::TopAvailableWithRevisit(const cv::Point &current, const cv::Mat &mat, const double &revisit)
    {
        return HasTop(current) && IsNotVisitedWithRevisit(Top(current), mat, revisit);
    }

    bool MazeGenerator::BottomAvailableWithRevisit(const cv::Point &current, const cv::Mat &mat, const double &revisit)
    {
        return HasBottom(current, mat) && IsNotVisitedWithRevisit(Bottom(current), mat, revisit);
    }

    bool MazeGenerator::HasLeft(const cv::Point &current)
    {
        return current.x > 2;
    }

    bool MazeGenerator::HasRight(const cv::Point &current, const cv::Mat &mat)
    {
        return current.x < (mat.cols - 3);
    }

    bool MazeGenerator::HasTop(const cv::Point &current)
    {
        return current.y > 2;
    }

    bool MazeGenerator::HasBottom(const cv::Point &current, const cv::Mat &mat)
    {
        return current.y < (mat.rows - 3);
    }

    cv::Point MazeGenerator::Left(const cv::Point &current)
    {
        return current + cv::Point(-2, 0);
    }

    cv::Point MazeGenerator::Right(const cv::Point &current)
    {
        return current + cv::Point(+2, 0);
    }

    cv::Point MazeGenerator::Top(const cv::Point &current)
    {
        return current + cv::Point(0, -2);
    }

    cv::Point MazeGenerator::Bottom(const cv::Point &current)
    {
        return current + cv::Point(0, +2);
    }

    void MazeGenerator::UpdateUnvisitedNeighbours(std::vector<cv::Point> &unvisitedNeighbours, const cv::Point &current, const cv::Mat &mat)
    {
        unvisitedNeighbours.clear();
        if (LeftAvailable(current, mat))
        {
            unvisitedNeighbours.emplace_back(Left(current));
        }
        if (RightAvailable(current, mat))
        {
            unvisitedNeighbours.emplace_back(Right(current));
        }
        if (TopAvailable(current, mat))
        {
            unvisitedNeighbours.emplace_back(Top(current));
        }
        if (BottomAvailable(current, mat))
        {
            unvisitedNeighbours.emplace_back(Bottom(current));
        }
    }

    void MazeGenerator::UpdateUnvisitedNeighboursWithRevisit(std::vector<cv::Point> &unvisitedNeighbours, const cv::Point &current, const cv::Mat &mat, const double &revisit)
    {
        unvisitedNeighbours.clear();
        if (LeftAvailableWithRevisit(current, mat, revisit))
        {
            unvisitedNeighbours.emplace_back(Left(current));
        }
        if (RightAvailableWithRevisit(current, mat, revisit))
        {
            unvisitedNeighbours.emplace_back(Right(current));
        }
        if (TopAvailableWithRevisit(current, mat, revisit))
        {
            unvisitedNeighbours.emplace_back(Top(current));
        }
        if (BottomAvailableWithRevisit(current, mat, revisit))
        {
            unvisitedNeighbours.emplace_back(Bottom(current));
        }
    }

    cv::Point MazeGenerator::RandomUnvisitedNeighbour(const std::vector<cv::Point> &unvisitedNeighbours)
    {
        return unvisitedNeighbours[std::rand() % unvisitedNeighbours.size()];
    }

    void MazeGenerator::RemoveWall(cv::Mat &mat, const cv::Point &current, const cv::Point &selectedNeighbour)
    {
        mat.at<uchar>((current + selectedNeighbour) / 2) = 255;
    }

    void MazeGenerator::MarkVisited(cv::Mat &mat, const cv::Point &current)
    {
        mat.at<uchar>(current) = 255;
        if (show_)
        {
            cv::imshow("aisa", mat_);
            auto key = cv::waitKey(1);
            if (key == 27)
            {
                exit(EXIT_FAILURE);
            }
        }
    }

    bool MazeGenerator::AllStacksAreEmpty(std::vector<std::stack<cv::Point>> stackVec)
    {
        for (auto &&stack : stackVec)
        {
            if (!stack.empty())
            {
                return false;
            }
        }
        return true;
    }

    cv::Mat MazeGenerator::GetMat()
    {
        return mat_;
    }

    bool MazeGenerator::Probably(const double &revisit)
    {
        // std::knuth_b rand_engine;
        // std::bernoulli_distribution distribution(revisit);
        // return distribution(rand_engine);

        // std::knuth_b rand_engine; // replace knuth_b with one of the engines listed below
        // std::uniform_real_distribution<> uniform_zero_to_one(0.0, 1.0);
        // return uniform_zero_to_one(rand_engine) >= revisit;

        return std::rand() / (RAND_MAX + 1.0) < revisit;
    }
}