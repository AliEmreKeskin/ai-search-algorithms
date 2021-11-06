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

#include "MazeGenerator.hpp"

namespace aisa
{
    MazeGenerator::MazeGenerator(cv::Size size)
    {
        size_ = size;
        mat_ = cv::Mat(size_ * 2 + cv::Size(1, 1), CV_8UC1, cv::Scalar(0));
    }

    aisa::Maze MazeGenerator::RandomizedDepthFirstSearch(cv::Point root, std::string implementation)
    {
        root = root * 2 + cv::Point(1, 1);
        if (implementation == "recursive")
        {
            RandomizedDepthFirstSearchRecursiveImplementation(root);
            return aisa::Maze(mat_);
        }
    }

    void MazeGenerator::RandomizedDepthFirstSearchRecursiveImplementation(cv::Point root)
    {
        cv::Point localCurrent = root;
        current_ = localCurrent;
        mat_.at<uchar>(current_) = 255;
        cv::imshow("aisa",mat_);
        cv::waitKey(1);
        UpdateUnvisitedNeighbours();
        while (!unvisitedNeighbours_.empty())
        {
            selectedNeighbour_ = RandomUnvisitedNeighbour();
            RemoveWall();
            RandomizedDepthFirstSearchRecursiveImplementation(selectedNeighbour_);
            current_ = localCurrent;
            UpdateUnvisitedNeighbours();
        }
    }

    bool MazeGenerator::LeftAvailable()
    {
        if (current_.x > 2 && mat_.at<uchar>(cv::Point(current_.x - 2, current_.y)) == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool MazeGenerator::RightAvailable()
    {
        if (current_.x < (mat_.cols - 3) && mat_.at<uchar>(cv::Point(current_.x + 2, current_.y)) == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool MazeGenerator::TopAvailable()
    {
        if (current_.y > 2 && mat_.at<uchar>(cv::Point(current_.x, current_.y - 2)) == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool MazeGenerator::BottomAvailable()
    {
        if (current_.y < (mat_.rows - 3) && mat_.at<uchar>(cv::Point(current_.x, current_.y + 2)) == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    cv::Point MazeGenerator::Left()
    {
        return current_ + cv::Point(-2, 0);
    }

    cv::Point MazeGenerator::Right()
    {
        return current_ + cv::Point(+2, 0);
    }

    cv::Point MazeGenerator::Top()
    {
        return current_ + cv::Point(0, -2);
    }

    cv::Point MazeGenerator::Bottom()
    {
        return current_ + cv::Point(0, +2);
    }

    void MazeGenerator::UpdateUnvisitedNeighbours()
    {
        unvisitedNeighbours_.clear();

        if (LeftAvailable())
        {
            unvisitedNeighbours_.push_back(Left());
        }
        if (RightAvailable())
        {
            unvisitedNeighbours_.push_back(Right());
        }
        if (TopAvailable())
        {
            unvisitedNeighbours_.push_back(Top());
        }
        if (BottomAvailable())
        {
            unvisitedNeighbours_.push_back(Bottom());
        }
    }

    cv::Point MazeGenerator::RandomUnvisitedNeighbour()
    {
        return unvisitedNeighbours_[std::rand() % unvisitedNeighbours_.size()];
    }

    void MazeGenerator::RemoveWall()
    {
        mat_.at<uchar>((current_ + selectedNeighbour_) / 2) = 255;
    }

    cv::Mat MazeGenerator::GetMat()
    {
        return mat_;
    }
}