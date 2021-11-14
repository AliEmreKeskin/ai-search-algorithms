/**
 * @file Maze.cpp
 * @author Ali Emre Keskin (aliemrekskn@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <opencv2/opencv.hpp>
#include "Maze.hpp"

namespace aisa
{
    Maze::Maze(cv::InputOutputArray mat) : mat_(mat.getMat())
    {
        cv::namedWindow("maze", cv::WindowFlags::WINDOW_NORMAL);
    }

    cv::Mat &Maze::Mat()
    {
        return mat_;
    }

    bool Maze::LeftIsWall(const cv::Point &current)
    {
        return IsWall(LeftWall(current));
    }

    bool Maze::RightIsWall(const cv::Point &current)
    {
        return IsWall(RightWall(current));
    }

    bool Maze::TopIsWall(const cv::Point &current)
    {
        return IsWall(TopWall(current));
    }

    bool Maze::BottomIsWall(const cv::Point &current)
    {
        return IsWall(BottomWall(current));
    }

    cv::Point Maze::Left(const cv::Point &current)
    {
        return current + cv::Point(-2, 0);
    }

    cv::Point Maze::Right(const cv::Point &current)
    {
        return current + cv::Point(+2, 0);
    }

    cv::Point Maze::Top(const cv::Point &current)
    {
        return current + cv::Point(0, -2);
    }

    cv::Point Maze::Bottom(const cv::Point &current)
    {
        return current + cv::Point(0, +2);
    }

    void Maze::Trace(std::vector<cv::Point> trace)
    {
        for (size_t i = 0; i < trace.size() - 1; i++)
        {
            cv::line(mat_, trace[i], trace[i + 1], cv::Scalar(trace_));
        }
    }

    void Maze::MarkDiscovered(cv::Point point)
    {
        mat_.at<uchar>(point) = discovered_;
        if (show_)
        {
            cv::imshow("maze", mat_);
            auto key = cv::waitKey(1);
            if (key == 27)
            {
                exit(EXIT_FAILURE);
            }
        }
    }

    std::vector<cv::Point> Maze::Ways(cv::Point point)
    {
        std::vector<cv::Point> res;
        if (!LeftIsWall(point))
        {
            res.push_back(Left(point));
        }
        if (!RightIsWall(point))
        {
            res.push_back(Right(point));
        }
        if (!TopIsWall(point))
        {
            res.push_back(Top(point));
        }
        if (!BottomIsWall(point))
        {
            res.push_back(Bottom(point));
        }
        return res;
    }

    void Maze::Show(bool enable)
    {
        show_ = enable;
    }

    bool Maze::IsNotDiscovered(const cv::Point &point)
    {
        return mat_.at<uchar>(point) != discovered_;
    }

    aisa::Maze Maze::Clone()
    {
        return aisa::Maze(mat_.clone());
    }

    size_t Maze::LengthOfPath()
    {
        size_t res = 0;
        for (size_t i = 1; i < mat_.rows; i += 2)
        {
            for (size_t j = 1; j < mat_.cols; j += 2)
            {
                if (mat_.at<uchar>(j, i) == trace_)
                {
                    res++;
                }
            }
        }
        return res;
    }

    size_t Maze::NumberOfExpanded()
    {
        size_t res = 0;
        for (size_t i = 1; i < mat_.rows; i += 2)
        {
            for (size_t j = 1; j < mat_.cols; j += 2)
            {
                if (mat_.at<uchar>(j, i) == discovered_)
                {
                    res++;
                }
            }
        }
        return res + LengthOfPath();
    }

    bool Maze::IsWall(const cv::Point &point)
    {
        return mat_.at<uchar>(point) == wall_;
    }

    cv::Point Maze::LeftWall(const cv::Point &current)
    {
        return current + cv::Point(-1, 0);
    }

    cv::Point Maze::RightWall(const cv::Point &current)
    {
        return current + cv::Point(+1, 0);
    }

    cv::Point Maze::TopWall(const cv::Point &current)
    {
        return current + cv::Point(0, -1);
    }

    cv::Point Maze::BottomWall(const cv::Point &current)
    {
        return current + cv::Point(0, +1);
    }
}