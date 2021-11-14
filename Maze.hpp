#ifndef __MAZE_H__
#define __MAZE_H__

/**
 * @file Maze.hpp
 * @author Ali Emre Keskin (aliemrekskn@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <opencv2/opencv.hpp>

namespace aisa
{
    class Maze
    {
    public:
        enum class Type
        {

        };
        Maze(cv::InputOutputArray mat);
        ~Maze() = default;
        cv::Mat &Mat();
        bool LeftIsWall(const cv::Point &current);
        bool RightIsWall(const cv::Point &current);
        bool TopIsWall(const cv::Point &current);
        bool BottomIsWall(const cv::Point &current);
        cv::Point Left(const cv::Point &current);
        cv::Point Right(const cv::Point &current);
        cv::Point Top(const cv::Point &current);
        cv::Point Bottom(const cv::Point &current);
        void Trace(std::vector<cv::Point> trace);
        void MarkDiscovered(cv::Point point);
        std::vector<cv::Point> Ways(cv::Point point);
        void Show(bool enable);
        bool IsNotDiscovered(const cv::Point &point);
        aisa::Maze Clone();
        size_t LengthOfPath();
        size_t NumberOfExpanded();

    private:
        bool IsWall(const cv::Point &point);
        cv::Point LeftWall(const cv::Point &point);
        cv::Point RightWall(const cv::Point &point);
        cv::Point TopWall(const cv::Point &point);
        cv::Point BottomWall(const cv::Point &point);

        uchar wall_ = 0;
        uchar trace_ = 127;
        uchar discovered_ = 191;
        cv::Mat mat_;
        bool show_;
    };
} // namespace aisa

#endif // __MAZE_H__