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
        enum class Type{

        };
        Maze(cv::InputOutputArray mat);
        ~Maze()=default;
        cv::Mat GetMat();

    private:
        cv::Mat mat_;
    };
} // namespace aisa

#endif // __MAZE_H__