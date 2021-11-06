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

#include "Maze.hpp"

namespace aisa
{
    Maze::Maze(cv::InputOutputArray mat) : mat_(mat.getMat())
    {
    }

    cv::Mat Maze::GetMat()
    {
        return mat_;
    }
}