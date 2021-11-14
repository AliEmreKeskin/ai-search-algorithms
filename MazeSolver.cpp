#include "MazeSolver.hpp"
#include <stack>
#include <vector>
#include <opencv2/opencv.hpp>
#include <deque>

#include "Tree.hpp"

namespace aisa
{
    MazeSolver::MazeSolver()
    {
    }

    MazeSolver::~MazeSolver()
    {
    }

    bool MazeSolver::DfsIterativeWithPath(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution)
    {
        initial = initial * 2 + cv::Point(1, 1);
        goal = goal * 2 + cv::Point(1, 1);
        std::stack<std::vector<cv::Point>> stack;
        std::vector<cv::Point> current;
        current.push_back(initial);
        stack.push(current);
        while (!stack.empty())
        {
            current = stack.top();
            stack.pop();
            if (maze.IsNotDiscovered(current.back()))
            {
                maze.MarkDiscovered(current.back());
                if (current.back() == goal)
                {
                    solution = current;
                    return true;
                }

                if (!maze.LeftIsWall(current.back()))
                {
                    stack.emplace(std::vector<cv::Point>(current));
                    stack.top().emplace_back(maze.Left(current.back()));
                }
                if (!maze.RightIsWall(current.back()))
                {
                    stack.emplace(std::vector<cv::Point>(current));
                    stack.top().emplace_back(maze.Right(current.back()));
                }
                if (!maze.TopIsWall(current.back()))
                {
                    stack.emplace(std::vector<cv::Point>(current));
                    stack.top().emplace_back(maze.Top(current.back()));
                }
                if (!maze.BottomIsWall(current.back()))
                {
                    stack.emplace(std::vector<cv::Point>(current));
                    stack.top().emplace_back(maze.Bottom(current.back()));
                }
            }
        }
        return false;
    }

    bool MazeSolver::DfsIterative(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution)
    {
        initial = initial * 2 + cv::Point(1, 1);
        goal = goal * 2 + cv::Point(1, 1);
        std::stack<cv::Point> stack;
        stack.push(initial);

        while (!stack.empty())
        {
            initial = stack.top();
            stack.pop();
            if (maze.IsNotDiscovered(initial))
            {
                maze.MarkDiscovered(initial);

                if (initial == goal)
                {
                    return true;
                }
                if (!maze.LeftIsWall(initial))
                {
                    stack.push(maze.Left(initial));
                }
                if (!maze.RightIsWall(initial))
                {
                    stack.push(maze.Right(initial));
                }
                if (!maze.TopIsWall(initial))
                {
                    stack.push(maze.Top(initial));
                }
                if (!maze.BottomIsWall(initial))
                {
                    stack.push(maze.Bottom(initial));
                }
            }
        }
        return false;
    }

    MazeSolver::DlsResult MazeSolver::Dls(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution, size_t limit, Implementation implementation)
    {
        initial = initial * 2 + cv::Point(1, 1);
        goal = goal * 2 + cv::Point(1, 1);
        solution_.clear();
        if (implementation == Implementation::recursive)
        {
            auto result = DlsRecursive(maze, initial, goal, limit);
            if (result == DlsResult::solution)
            {
                solution_.push_back(initial);
                solution = solution_;
            }
            return result;
        }
        else
        {
            std::cerr << "Dls iterative is not implemented." << std::endl;
            return DlsResult::failure;
        }
    }

    bool MazeSolver::IterativeDeepeningSearch(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution, Implementation implementation)
    {
        // initial = initial * 2 + cv::Point(1, 1);
        // goal = goal * 2 + cv::Point(1, 1);
        for (size_t i = 0; i < SIZE_MAX; i++)
        {
            auto cleanMaze = maze.Clone();
            auto result = Dls(cleanMaze, initial, goal, solution, i, implementation);
            if (result != DlsResult::cutoff)
            {
                if (result == DlsResult::solution)
                {
                    maze = cleanMaze;
                    return true;
                }
                else if (result == DlsResult::failure)
                {
                    return false;
                }
            }
        }
    }

    bool MazeSolver::UniformCostSearch(aisa::Maze &maze, cv::Point initial, cv::Point goal, std::vector<cv::Point> &solution)
    {
        initial = initial * 2 + cv::Point(1, 1);
        goal = goal * 2 + cv::Point(1, 1);

        aisa::Tree<cv::Point> *node = new aisa::Tree<cv::Point>(initial, 0, nullptr);
        auto cmp = [](aisa::Tree<cv::Point> *left, aisa::Tree<cv::Point> *right)
        {
            return (left->PathCost()) < (right->PathCost());
        };
        std::deque<aisa::Tree<cv::Point> *> frontier;
        frontier.push_back(node);
        std::sort(frontier.begin(), frontier.end(), cmp);
        maze.MarkFrontier(node->State());
        while (true)
        {
            if (frontier.empty())
            {
                return false;
            }
            node = frontier.front();
            frontier.pop_front();
            maze.MarkRoad(node->State());
            if (node->State() == goal)
            {
                solution.push_back(node->State());
                while (node->Parent() != nullptr)
                {
                    node = node->Parent();
                    solution.push_back(node->State());
                }
                delete node;
                return true;
            }
            maze.MarkDiscovered(node->State());
            auto ways = maze.Ways(node->State());
            for (auto &&way : ways)
            {
                node->AddChild(way, 1);
            }

            for (auto &&child : node->Children())
            {
                if ((!maze.IsFrontier(child.State())) && maze.IsNotDiscovered(child.State()))
                {
                    frontier.push_back(&child);
                    maze.MarkFrontier(child.State());
                    std::sort(frontier.begin(), frontier.end(), cmp);
                }
                else if (maze.IsFrontier(child.State()))
                {
                    auto f = std::find(frontier.begin(), frontier.end(), &child);
                    if (f != frontier.end() && (*f)->PathCost() > child.PathCost())
                    {
                        auto index = f - frontier.begin();
                        maze.MarkRoad(frontier[index]->State());
                        frontier[index] = &child;
                    }
                }
            }
        }
    }

    MazeSolver::DlsResult MazeSolver::DlsRecursive(aisa::Maze &maze, cv::Point initial, cv::Point goal, size_t limit)
    {
        maze.MarkDiscovered(initial);
        if (initial == goal)
        {
            return DlsResult::solution;
        }
        else if (limit == 0)
        {
            return DlsResult::cutoff;
        }
        else
        {
            bool cutoffOccured = false;

            auto ways = maze.Ways(initial);
            for (int i = ways.size() - 1; i >= 0; i--)
            {
                if (maze.IsNotDiscovered(ways[i]))
                {
                    auto result = DlsRecursive(maze, ways[i], goal, limit - 1);
                    if (result == DlsResult::cutoff)
                    {
                        cutoffOccured = true;
                    }
                    else if (result != DlsResult::failure)
                    {
                        if (result == DlsResult::solution)
                        {
                            solution_.push_back(ways[i]);
                        }
                        return result;
                    }
                }
            }

            if (cutoffOccured)
            {
                return DlsResult::cutoff;
            }
            else
            {
                return DlsResult::failure;
            }
        }
    }
}