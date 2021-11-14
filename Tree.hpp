#ifndef __TREE_H__
#define __TREE_H__

/**
 * @file Tree.hpp
 * @author Ali Emre Keskin (aliemrekskn@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <vector>
#include <opencv2/opencv.hpp>

namespace aisa
{
    template <class T>
    class Tree
    {
    public:
        Tree(T state, double pathCost, aisa::Tree<T> *parent);
        ~Tree();
        void AddChild(T state, double cost);
        std::vector<aisa::Tree<T>> &Children();
        double &PathCost();
        T &State();
        aisa::Tree<T> *Parent();
        friend bool operator==(const Tree<cv::Point> n1, const Tree<cv::Point> n2)
        {
            return n1.state_ == n2.state_;
        }

        T state_;

    private:
        aisa::Tree<T> *parent_;
        double pathCost_;
        std::vector<Tree> children_;
    };

    template <class T>
    Tree<T>::Tree(T state, double pathCost, aisa::Tree<T> *parent)
        : state_(state),
          pathCost_(pathCost),
          parent_(parent)
    {
    }

    template <class T>
    Tree<T>::~Tree()
    {
    }

    template <class T>
    void Tree<T>::AddChild(T state, double cost)
    {
        children_.push_back(Tree<T>(state, pathCost_ + cost, this));
    }

    template <class T>
    std::vector<aisa::Tree<T>> &Tree<T>::Children()
    {
        return children_;
    }

    template <class T>
    double &Tree<T>::PathCost()
    {
        return pathCost_;
    }

    template <class T>
    T &Tree<T>::State()
    {
        return state_;
    }

    template <class T>
    aisa::Tree<T> *Tree<T>::Parent()
    {
        return parent_;
    }

} // namespace aisa

#endif // __TREE_H__