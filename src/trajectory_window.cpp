/**
 * \file trajectory_window.cpp
 *
 *  Created on: Aug 3, 2016
 *      \author: jvallve
 */

#include "trajectory_window.h"

namespace wolf
{

TrajectoryWindow::TrajectoryWindow(const FrameStructure _frame_sturcture, const unsigned int _max_window_keyframes) :
        TrajectoryBase(_frame_sturcture), max_window_keyframes_(_max_window_keyframes), window_keyframes_(0)
{
    assert(max_window_keyframes_ > 0 && "trajectory window with 0 keyframes");
}

TrajectoryWindow::~TrajectoryWindow()
{
    //
}

FrameBase* TrajectoryWindow::addFrame(FrameBase* _frame_ptr)
{
    std::cout << "TrajectoryWindow::addFrame: " << _frame_ptr->id() << std::endl;

    // add frame
    FrameBase* new_frame = TrajectoryBase::addFrame(_frame_ptr);

    // update key_frame counter
    if (_frame_ptr->isKey())
    {
        window_keyframes_++;
        manageWindow();
    }

    std::cout << "window keyframes: " << window_keyframes_ << std::endl;

    return new_frame;
}

void TrajectoryWindow::removeFrame(const FrameBaseIter& _frame_iter)
{
    if ((*_frame_iter)->isKey())
        window_keyframes_--;

    TrajectoryBase::removeFrame(_frame_iter);
}

void TrajectoryWindow::setKeyFrame(FrameBase* _frame_ptr)
{
    TrajectoryBase::setKeyFrame(_frame_ptr);
    window_keyframes_++;
    manageWindow();
}

void TrajectoryWindow::manageWindow()
{
    // threshold exceeded -> remove first keyframe
    if (window_keyframes_ > max_window_keyframes_)
    {
        std::cout << "max_window_keyframes exceed" << std::endl;
        // find first keyframe
        auto first_key_frame_it = getDownNodeListPtr()->begin();
        while (!(*first_key_frame_it)->isKey() && first_key_frame_it != getDownNodeListPtr()->end())
            first_key_frame_it++;

        assert(first_key_frame_it != getDownNodeListPtr()->end() && "no keyframes in trajectory");

        // remove first keyframe
        removeFrame(first_key_frame_it);
    }
}

} /* namespace wolf */
