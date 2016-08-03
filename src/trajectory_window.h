/**
 * \file trajectory_window.h
 *
 *  Created on: Aug 3, 2016
 *      \author: jvallve
 */

#ifndef TRAJECTORY_WINDOW_H_
#define TRAJECTORY_WINDOW_H_

#include "trajectory_base.h"

namespace wolf
{

class TrajectoryWindow : public TrajectoryBase
{
    protected:
        unsigned int max_window_keyframes_;
        unsigned int window_keyframes_;

    public:
        /** \brief Constructor from frame structure and max window keyframes
         * \param _frame_structure: the frame structure (2D, 3D, position, orientation, velocity) see wolf.h
         * \param _max_window_keyframes: amount of key frames in the window.
         */
        TrajectoryWindow(const FrameStructure _frame_sturcture, const unsigned int _max_window_keyframes);
        virtual ~TrajectoryWindow();

        /** \brief Add a frame to the trajectory
         **/
        virtual FrameBase* addFrame(FrameBase* _frame_ptr);

        /** \brief Remove a frame to the trajectory
         **/
        virtual void removeFrame(const FrameBaseIter& _frame_iter);

        /** \brief Sets the frame as keyframe
         **/
        virtual void setKeyFrame(FrameBase* _frame_ptr);

    protected:
        virtual void manageWindow();

};

} /* namespace wolf */

#endif /* TRAJECTORY_WINDOW_H_ */
