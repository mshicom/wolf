
#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

// Fwd refs
namespace wolf{
class Problem;
class FrameBase;
}

//Wolf includes
#include "wolf.h"
#include "node_linked.h"

//std includes


namespace wolf {

//class TrajectoryBase
class TrajectoryBase : public NodeLinked<Problem,FrameBase>
{
    protected:
        FrameStructure frame_structure_; // Defines the structure of the Frames in the Trajectory.
        FrameBase* last_key_frame_ptr_;  // keeps pointer to the last key frame
        
    public:
        TrajectoryBase(const FrameStructure _frame_sturcture);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/        
        virtual ~TrajectoryBase();
        
        /** \brief Add a frame to the trajectory
         **/
        virtual FrameBase* addFrame(FrameBase* _frame_ptr);

        /** \brief Remove a frame to the trajectory
         **/
        virtual void removeFrame(const FrameBaseIter& _frame_iter);

        /** \brief Returns a pointer to frame list
         **/
        FrameBaseList* getFrameListPtr();

        /** \brief Returns a pointer to last frame
         **/
        FrameBase* getLastFramePtr();

        /** \brief Returns a pointer to last key frame
         */
        FrameBase* getLastKeyFramePtr();

        /** \brief Returns a list of all constraints in the trajectory thru reference
         **/
        void getConstraintList(ConstraintBaseList & _ctr_list);
        
        /** \brief Returns the frame structure (see wolf.h)
         **/
        FrameStructure getFrameStructure() const;

        /** \brief Sets the frame as keyframe
         **/
        virtual void setKeyFrame(FrameBase* _frame_ptr);

        /** \brief Finds the closes key frame to a given timestamp
         **/
        FrameBase* closestKeyFrameToTimeStamp(const TimeStamp& _ts);

    protected:

        /** \brief Sorts the frame by timestamp
         **/
        void sortFrame(FrameBase* _frame_ptr);

        /** \brief Compute the position where the frame should be
         **/
        FrameBaseIter computeFrameOrder(FrameBase* _frame_ptr);

};

inline void TrajectoryBase::removeFrame(const FrameBaseIter& _frame_iter)
{
    removeDownNode(_frame_iter);
}

inline FrameBaseList* TrajectoryBase::getFrameListPtr()
{
    return getDownNodeListPtr();
}

inline FrameBase* TrajectoryBase::getLastFramePtr()
{
    return getDownNodeListPtr()->back();
}

inline FrameBase* TrajectoryBase::getLastKeyFramePtr()
{
    return last_key_frame_ptr_;
}

inline FrameStructure TrajectoryBase::getFrameStructure() const
{
    return frame_structure_;
}


} // namespace wolf

#endif
