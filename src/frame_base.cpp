
#include "frame_base.h"

FrameBase::FrameBase(const double & _ts) :
            NodeLinked(TOP, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts)
{
    //
}

FrameBase::FrameBase(const FrameType & _tp, const double & _ts) :
            NodeLinked(TOP, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts) 
{
    //
}
                
FrameBase::~FrameBase()
{
    //
}

inline bool FrameBase::isKey() const
{
    if ( type_ == KEY_FRAME ) return true;
    else return false; 
}

inline void FrameBase::setType(FrameType _ft)
{
    type_ = _ft;
}

inline void FrameBase::setTimeStamp(const WolfScalar & _ts)
{
    time_stamp_ = _ts;
}

inline WolfScalar FrameBase::getTimeStamp() const
{
    return time_stamp_.get();
}
        
inline void FrameBase::getTimeStamp(WolfScalar & _ts) const
{
    _ts = time_stamp_.get();
}

inline void FrameBase::addCapture(CaptureBaseShPtr & _capt_ptr)
{
    addDownNode(_capt_ptr);
}

inline const CaptureBaseList & FrameBase::captureList() const
{
    return downNodeList();
}

inline const Eigen::Vector3s & FrameBase::state() const
{
    return state_;
}

void FrameBase::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    NodeLinked::printSelf(_ntabs, _ost);
    printTabs(_ntabs);
    _ost << "\tFrame Pose : ( " << state_.transpose() << " )" << std::endl;    
}



