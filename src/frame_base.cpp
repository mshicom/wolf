
#include "frame_base.h"

FrameBase::FrameBase(const double & _ts) :
            NodeLinked(TOP, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts), 
            state_(nullptr)
{
    //
}

FrameBase::FrameBase(const FrameType & _tp, const double & _ts, const WolfScalar * _st) :
            NodeLinked(TOP, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts), 
            state_(_st)
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

