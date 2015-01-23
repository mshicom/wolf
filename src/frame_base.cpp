
#include "frame_base.h"

FrameBase::FrameBaseX(const double & _ts) :
            NodeLinked(TOP, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts), 
            state_(nullptr)
{
    //
}

FrameBaseX(const FrameType & _tp, const double & _ts, const WolfScalar * _st) :
            NodeLinked(TOP, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts), 
            state_(_st)
{
    //
}
                
FrameBase::~FrameBaseX()
{
    //
}

inline bool isKey() const;
{
    if ( type_ == KEY_FRAME ) return true;
    else return false; 
}

inline void setType(FrameType _ft)
{
    type_ = _ft;
}

inline void setTimeStamp(const WolfScalar & _ts)
{
    time_stamp_ = _ts;
}


