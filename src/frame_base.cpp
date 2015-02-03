
#include "frame_base.h"

FrameBase::FrameBase(const WolfScalar & _ts, const StateBaseShPtr& _p_ptr, const StateBaseShPtr& _o_ptr, const StateBaseShPtr& _v_ptr, const StateBaseShPtr& _w_ptr) :
            NodeLinked(TOP, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr),
			v_ptr_(_v_ptr),
			w_ptr_(_w_ptr)
{
    //
}

FrameBase::FrameBase(const FrameType & _tp, const WolfScalar & _ts, const StateBaseShPtr& _p_ptr, const StateBaseShPtr& _o_ptr, const StateBaseShPtr& _v_ptr, const StateBaseShPtr& _w_ptr) :
            NodeLinked(TOP, "FRAME"),
            type_(REGULAR_FRAME),
            time_stamp_(_ts),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr),
			v_ptr_(_v_ptr),
			w_ptr_(_w_ptr)
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

//inline const Eigen::Vector3s & FrameBase::state() const
//{
//    return state_;
//}

StateBaseShPtr FrameBase::getPPtr() const
{
	return p_ptr_;
}

StateBaseShPtr FrameBase::getOPtr() const
{
	return o_ptr_;
}

StateBaseShPtr FrameBase::getVPtr() const
{
	return v_ptr_;
}

StateBaseShPtr FrameBase::getWPtr() const
{
	return w_ptr_;
}

void FrameBase::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    NodeLinked::printSelf(_ntabs, _ost);
    if (p_ptr_.get() != nullptr)
    {
    	_ost << "\tPosition : \n";
    	p_ptr_->printSelf(_ntabs,_ost);
    }
    if (o_ptr_.get() != nullptr)
    {
		_ost << "\tOrientation : \n";
		o_ptr_->printSelf(_ntabs,_ost);
    }
    if (v_ptr_.get() != nullptr)
    {
    	_ost << "\tVelocity : \n";
    	v_ptr_->printSelf(_ntabs,_ost);
    }
    if (w_ptr_.get() != nullptr)
    {
    	_ost << "\tAngular velocity : \n";
    	v_ptr_->printSelf(_ntabs,_ost);
    }

}



