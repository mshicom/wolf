
#ifndef FRAME_BASE_H_
#define FRAME_BASE_H_

class NodeTerminus;
class CaptureBase;

//std includes
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

//Wolf includes
 #include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "node_terminus.h"
#include "capture_base.h"

//class FrameBase
class FrameBase : public NodeLinked<NodeTerminus,CaptureBase>
{
    protected:
        FrameType type_; //type of frame. Either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
        TimeStamp time_stamp_; //frame time stamp
        std::shared_ptr<const WolfScalar> state_; //TBD: Instead , It could be a vector/list of pointers to state units
        
    public:
        /** \brief Constructor with only time stamp
         * 
         * Constructor with only time stamp
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * 
         **/
        FrameBase(const WolfScalar & _ts);

        /** \brief Constructor with type, time stamp and state pointer
         * 
         * Constructor with type, time stamp and state pointer
         * \param _tp indicates frame type. Generally either REGULAR_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _st a pointer to the state block marking this frame 
         * 
         **/        
        FrameBase(const FrameType & _tp, const WolfScalar & _ts, const WolfScalar * _st);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~FrameBase();
        
        /** \brief Checks if this frame is KEY_FRAME 
         * 
         * Returns true if type_ is KEY_FRAME. Oterwise returns false.
         * 
         **/
        bool isKey() const;
        
        void setType(FrameType _ft);
        
        void setTimeStamp(const double & _ts);
        
        WolfScalar getTimeStamp() const;
        
        void getTimeStamp(WolfScalar & _ts) const;
//--        
//         void addCapture(CaptureShPtr& _sc_ptr);
// 
//         const CaptureList& captureList() const;
// 
//         const StateShPtr& stateShPtr() const;
//         const StatePtr statePtr() const;
//         const StatePose& state() const;
// 
//         virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
        
        
};
#endif
