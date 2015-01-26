
#ifndef CAPTURE_BASE_H_
#define CAPTURE_BASE_H_

// Forward declarations for node templates
class FrameBase;
class FeatureBase;

//std includes
//

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "frame_base.h"
#include "feature_base.h"

//class CaptureBase
class CaptureBase : public NodeLinked<FrameBase,FeatureBase>
{
    protected:
        double time_stamp_; //capture ts
        
    public:
        CaptureBase(double _ts) :
            NodeLinked(MID, "CAPTURE"),
            time_stamp_(_ts)
        {
            //
        };
        
        virtual ~CaptureBase()
        {
            
        };
};
#endif
