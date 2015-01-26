#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

// Forward declarations for node templates
class CaptureBase;
class CorrespondenceBase;

//std includes

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "capture_base.h"
#include "correspondence_base.h"

//class FeatureBase
class FeatureBase : public NodeLinked<CaptureBase,CorrespondenceBase>
{
    protected:
        
    public:
        FeatureBase() :
            NodeLinked(MID, "FEATURE")
        {
            //
        };
        
        virtual ~FeatureBase()
        {
            //
        };
};
#endif
