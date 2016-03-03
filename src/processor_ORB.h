#ifndef PROCESSORORB_H
#define PROCESSORORB_H

// Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"

//OpenCV includes
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include ORBextractor.h

class ProcessorORB : public ProcessorBase
{
    protected:
        ProcessorSensorCamera* sensor_cam_ptr_; //specific pointer to sensor camera object
        CaptureImage* capture_img_ptr_; //specific pointer to capture image object;
        //add an ORBextractor
        //add Settings (camera parameters, nFeatures, ScaleFactor, nLevels, FIniThFAST,fMinThFAST)

    public:
        ProcessorORB();
        virtual ~ProcessorORB();

        virtual void extractFeatures(CaptureBase *_capture_ptr);
        virtual void establishConstraints(CaptureBase *_capture_ptr);
};

#endif // PROCESSORORB_H
