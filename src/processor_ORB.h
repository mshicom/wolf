#ifndef PROCESSORORB_H
#define PROCESSORORB_H

// Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"

//OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "ORBextractor.h"

class ProcessorORB : public ProcessorBase
{
    protected:
        SensorCamera* sensor_cam_ptr_; //specific pointer to sensor camera object
        CaptureImage* capture_img_ptr_; //specific pointer to capture image object;
        ORBextractor* orb_extractor_ptr;

    private:
        int nFeatures = 1000;
        float fScaleFactor = 1.2;
        int nLevels = 8;
        int fIniThFAST = 20;
        int fMinThFAST = 7;

    public:
        ProcessorORB();
        virtual ~ProcessorORB();

        virtual void extractFeatures(CaptureBase *_capture_ptr);
        virtual void establishConstraints(CaptureBase *_capture_ptr);
        virtual void process(CaptureBase* _capture_ptr);

        void drawImage();
};

#endif // PROCESSORORB_H
