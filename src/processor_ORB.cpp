// Wolf includes
#include "processor_ORB.h"

#include "unistd.h"

ProcessorORB::ProcessorORB()
{
    std::cout << "ProcessorORB constructor" << std::endl;
}

ProcessorORB::~ProcessorORB()
{
    std::cout << "ProcessorORB destructor" << std::endl;
}

void ProcessorORB::extractFeatures(CaptureBase *_capture_ptr)
{
    capture_img_ptr_ = (CaptureImage*)(_capture_ptr); //uses a capture
    cv::Mat img = capture_img_ptr_->getImage();
    std::cvtColor(img,img,CV_RGB2GRAY);
    //create the ORB extractor
    orb_extractor_ptr = new ORBextractor(2*this->nFeatures,this->fScaleFactor,this->nLevels,this->fIniThFAST,this->fMinThFAST);

    ///needed variables to run ORB
    // Vector of keypoints
    std::vector<cv::KeyPoint> keypoints;
    // ORB descriptor, each row associated to a keypoint.
    cv::Mat descriptors
    //extract features
    (*orb_extractor_ptr)(im,cv::Mat(),keypoints,descriptors);
}

void ProcessorORB::establishConstraints(CaptureBase *_capture_ptr)
{
    //not used yet
}
