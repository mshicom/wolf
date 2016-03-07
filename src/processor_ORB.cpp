// Wolf includes
#include "processor_ORB.h"

#include "unistd.h"

ProcessorORB::ProcessorORB()
{
    std::cout << "ProcessorORB constructor" << std::endl;
    //create the ORB extractor
    orb_extractor_ptr = new ORBextractor(this->nFeatures,this->fScaleFactor,this->nLevels,this->fIniThFAST,this->fMinThFAST);
}

ProcessorORB::~ProcessorORB()
{
    std::cout << "ProcessorORB destructor" << std::endl;
}

void ProcessorORB::extractFeatures(CaptureBase *_capture_ptr)
{
    capture_img_ptr_ = (CaptureImage*)(_capture_ptr); //uses a capture
    cv::Mat img = capture_img_ptr_->getImage();
    cv::cvtColor(img,img,CV_RGB2GRAY);


    ///needed variables to run ORB
    // Vector of keypoints
    std::vector<cv::KeyPoint> keypoints;
    // ORB descriptor, each row associated to a keypoint.
    std::vector<float> descript_vector;     // Vector to store the descriptor for each keypoint
    cv::Mat descriptors;
    //extract features
    (*orb_extractor_ptr)(img,cv::Mat(),keypoints,descriptors);

    //store features in list<FeatureBase*>
    if(keypoints.size() != 0)
    {
        Eigen::Vector2s ORBImageFeatures;
        for(unsigned int i = 0; i <= (keypoints.size()-1);i++)
        {
            ORBImageFeatures(0) = keypoints[i].pt.x;
            ORBImageFeatures(1) = keypoints[i].pt.y;

            descript_vector=descriptors(cv::Range(i,i+1),cv::Range(0,descriptors.cols));
            capture_img_ptr_->addFeature(new FeaturePoint(ORBImageFeatures, keypoints[i], descript_vector));
            capture_img_ptr_->setKeypoints(keypoints);
        }
    }

    drawImage();
}

void ProcessorORB::establishConstraints(CaptureBase *_capture_ptr)
{

}

void ProcessorORB::process(CaptureBase* _capture_ptr)
{

}

void ProcessorORB::drawImage()
{
    //for each keypoint in frame draw keypoint
    cv::Mat img = this->capture_img_ptr_->getImage();
    std::vector<cv::KeyPoint> keypoint_vector = capture_img_ptr_->getKeypoints();

    for(unsigned int keypoint_nbr=0; keypoint_nbr < keypoint_vector.size(); keypoint_nbr++)
    {
        cv::circle(img,keypoint_vector[keypoint_nbr].pt,2,cv::Scalar(88.0,250.0,154.0),-1,8,0);
    }

    //Visualize the image
    cv::imshow("Keypoint drawing",img);
    std::cout << "capture_img_ptr_ size: " << capture_img_ptr_->getFeatureListPtr()->size() << std::endl;
    cv::waitKey(50);
}
