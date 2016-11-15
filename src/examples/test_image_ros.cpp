// Testing things for the 3D image odometry

//Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"
#include "processor_image.h"

#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <exception>
//#include <opencv/highgui.h>
#include <opencv/cv.h>

//std includes
#include <ctime>
#include <iostream>

namespace enc = sensor_msgs::image_encodings;
int getCvType(const std::string& encoding)
{
  // Check for the most common encodings first
  if (encoding == enc::BGR8)   return CV_8UC3;
  if (encoding == enc::MONO8)  return CV_8UC1;
  if (encoding == enc::RGB8)   return CV_8UC3;
  if (encoding == enc::MONO16) return CV_16UC1;
  if (encoding == enc::BGR16)  return CV_16UC3;
  if (encoding == enc::RGB16)  return CV_16UC3;
  if (encoding == enc::BGRA8)  return CV_8UC4;
  if (encoding == enc::RGBA8)  return CV_8UC4;
  if (encoding == enc::BGRA16) return CV_16UC4;
  if (encoding == enc::RGBA16) return CV_16UC4;

  // For bayer, return one-channel
  if (encoding == enc::BAYER_RGGB8) return CV_8UC1;
  if (encoding == enc::BAYER_BGGR8) return CV_8UC1;
  if (encoding == enc::BAYER_GBRG8) return CV_8UC1;
  if (encoding == enc::BAYER_GRBG8) return CV_8UC1;
  if (encoding == enc::BAYER_RGGB16) return CV_16UC1;
  if (encoding == enc::BAYER_BGGR16) return CV_16UC1;
  if (encoding == enc::BAYER_GBRG16) return CV_16UC1;
  if (encoding == enc::BAYER_GRBG16) return CV_16UC1;
  assert(false);
}

class RosDataSource
{
protected:
    rosbag::Bag mbag;
    rosbag::View *mpview;
    rosbag::View::iterator mit;
    int height,width;
public:
    RosDataSource(){}
    ~RosDataSource(){close();}
    inline int get_height(){return height;}
    inline int get_width() {return width;}

    bool open(const std::string &bagFile, const std::string &imageTopic)
    {
        try {
            mbag.open(bagFile, rosbag::bagmode::Read);
        }
        catch(rosbag::BagException e) {
            return false;
        }
        std::vector<std::string> topics;
        topics.push_back(imageTopic);

        mpview = new rosbag::View(mbag, rosbag::TopicQuery(topics));

        mit = mpview->begin();
        if(mit==mpview->end())
            return false;

        sensor_msgs::ImageConstPtr pimg =  mit->instantiate<sensor_msgs::Image>();
        if(pimg == NULL)
            return false;
        else {
            width = pimg->width;
            height = pimg->height;
            return true;
        }
    }
    void close(void)
    {
        delete mpview;
        mbag.close();
    }

    void dropFrames(int count)
    {
        for(int i=0; i<count && mit!= mpview->end(); i++)
            mit++;
    }

    bool update(cv::Mat &out,  double &ts)
    {
        if (mit == mpview->end())
            return false;

        sensor_msgs::ImageConstPtr pimg =  (mit++)->instantiate<sensor_msgs::Image>();
        if(pimg == NULL)
            return false;

        int type = getCvType(pimg->encoding);
        cv::Mat frame(pimg->height,
                      pimg->width,
                      type,
                      const_cast<uchar*>(&pimg->data[0]),
                      pimg->step);
        if(frame.type() == CV_8UC1)
            cv::cvtColor(frame,out,cv::COLOR_GRAY2RGB);
        else
            out = frame;
        ts = pimg->header.stamp.toSec();

        return true;
    }

};

int main(int argc, char** argv)
{
    using namespace wolf;

    //ProcessorImage test
    std::cout << std::endl << " ========= ProcessorImage test ===========" << std::endl << std::endl;

    RosDataSource capture;
    std::string filename,topicname;
    if (argc == 1)
    {
        //filename = "/home/nubot/data/bumblebee_11170132.bag";
        //topicname = "/camera/image_raw";
        filename = "/home/kaihong/dataset/bumblebee_11170132.bag";
        topicname = "/stereo/11170132/left";
    }
    else
    {
        filename = argv[1];
        topicname = argv[2];
    }
    std::cout << "Input video file: " << filename << std::endl;
    if(capture.open(filename, topicname))
        std::cout << "succeded" << std::endl;
    else
        throw std::runtime_error("failed to open file.");

    unsigned int img_width  = capture.get_width();
    unsigned int img_height = capture.get_height();
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    unsigned int buffer_size = 8;
    std::vector<cv::Mat> frame(buffer_size);

    TimeStamp t = 1;

    char const* tmp = std::getenv( "WOLF_ROOT" );
    if ( tmp == nullptr )
        throw std::runtime_error("WOLF_ROOT environment not loaded.");

    std::string wolf_path( tmp );

    std::cout << "Wolf path: " << wolf_path << std::endl;

    Problem* wolf_problem_ = new Problem(FRM_PO_3D);

    //=====================================================
    // Method 1: Use data generated here for sensor and processor
    //=====================================================

    //    // SENSOR
    //    Eigen::Vector4s k = {320,240,320,320};
    //    SensorCamera* sen_cam_ = new SensorCamera(new StateBlock(Eigen::Vector3s::Zero()),
    //                                              new StateBlock(Eigen::Vector3s::Zero()),
    //                                              new StateBlock(k,false),img_width,img_height);
    //
    //    wolf_problem_->getHardwarePtr()->addSensor(sen_cam_);
    //
    //    // PROCESSOR
    //    ProcessorParamsImage tracker_params;
    //    tracker_params.image = {img_width,  img_height};
    //    tracker_params.matcher.min_normalized_score = 0.75;
    //    tracker_params.matcher.similarity_norm = cv::NORM_HAMMING;
    //    tracker_params.matcher.roi_width = 30;
    //    tracker_params.matcher.roi_height = 30;
    //    tracker_params.active_search.grid_width = 12;
    //    tracker_params.active_search.grid_height = 8;
    //    tracker_params.active_search.separation = 1;
    //    tracker_params.algorithm.max_new_features =0;
    //    tracker_params.algorithm.min_features_for_keyframe = 20;
    //
    //    DetectorDescriptorParamsOrb orb_params;
    //    orb_params.type = DD_ORB;
    //
    //    DetectorDescriptorParamsBrisk brisk_params;
    //    brisk_params.type = DD_BRISK;
    //
    //    // select the kind of detector-descriptor parameters
    //    tracker_params.detector_descriptor_params_ptr = &orb_params; // choose ORB
    //
    //    ProcessorImage* prc_image = new ProcessorImage(tracker_params);
    //
    //    sen_cam_->addProcessor(prc_image);
    //=====================================================


    //=====================================================
    // Method 2: Use factory to create sensor and processor
    //=====================================================

    // SENSOR
    // one-liner API
    SensorBase* sensor_ptr = wolf_problem_->installSensor("CAMERA", "PinHole", Eigen::VectorXs::Zero(7), wolf_path + "/src/examples/camera_params.yaml");
    SensorCamera* camera_ptr = (SensorCamera*)sensor_ptr;

    // PROCESSOR
    // one-liner API
    wolf_problem_->installProcessor("IMAGE", "ORB", "PinHole", wolf_path + "/src/examples/processor_image_ORB.yaml");

    //=====================================================


    // CAPTURES
    unsigned int f  = 1;
    double ts;
    bool success = capture.update(frame[f % buffer_size], ts);

    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);

    while(success)
    {
        std::cout << "\n=============== Frame #: " << f << " in buffer: " << f%buffer_size << " ===============" << std::endl;

        t.setToNow();

        clock_t t1 = clock();

        // Old method with non-factory objects
        //        capture_image_ptr = new CaptureImage(t, sen_cam_,frame[f % buffer_size]);
        //        prc_image->process(capture_image_ptr);

        // Preferred method with factory objects:
        CaptureImage* image_ptr = new CaptureImage(t, camera_ptr, frame[f % buffer_size]);
        image_ptr->process();
        //cv::imshow("test",frame[f % buffer_size]);
        std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
        cv::waitKey(5);

        f++;
        success = capture.update(frame[f % buffer_size], ts);
    }

    wolf_problem_->destruct();
}
