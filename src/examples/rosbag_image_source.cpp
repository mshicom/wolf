#include "rosbag_image_source.h"

#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <opencv/cv.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;
int getCvType(const std::string &encoding) {
  // Check for the most common encodings first
  if (encoding == enc::BGR8) return CV_8UC3;
  if (encoding == enc::MONO8) return CV_8UC1;
  if (encoding == enc::RGB8) return CV_8UC3;
  if (encoding == enc::MONO16) return CV_16UC1;
  if (encoding == enc::BGR16) return CV_16UC3;
  if (encoding == enc::RGB16) return CV_16UC3;
  if (encoding == enc::BGRA8) return CV_8UC4;
  if (encoding == enc::RGBA8) return CV_8UC4;
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

bool RosDataSource::open(const std::string &bagFile,
                         const std::string &imageTopic) {
  try {
    mbag.open(bagFile, rosbag::bagmode::Read);
  } catch (rosbag::BagException& e) {
    return false;
  }
  std::vector<std::string> topics;
  topics.push_back(imageTopic);

  mpview = new rosbag::View(mbag, rosbag::TopicQuery(topics));

  mit = mpview->begin();
  if (mit == mpview->end()) return false;

  sensor_msgs::ImageConstPtr pimg = mit->instantiate<sensor_msgs::Image>();
  if (pimg == NULL)
    return false;
  else {
    width = pimg->width;
    height = pimg->height;
    return true;
  }
}

void RosDataSource::close(void) {
  delete mpview;
  mbag.close();
}

void RosDataSource::dropFrames(int count) {
  for (int i = 0; i < count && mit != mpview->end(); i++) ++mit;
}

bool RosDataSource::update(cv::Mat &out, double &ts) {
  if (mit == mpview->end()) return false;

  sensor_msgs::ImageConstPtr pimg = (mit++)->instantiate<sensor_msgs::Image>();
  if (pimg == NULL) return false;

  int type = getCvType(pimg->encoding);
  cv::Mat frame(static_cast<int>(pimg->height),
                static_cast<int>(pimg->width), type,
                const_cast<uchar *>(&pimg->data[0]), pimg->step);
  if (frame.type() == CV_8UC1)
    cv::cvtColor(frame, out, cv::COLOR_GRAY2RGB);
  else
    out = frame;
  ts = pimg->header.stamp.toSec();

  return true;
}
