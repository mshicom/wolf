#ifndef ROSBAGIMAGESOURCE_H
#define ROSBAGIMAGESOURCE_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv/cv.h>

class RosDataSource {
 protected:
  rosbag::Bag mbag;
  rosbag::View *mpview;
  rosbag::View::iterator mit;
  unsigned int height, width;

 public:
  RosDataSource(){}
  ~RosDataSource(){ close(); }
  inline unsigned int get_height() const { return height; }
  inline unsigned int get_width() const { return width; }

  bool open(const std::string &bagFile, const std::string &imageTopic);
  void close(void);
  void dropFrames(int count);
  bool update(cv::Mat &out, double &ts);
};

#endif  // ROSBAGIMAGESOURCE_H
