#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


void imageCallback(const std_msgs::UInt8MultiArray::ConstPtr& array)
{
  try
  {
    cv::Mat frame = cv::imdecode(array->data,1);
    cv::imshow("view", frame);
    cv::waitKey(1);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cannot decode image");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opencv_sub");

  cv::namedWindow("view");
  cv::startWindowThread();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("camera/image",5, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
}
