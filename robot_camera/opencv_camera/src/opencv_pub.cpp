#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt8MultiArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_camera_pub");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("/camera/image",1);
  cv::VideoCapture cap(0);
  cv::Mat frame;

  while(nh.ok())
  {
    cap >> frame;
    if(!frame.empty())
    {
      //cv::imshow("frame", frame);

      std::vector<uchar> encode;
      std::vector<int> encode_param;

      encode_param.push_back(cv::IMREAD_UNCHANGED);
      encode_param.push_back(20);

      cv::imencode(".jpg", frame, encode,encode_param);
      cv::Mat decode = cv::imdecode(encode,1);
      //cv::imshow("decode", decode);

      std_msgs::UInt8MultiArray msgArray;
      msgArray.data.clear();
      msgArray.data.resize(encode.size());
      std::copy(encode.begin(), encode.end(), msgArray.data.begin());

      pub.publish(msgArray);

      cv::waitKey(1);

    }
    ros::spinOnce();

  }
  return 0;
}
