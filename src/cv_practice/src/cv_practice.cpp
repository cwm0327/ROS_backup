
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include </usr/include/opencv4/opencv2/highgui/highgui_c.h>

using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

double pose_data[7] = {0};

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose_data[0]=pose.pose.pose.position.x;
  pose_data[1]=pose.pose.pose.position.y;
  pose_data[2]=pose.pose.pose.position.z;
  pose_data[3]=pose.pose.pose.orientation.x;
  pose_data[4]=pose.pose.pose.orientation.y;
  pose_data[5]=pose.pose.pose.orientation.z;
  pose_data[6]=pose.pose.pose.orientation.w;
}

/*void depthCallback(const sensor_msgs::Image& depth_image)
{
  cv_bridge::CvImagePtr depth_ptr;
  
  cv::Mat depth_mat;

  depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);

  depth_ptr->image.copyTo(depth_mat);

  cv::imshow("depth_mat", depth_mat);

  cv::waitKey(3);
}
void rgbCallback(const sensor_msgs::Image& rgb_image)
{

  cv_bridge::CvImagePtr rgb_ptr;
 
  cv::Mat rgb_mat;

  rgb_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);

  rgb_ptr->image.copyTo(rgb_mat);

  cv::imshow("rgb_mat", rgb_mat);

  cv::waitKey(3);
}*/

//CameraInfoConstPtr
void image_callback(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  cv_bridge::CvImagePtr rgb_ptr, depth_ptr;

  cv::Mat rgb_mat, depth_mat;

  rgb_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  rgb_ptr->image.copyTo(rgb_mat);
  cv::imshow("rgb_mat", rgb_mat);

  depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  depth_ptr->image.copyTo(depth_mat);
  cv::imshow("depth_mat", depth_mat);
  //cout<<"asdf";
  cv::waitKey(3);
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "cv_practice");

    // 创建节点句柄
    ros::NodeHandle RDM;

    std::string rgb_topic;
    std::string depth_topic;
 
    RDM.param<std::string>("/rgb_topic",   rgb_topic,   "/camera/rgb/image_raw");
    RDM.param<std::string>("/depth_topic", depth_topic, "/camera/depth/image_raw");

    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    //ros::Subscriber pose_sub = RDM.subscribe("/rot_pose_ekf/odom_combined", 10, poseCallback);
    //ros::Subscriber rgb_sub = RDM.subscribe(rgb_topic, 10, rgbCallback);
    //ros::Subscriber depth_sub = RDM.subscribe(depth_topic, 10, depthCallback);
    message_filters::Subscriber<Image> rgb_image_sub        (RDM, rgb_topic,   1);
    message_filters::Subscriber<Image> depth_image_sub      (RDM, depth_topic, 1);
    message_filters::Subscriber<CameraInfo> camera_info_sub (RDM, "/camera/rgb/camera_info", 1);

    typedef message_filters::sync_policies::ApproximateTime<Image,
          Image,
          CameraInfo> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20),
                                                   rgb_image_sub,
                                                   depth_image_sub,
                                                   camera_info_sub);
    sync.registerCallback(boost::bind(&image_callback, _1, _2, _3));

    //TimeSynchronizer<Image, CameraInfo> sync(depth_image_sub, camera_info_sub, 10);
    //TimeSynchronizer<Image, CameraInfo> sync(rgb_image_sub, camera_info_sub, 10);
    //sync.registerCallback(boost::bind(&image_callback, _1, _2));

    // 循环等待回调函数
    ros::spin();

    return 0;
}
