#include <iostream>
#include <fstream>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
//#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

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

double pose_data[7] = {0};
vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
vector<Eigen::Isometry3d> poses;         // 相机位姿
int frame_catch_number=20,frame_read_number=10;
double cx = 314.5;
double cy = 210.5;
double fx = 581.2;
double fy = -592.0;
double depthScale = 5000.0;

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

void depthCallback(const sensor_msgs::Image& depth_image)
{
  static int time_count=0;

  cv_bridge::CvImagePtr depth_ptr;
  
  cv::Mat depth_mat;

  time_count++;

  depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);

  depth_ptr->image.copyTo(depth_mat);

  cv::imshow("depth_mat", depth_mat);

  cv::waitKey(3);

  depthImgs.push_back(depth_mat);
  if(time_count>20)
  {
   depthImgs.clear();
   time_count=0;
  }
}

void rgbCallback(const sensor_msgs::Image& rgb_image)
{
  static int time_count=0;
  int create_poitcloud=1;

  cv_bridge::CvImagePtr rgb_ptr;
 
  cv::Mat rgb_mat;

  time_count++;

  rgb_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);

  rgb_ptr->image.copyTo(rgb_mat);

  colorImgs.push_back(rgb_mat);

  cv::imshow("rgb_mat", rgb_mat);

  if(cv::waitKey(3)==113)create_poitcloud=0; //q
  else if(cv::waitKey(3)==119)create_poitcloud=1; //w
  

  Eigen::Quaterniond q(pose_data[6], pose_data[3], pose_data[4], pose_data[5]);
  Eigen::Isometry3d T(q);
  T.pretranslate(Eigen::Vector3d(pose_data[0], pose_data[1], pose_data[2]));
  poses.push_back(T);
  if(time_count>frame_catch_number && create_poitcloud)
  {
   time_count=0;
    // 计算点云并拼接
    // 相机内参 
    /*double cx = 319.5;
    double cy = 239.5;
    double fx = 481.2;
    double fy = -480.0;
    double depthScale = 5000.0;*/

    cout << "正在将图像转换为点云..." << endl;
    //ROS_INFO("正在将图像转换为点云...");

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    for (int i = 0; i < frame_read_number; i++) {
        PointCloud::Ptr current(new PointCloud);
        cout << "转换图像中: " << i + 1 << endl;
        //ROS_INFO("转换图像中: %d",i+1);
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;

                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];
                current->points.push_back(p);
            }
        // depth filter and statistical removal 
        PointCloud::Ptr tmp(new PointCloud);
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter(*tmp);
        (*pointCloud) += *tmp;
    }

    pointCloud->is_dense = false;
    cout << "点云共有" << pointCloud->size() << "个点." << endl;
    //ROS_INFO("点云共有%d个点.",pointCloud->size());

    // voxel filter 
    pcl::VoxelGrid<PointT> voxel_filter;
    double resolution = 0.03;
    voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
    PointCloud::Ptr tmp(new PointCloud);
    voxel_filter.setInputCloud(pointCloud);
    voxel_filter.filter(*tmp);
    tmp->swap(*pointCloud);

    cout << "滤波之后，点云共有" << pointCloud->size() << "个点." << endl;
    //ROS_INFO("滤波之后，点云共有%d个点.",pointCloud->size());

    pcl::io::savePCDFileBinary("map.pcd", *pointCloud);

    colorImgs.clear();
    poses.clear();
  }
}

void image_callback(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::ImageConstPtr& depth_image)
{
  static int time_count=0;
  int create_poitcloud=1;

  cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
 
  cv::Mat rgb_mat, depth_mat;

  time_count++;

  rgb_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);

  rgb_ptr->image.copyTo(rgb_mat);
  depth_ptr->image.copyTo(depth_mat);

  colorImgs.push_back(rgb_mat);

  cv::imshow("rgb_mat", rgb_mat);
  cv::imshow("depth_mat", depth_mat);

  if(cv::waitKey(3)==113)create_poitcloud=0; //q
  else if(cv::waitKey(3)==119)create_poitcloud=1; //w
  

  Eigen::Quaterniond q(pose_data[6], pose_data[3], pose_data[4], pose_data[5]);
  Eigen::Isometry3d T(q);
  T.pretranslate(Eigen::Vector3d(pose_data[0], pose_data[1], pose_data[2]));
  poses.push_back(T);
  if(time_count>frame_catch_number && create_poitcloud)
  {
   time_count=0;
    // 计算点云并拼接
    // 相机内参 
    /*double cx = 319.5;
    double cy = 239.5;
    double fx = 481.2;
    double fy = -480.0;
    double depthScale = 5000.0;*/

    //cout << "正在将图像转换为点云..." << endl;
    ROS_INFO("正在将图像转换为点云...");

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    for (int i = 0; i < frame_read_number; i++) {
        PointCloud::Ptr current(new PointCloud);
        //cout << "转换图像中: " << i + 1 << endl;
        ROS_INFO("转换图像中: %d",i+1);
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;

                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];
                current->points.push_back(p);
            }
        // depth filter and statistical removal 
        PointCloud::Ptr tmp(new PointCloud);
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter(*tmp);
        (*pointCloud) += *tmp;
    }

    pointCloud->is_dense = false;
    //cout << "点云共有" << pointCloud->size() << "个点." << endl;
    ROS_INFO("点云共有%d个点.",pointCloud->size());

    // voxel filter 
    pcl::VoxelGrid<PointT> voxel_filter;
    double resolution = 0.03;
    voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
    PointCloud::Ptr tmp(new PointCloud);
    voxel_filter.setInputCloud(pointCloud);
    voxel_filter.filter(*tmp);
    tmp->swap(*pointCloud);

    //cout << "滤波之后，点云共有" << pointCloud->size() << "个点." << endl;
    ROS_INFO("滤波之后，点云共有%d个点.",pointCloud->size());

    pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
    colorImgs.clear();
    depthImgs.clear();
    poses.clear();
  }
}

int main(int argc, char **argv) {

   using namespace sensor_msgs;
   using namespace message_filters;
    // 初始化ROS节点
    ros::init(argc, argv, "RGBD_Dense_Mapping");

    // 创建节点句柄
    ros::NodeHandle RDM;

    std::string rgb_topic;
    std::string depth_topic;

    RDM.param<std::string>("/rgb_topic",   rgb_topic,"/camera/rgb/image_raw");
    RDM.param<std::string>("/depth_topic", depth_topic,"/camera/depth/image");
    RDM.param<int>("frame_catch_number", frame_catch_number,20);
    RDM.param<int>("frame_read_number",  frame_read_number,10);
    RDM.param<double>("cx", cx,  314.5);
    RDM.param<double>("cy", cy,  210.5);
    RDM.param<double>("fx", fx,  581.2);
    RDM.param<double>("fy", fy, -592.0);
    RDM.param<double>("depthScale", depthScale, 5000.0);

    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    ros::Subscriber pose_sub = RDM.subscribe("/rot_pose_ekf/odom_combined", 10, poseCallback);
    ros::Subscriber rgb_sub = RDM.subscribe(rgb_topic, 10, rgbCallback);
    ros::Subscriber depth_sub = RDM.subscribe(depth_topic, 10, depthCallback);
    //message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub  (RDM, rgb_topic,   1);
    //message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(RDM, depth_topic, 1);
    /*typedef message_filters::sync_policies::ApproximateTime<Image,
          Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20),
                                                   rgb_image_sub,
                                                   depth_image_sub);*/

    //TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(rgb_image_sub, depth_image_sub, 10);
    //sync.registerCallback(boost::bind(&image_callback, _1, _2));

    // 循环等待回调函数
    ros::spin();

    return 0;
}
