#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>

static double imgHeaderTime, ptcHeaderTime;
static bool imgReceived = false, ptcReceived = false;

cv::Mat imgMat;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_STREAM("img data received!");
    try
    {
        imgMat =  cv_bridge::toCvShare(msg, "bgr8")->image;

        imgReceived = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

    }

}

void ptcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    ROS_INFO_STREAM("ptc data received!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_collection");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImg = it.subscribe("/occam/stitched_image0", 1, imageCallback);
    ros::Subscriber subPtc = nh.subscribe("/velodyne_points", 1, ptcCallback);
    
    ros::spin();
    cv::destroyWindow("view");

}
