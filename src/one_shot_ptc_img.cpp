#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <string>
#include <cstddef>
#include <ctime>

#include <boost/filesystem.hpp> 

static double imgHeaderTime, ptcHeaderTime;
static bool imgReceived = false, ptcReceived = false;

static cv::Mat imgMat;
static sensor_msgs::PointCloud2 ptcData;
static double shotNum = 0;

bool file_exists (const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_STREAM("img data received!");
    try
    {
        imgMat =  cv_bridge::toCvShare(msg, "bgr8")->image;

        imgReceived = true;
        imgHeaderTime = msg->header.stamp.toSec();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

    }

}

void ptcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    ROS_INFO_STREAM("ptc data received!");
    if (!msg->fields.empty())
    {
        ptcData = *msg;
        ptcReceived = true;
        ptcHeaderTime = msg->header.stamp.toSec();
    }
}

int saveData()
{
    time_t t = time(0); // get time now
    struct tm *now = localtime(&t);
    std::string imgFileName;
    std::string ptcFileName;

    while (1)
    {
        std::ostringstream ss;
        ss << std::setw(4) << std::setfill('0') << shotNum++;
        std::string path = boost::filesystem::current_path().string();
        std::size_t location = path.find_last_of("/\\");
        imgFileName = std::to_string(now->tm_year+1900) + "-" + std::to_string(now->tm_mon+1) + "-"
                              + std::to_string(now->tm_mday) + "-" + path.substr(location+1) + "-" + ss.str() + ".png";
        ptcFileName = std::to_string(now->tm_year+1900) + "-" + std::to_string(now->tm_mon+1) + "-"
                              + std::to_string(now->tm_mday) + "-" + path.substr(location+1) + "-" + ss.str() + ".pcd";

        if (!file_exists(imgFileName) && !file_exists(ptcFileName))
        {
            break;
        }
    }

    ROS_INFO_STREAM(imgFileName << " saved");
    ROS_INFO_STREAM(ptcFileName << " saved");
    imwrite(imgFileName, imgMat);
    pcl::io::savePCDFile(ptcFileName, ptcData);

//     pcl::io::savePCDFile("pointcloud.pcd", ptcData);
//     imwrite("img.png", imgMat);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_collection");
    ros::NodeHandle nh;
//     cv::namedWindow("view");
//     cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImg = it.subscribe("/occam/stitched_image0", 1, imageCallback);
    ros::Subscriber subPtc = nh.subscribe("/velodyne_points", 1, ptcCallback);
   
    while (ros::ok)
    {
        ros::spinOnce();
        if (imgReceived && ptcReceived && fabs(imgHeaderTime - ptcHeaderTime) < 0.001)
        {
            ROS_WARN_STREAM("time interval less than 0.001s " << fabs(imgHeaderTime - ptcHeaderTime) );
            saveData();
            break;
        }
    }
//     cv::destroyWindow("view");

}
