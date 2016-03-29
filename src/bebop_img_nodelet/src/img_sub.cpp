#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>



using namespace cv;
using namespace std;

PLUGINLIB_EXPORT_CLASS(example_pkg::MyNodeletClass, nodelet::Nodelet)

namespace example_pkg
{
    void MyNodeletClass::onInit() 
    {
        NODELET_DEBUG("Initializing nodelet...");
        ros::init(argc, argv, "image_listener");
        ros::NodeHandle nh;
        cv::namedWindow("view");
        cv::startWindowThread();

        cv::namedWindow("grayView");
        cv::startWindowThread();
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("bebop/image_raw", 1, imageCallback);
        ros::spin();
        cv::destroyWindow("view");
        cv::destroyWindow("grayView");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image); 
        Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        Mat imgBW;
        cvtColor(img, imgBW, CV_BGR2GRAY);
        threshold(imgBW, imgBW, 128, 255, THRESH_BINARY);
        cv::imshow("grayView", imgBW);
        cv::waitKey(30);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }
}

