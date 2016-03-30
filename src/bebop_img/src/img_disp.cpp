#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "QRRegion.cpp"
#include <cmath>
#include "geometry_msgs/Twist.h"


using namespace cv;
using namespace std;

//global variables
ros::Publisher pilot_pub;

bool atCenter(float height, float mean, float center) 
{
    if (abs(mean - center)/height <= 0.2)
    {
      return true;
    }
    else
    {
      return false;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //cv::imshow("Image", cv_bridge::toCvShare(msg, "bgr8")->image); 

    /*
    Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    Mat imgBW;
    cvtColor(img, imgBW, CV_BGR2GRAY);
    threshold(imgBW, imgBW, 128, 255, THRESH_BINARY);
    cv::imshow("grayView", imgBW);*/
    Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image; 



    vector<Point2f> corners = findRegion(img);

    /*
    if(corners.size() == 4)
    {
        float heightLeft = corners[3].y - corners[0].y;
        float heightRight = corners[2].y - corners[1].y;
        float centerLeft = (corners[3].y + corners[0].y)/2;
        float centerRight = (corners[2].y + corners[1].y)/2;
        float centerHeight = 184.0;

        if(!atCenter(heightLeft, centerLeft, centerHeight) || !atCenter(heightRight, centerRight, centerHeight))
        {
            geometry_msgs::Twist twist;
            if(centerLeft <= centerHeight)
            {
                twist.linear.z = 0.2;
                cout << "lower: " << centerLeft << ", " << centerRight << "\n";
            }
            else
            {
                twist.linear.z = -0.2;
                cout << "higher: " << centerLeft << ", " << centerRight << "\n";
            }
            pilot_pub.publish(twist);
            ros::spinOnce();
        }

    }*/

    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("Image");
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("bebop/image_raw", 1, imageCallback);
  pilot_pub = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1000);
  ros::spin();
  cv::destroyWindow("Image");
}