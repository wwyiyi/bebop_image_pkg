#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "SquareRegion.cpp"
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "hull_demo.cpp"





using namespace cv;
using namespace std;

//global variables
ros::Publisher pilot_pub;
int state;

bool atCenter(float height, float mean, float center, float range) 
{
    if (abs(mean - center)/height <= range)
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
        /*cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image); 
        Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        Mat imgBW;
        cvtColor(img, imgBW, CV_BGR2GRAY);
        threshold(imgBW, imgBW, 128, 255, THRESH_BINARY);
        cv::imshow("grayView", imgBW);*/
        Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        bool stable = false;
        
        if (state == 0)
        {
            vector<Point2f> corners = squareRegion(img);

            if(corners.size() == 4)
            {
                float heightLeft = corners[3].y - corners[0].y;
                float heightRight = corners[2].y - corners[1].y;
                float centerLeft = (corners[3].y + corners[0].y)/2;
                float centerRight = (corners[2].y + corners[1].y)/2;
                float centerHeight = 184.0;

                float widthUpper = corners[1].x - corners[0].x;
                float widthBottom = corners[2].x - corners[3].x;
                float centerUpper = (corners[1].x + corners[0].x)/2;
                float centerBottom = (corners[2].x + corners[3].x)/2;
                float centerWidth = 320.0;

                //check height
                if(!atCenter(heightLeft, centerLeft, centerHeight, 0.3) || !atCenter(heightRight, centerRight, centerHeight, 0.3))
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
                else
                {
                    stable = true;
                    //change distance
                    if((heightLeft + heightRight)/2 >= 200) 
                    {
                      //move backward
                        geometry_msgs::Twist twist;
                        twist.linear.x = -0.1;
                        pilot_pub.publish(twist);
                        ros::spinOnce();
                        cout << "move backward\n";
                        stable = false;
                    }
                    else if((heightRight + heightLeft)/2 <= 100)
                    {
                      //move forward
                        geometry_msgs::Twist twist;
                        twist.linear.x = 0.1;
                        pilot_pub.publish(twist);
                        ros::spinOnce();
                        cout << "move forward\n";
                        stable = false;
                    }
                }

                //check rotate or horizontal move
                if(!atCenter((heightLeft + heightRight)/2, centerUpper, centerWidth, 0.5) || !atCenter((heightLeft + heightRight)/2, centerBottom, centerWidth, 0.5))
                {
                    stable = false;
                    if(centerUpper < centerWidth)
                    {
                      //benchmark in left 
                        if(heightLeft > heightRight*1.05) 
                        {
                            //rotate counterclockwise
                            geometry_msgs::Twist twist;
                            twist.angular.z = 0.3;
                            pilot_pub.publish(twist);
                            ros::spinOnce();
                            cout << "not center: rotate counterclockwise\n";
                            cout << centerUpper << "\n";
                        }
                        else 
                        {
                            //move left
                            geometry_msgs::Twist twist;
                            twist.linear.y = 0.1;
                            pilot_pub.publish(twist);
                            ros::spinOnce();
                            cout << "not center: move left\n";
                            cout << centerUpper << "\n";
                        }
                    }
                    else
                    {
                        //benchmark in right
                        if(heightRight > heightLeft*1.05) 
                        {
                            //rotate clockwise
                            geometry_msgs::Twist twist;
                            twist.angular.z = -0.3;
                            pilot_pub.publish(twist);
                            ros::spinOnce();
                            cout << "not center: rotate clockwise\n";
                            cout << centerUpper << "\n";
                        }
                        else
                        {
                            //move right
                            geometry_msgs::Twist twist;
                            twist.linear.y = -0.1;
                            pilot_pub.publish(twist);
                            ros::spinOnce();
                            cout << "not center: move right\n";
                            cout << centerUpper << "\n";
                        }
                    }
                }
                else
                {
                    //move case
                    if(heightLeft > heightRight*1.2)
                    {
                        //move right
                        stable = false;
                        geometry_msgs::Twist twist;
                        twist.linear.y = -0.1;
                        pilot_pub.publish(twist);
                        ros::spinOnce();
                        cout << "at center: move right\n";
                        cout << centerUpper << "\n";
                    }
                    else if(heightRight > heightLeft*1.2)
                    {
                        //move left
                        stable = false;
                        geometry_msgs::Twist twist;
                        twist.linear.y = 0.1;
                        pilot_pub.publish(twist);
                        ros::spinOnce();
                        cout << "at center: move left\n";
                        cout << centerUpper << "\n";
                    }
                }

            }
        }
        else if (state == 1)
        {
            Point center = findPalm(img);
            if (center.x > 0)
            {
                if(abs(center.x - 320) > 40)
                {
                    if(center.x > 320)
                    {
                        //move left
                        cout << "move left" <<endl;                    }
                    else 
                    {
                        //move right
                        cout << "move right" <<endl;
                    }
                }
                if(abs(center.y -184 ) > 40)
                {
                    if (center.y > 184)
                    {
                        //go up
                        cout << "go up" <<endl;
                    }
                    else
                    {
                        //go down
                        cout << "go down" <<endl;
                    }
                }
            }
        }

        if (stable)
        {
            state = 1;
        }



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

    cv::namedWindow("Traces");
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("bebop/image_raw", 1, imageCallback);
    pilot_pub = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1000);

    state = 1;
    ros::spin();
    cv::destroyWindow("Image");
    cv::destroyWindow("Traces");
}

