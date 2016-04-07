#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "SquareRegion.cpp"
#include <cmath>
#include "geometry_msgs/Twist.h"


using namespace cv;
using namespace std;

//global variables
ros::Publisher pilot_pub;

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
        vector<Point2f> corners = squareRegion(img);


        if(corners.size() == 4)
        {
            
            float max = -1.0;
            float min = 10000000.0;
            int index = -1;
            int minIndex = -1;
            vector<Point2f> ordered(4);
            for (int i = 0; i < 4; i++) 
            {
            	if (corners[i].x * corners[i].x + corners[i].y * corners[i].y >= max)
            	{
            		max = corners[i].x * corners[i].x + corners[i].y * corners[i].y;
            		index = i;
            	}
            	if (corners[i].x * corners[i].x + corners[i].y * corners[i].y <= min)
            	{
            		min = corners[i].x * corners[i].x + corners[i].y * corners[i].y;
            		minIndex = i;
            	}
            }
            ordered[0] = corners[minIndex];
            ordered[2] = corners[index];

            for (int i = 0; i < 4; i++)
            {
            	if (i != index)
            	{
            		if (corners[i].x <= (corners[index].x + corners[minIndex].x)/2 )
            			ordered[3] = corners[i];
            		else
            			ordered[1] = corners[i];

            	}
            	

            }
            corners = ordered;

            //cout << "Largest: " << index <<endl;

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

            cout << "top: " << centerUpper << "bottom: " << centerBottom << "left: " << centerLeft << "right: " << centerRight <<endl;

            //check height
            if(!atCenter(heightLeft, centerLeft, centerHeight, 0.3) || !atCenter(heightRight, centerRight, centerHeight, 0.3))
            {
                geometry_msgs::Twist twist;
                if(!atCenter(heightLeft, centerLeft, centerHeight, 0.5) || !atCenter(heightRight, centerRight, centerHeight, 0.5))
                {
                    if(centerLeft <= centerHeight)
                    {
                        twist.linear.z = 0.3;
                        cout << "lower: " << centerLeft << ", " << centerRight << "\n";
                    }
                    else
                    {
                        twist.linear.z = -0.3;
                        cout << "higher: " << centerLeft << ", " << centerRight << "\n";
                    }
                }
                else
                {
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
                }
                pilot_pub.publish(twist);
                ros::spinOnce();
            }
            //else
            //{
                //change distance
                if((heightLeft + heightRight)/2 >= 160) 
                {
                  //move backward
                    geometry_msgs::Twist twist;
                    twist.linear.x = -0.2;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "move backward\n";
                }
                else if((heightRight + heightLeft)/2 >= 140)
                {
                  //move forward
                    geometry_msgs::Twist twist;
                    twist.linear.x = -0.1;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "move forward\n";
                }
                else if((heightRight + heightLeft)/2 <= 60)
                {
                  //move forward
                    geometry_msgs::Twist twist;
                    twist.linear.x = 0.2;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "move forward\n";
                }
                else if((heightRight + heightLeft)/2 <= 80)
                {
                  //move forward
                    geometry_msgs::Twist twist;
                    twist.linear.x = 0.1;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "move forward\n";
                }
            //}

            //check rotate or horizontal move
            if(!atCenter((heightLeft + heightRight)/2, centerUpper, centerWidth, 0.5) || !atCenter((heightLeft + heightRight)/2, centerBottom, centerWidth, 0.5))
            {
                if(centerUpper < centerWidth)
                {
                  //benchmark in left 
                    if(heightLeft > heightRight*1.15) 
                    {
                        //rotate counterclockwise
                        geometry_msgs::Twist twist;
                        twist.angular.z = 0.4;
                        pilot_pub.publish(twist);
                        ros::spinOnce();
                        cout << "not center: rotate counterclockwise\n";
                        cout << centerUpper << "\n";
                    }
                    else if(heightLeft > heightRight*1.05) 
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
                    if(heightRight > heightLeft*1.15) 
                    {
                        //rotate clockwise
                        geometry_msgs::Twist twist;
                        twist.angular.z = -0.4;
                        pilot_pub.publish(twist);
                        ros::spinOnce();
                        cout << "not center: rotate clockwise\n";
                        cout << centerUpper << "\n";
                    }
                    else if(heightRight > heightLeft*1.05) 
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
                    geometry_msgs::Twist twist;
                    twist.linear.y = 0.1;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "at center: move left\n";
                    cout << centerUpper << "\n";
                }
            }

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
    ros::spin();
    cv::destroyWindow("Image");
    cv::destroyWindow("Traces");
}

