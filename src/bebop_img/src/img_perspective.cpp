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
int position;
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
        vector<Point2f> corners = findRegion(img);

        if (state == -1 && corners.size() == 4)
        {
            //find the center 
            state = 0;
        }

        bool stable = false;

        if(corners.size() == 4)
        {
            float heightLeft = corners[3].y - corners[0].y;
            float heightRight = corners[2].y - corners[1].y;
            float centerLeft = (corners[3].y + corners[0].y)/2;
            float centerRight = (corners[2].y + corners[1].y)/2;

            float widthUpper = corners[1].x - corners[0].x;
            float widthBottom = corners[2].x - corners[3].x;
            float centerUpper = (corners[1].x + corners[0].x)/2;
            float centerBottom = (corners[2].x + corners[3].x)/2;

            float centerHeight = 184.0;
            float centerWidth = 320.0;

            if(state == 1) 
            {
                switch(position)
                {
                    case 1:
                    {
                        centerHeight = 240;
                        centerWidth = 320;
                        break;
                    }
                    case 2:
                    {
                        centerHeight = 220;
                        centerWidth = 420;
                        break;
                    }
                    case 3:
                    {
                        centerHeight = 220;
                        centerWidth = 220;
                        break;
                    }
                    case 4:
                    {
                        centerHeight = 130;
                        centerWidth = 420;
                        break;
                    }
                    case 5:
                    {
                        centerHeight = 130;
                        centerWidth = 220;
                        break;
                    }
                }
            }


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
                stable = false;
            }
            else
            {
                //change distance
                if((heightRight + heightLeft)/2 >= 115)
                {
                  //move forward
                    geometry_msgs::Twist twist;
                    twist.linear.x = -0.05;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "move forward\n";
                }
                else if((heightRight + heightLeft)/2 <= 90)
                {
                  //move forward
                    geometry_msgs::Twist twist;
                    twist.linear.x = 0.05;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "move forward\n";
                }
                else
                {
                    stable = true;
                }
            }

            //check rotate or horizontal move
            if(state == 0 && (!atCenter((heightLeft + heightRight)/2, centerUpper, centerWidth, 0.5) || !atCenter((heightLeft + heightRight)/2, centerBottom, centerWidth, 0.5)))
            {
                stable = false;
                if(centerUpper < centerWidth)
                {
                  //benchmark in left 
                    if(heightLeft > heightRight*1.1) 
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
                    if(heightRight > heightLeft*1.1) 
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
                    stable = false;
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
                    stable = false;
                }
            }

            if(state == 1 && (!atCenter((heightLeft + heightRight)/2, centerUpper, centerWidth, 0.5) || !atCenter((heightLeft + heightRight)/2, centerBottom, centerWidth, 0.5)))
            {
                stable = false;
                if(centerUpper < centerWidth)
                {
                  //benchmark in left 
                   //move left
                    geometry_msgs::Twist twist;
                    twist.linear.y = 0.1;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "not center: move left\n";
                    cout << centerUpper << "\n";
                }
                else
                {
                    //benchmark in right
                    //move right
                    geometry_msgs::Twist twist;
                    twist.linear.y = -0.1;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "not center: move right\n";
                    cout << centerUpper << "\n";
                
                }
            }

            if(state == 2 && (!atCenter((heightLeft + heightRight)/2, centerUpper, centerWidth, 0.5) || !atCenter((heightLeft + heightRight)/2, centerBottom, centerWidth, 0.5)))
            {
                stable = false;
                if(centerUpper < centerWidth)
                {
                  //benchmark in left 
                    geometry_msgs::Twist twist;
                    twist.angular.z = 0.3;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "not center: rotate counterclockwise\n";
                    cout << centerUpper << "\n";
                }
                else
                {
                    //benchmark in right
                    geometry_msgs::Twist twist;
                    twist.angular.z = -0.3;
                    pilot_pub.publish(twist);
                    ros::spinOnce();
                    cout << "not center: rotate clockwise\n";
                    cout << centerUpper << "\n";
                }
            }
        }

        if (stable)
        {
            state = 1;
        }

        if(state >= 2)
        {
            state = 2;
            cout << "ready for photo-taking" <<endl;
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

    std::vector<std::string> args;
    std::copy(argv + 1, argv + argc, std::back_inserter(args));

    if (argc == 2)
    {
    
        if (args[0].compare("top") == 0)
        {
            position = 1;
        }
        else if (args[0].compare("top-left") == 0)
        {
            position = 2;
        }
        else if (args[0].compare("top-right") == 0)
        {
            position = 3;
        }
        else if (args[0].compare("bottom-left") == 0)
        {
            position = 4;
        }
        else if (args[0].compare("bottom-right") == 0)
        {
            position = 5;
        }
        else 
        {
            position = 0;
            cout << "Invalid argument, position set to center!" <<endl;
        }
    }
    else 
    {
        position = 0;
        cout << "Position set to center!" << endl;
    }
    state = -1;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("bebop/image_raw", 1, imageCallback);
    pilot_pub = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1000);
    ros::spin();
    cv::destroyWindow("Image");
    cv::destroyWindow("Traces");
}

