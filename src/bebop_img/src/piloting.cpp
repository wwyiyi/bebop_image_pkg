#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>
#include <string>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include <limits>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher pilot_pub = n.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1000);
  ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("bebop/takeoff", 1000);
  ros::Publisher land_pub = n.advertise<std_msgs::Empty>("bebop/land", 1000);

  //ros::Rate loop_rate(10);

  int count = 0;

  string str_input = "ok";

  while (str_input.compare("quit") != 0)
  {

    if(str_input.compare("takeoff") == 0)
    {
        std_msgs::Empty emp_msg;
        takeoff_pub.publish(emp_msg);
        ros::spinOnce();
        cout << "taking off...\n";
    }
    else if(str_input.compare("land") == 0)
    {
        std_msgs::Empty emp_msg;
        land_pub.publish(emp_msg);
        ros::spinOnce();
        cout << "landing...\n";
    } 
    else if(str_input.compare("0") == 0)
    {
        geometry_msgs::Twist twist;
        pilot_pub.publish(twist);
        ros::spinOnce();
    }
    else if(str_input.compare("gaz") == 0)
    {
        try
        {
          double speed;
          cout << "gaz speed:\n";
          cin >> speed;
          if(cin && speed <= 1.0 && speed >= -1.0)
          {
              geometry_msgs::Twist twist;
              twist.linear.z = speed;
              pilot_pub.publish(twist);
              ros::spinOnce();
          }
          else
          {
              cout << "Invalid number!\n";
              cin.clear(); // reset failbit
              cin.ignore(numeric_limits<streamsize>::max(), '\n');
          }
        }
        catch (int e)
        {
            cout << "An exception occurred. Exception no. " << e << "\n";
        }
    }
    else if(str_input.compare("yaw") == 0)
    {
        try
        {
          double speed;
          cout << "yaw speed:\n";
          cin >> speed;
          if(cin && speed <= 1.0 && speed >= -1.0)
          {
              geometry_msgs::Twist twist;
              twist.angular.z = speed;
              pilot_pub.publish(twist);
              ros::spinOnce();
          }
          else
          {
              cin.clear(); // reset failbit
              cin.ignore(numeric_limits<streamsize>::max(), '\n');
              cout << "Invalid number!\n";
          }
        }
        catch (int e)
        {
            cout << "An exception occurred. Exception no. " << e << "\n";
        }
    }
    else if(str_input.compare("pitch") == 0)
    {
        try
        {
          double speed;
          cout << "pitch speed:\n";
          cin >> speed;
          if(cin && speed <= 1.0 && speed >= -1.0)
          {
              geometry_msgs::Twist twist;
              twist.linear.x = speed;
              pilot_pub.publish(twist);
              ros::spinOnce();
          }
          else
          {
              cout << "Invalid number!\n";
              cin.clear(); // reset failbit
              cin.ignore(numeric_limits<streamsize>::max(), '\n');
          }
        }
        catch (int e)
        {
            cout << "An exception occurred. Exception no. " << e << "\n";
        }
    }
    else if(str_input.compare("roll") == 0)
    {
        try
        {
          double speed;
          cout << "roll speed:\n";
          cin >> speed;
          if(cin && speed <= 1.0 && speed >= -1.0)
          {
              geometry_msgs::Twist twist;
              twist.linear.y = speed;
              pilot_pub.publish(twist);
              ros::spinOnce();
          }
          else
          {
              cout << "Invalid number!\n";
              cin.clear(); // reset failbit
              cin.ignore(numeric_limits<streamsize>::max(), '\n');
          }
        }
        catch (int e)
        {
            cout << "An exception occurred. Exception no. " << e << "\n";
        }
    }
    else
    {
      /*cout << "gaz: up/down\npitch: forward/backward\nyaw: rotate\n roll: left/righ\n";
      geometry_msgs::Twist twist;
      pilot_pub.publish(twist);
      ros::spinOnce();*/
    }

    cout <<"Waiting for command...\n";
    cin >> str_input;

    //ROS_INFO("%s", msg.data.c_str());

    //chatter_pub.publish(msg);

    //loop_rate.sleep();
    //++count;
  }

  return 0;
}

