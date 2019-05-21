#include <ros/ros.h>
#include <test_package/aircraft_controls.h>
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/Joy.h>
#include <sstream>


struct Axis
  {
    int axis;
    double max;
  };

struct
  {
    Axis x;
    Axis y;
    Axis z;
  } axes_;

test_package::aircraft_controls msg;

void imuCallback(const sensor_msgs::Imu &msg)
  {
     
  }

sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr &joy, Axis axis)
  {
    if (axis.axis == 0)
    {return 0;}
    sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
    if (axis.axis < 0)
    {
      sign = -1.0;
      axis.axis = -axis.axis;
    }
    if ((size_t) axis.axis > joy->axes.size())
    {return 0;}
    return sign * joy->axes[axis.axis - 1] * axis.max;
  }


void joyCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    msg.elevator = getAxis(joy, axes_.z)*getAxis(joy, axes_.z)*8000;
    msg.rudder = getAxis(joy, axes_.x)*2;
    //msg.ailerons = 0;
    //msg.flaps = 0;
    msg.thrust = getAxis(joy, axes_.x)*2;
  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");

  ros::NodeHandle n;

    n.param<int>("elevator", axes_.y.axis, 3);
    n.param<int>("rudder", axes_.z.axis, 2);
    n.param<int>("thrust", axes_.x.axis, 1);

    n.param<double>("elevator_max", axes_.x.max, 20000);
    n.param<double>("thrust_max", axes_.y.max, 90.0 * M_PI / 180.0);
    n.param<double>("rudder_max", axes_.z.max, 2);


  ros::Publisher kite_publisher = n.advertise<test_package::aircraft_controls>("/Kite",1000);
  ros::Subscriber kite_imu = n.subscribe("/crazyflie/imu",100,imuCallback);
  ros::Subscriber kite_joy = n.subscribe("/crazyflie/joy",100,joyCallback);
  ros::Rate loop_rate(5.0);

  while(ros::ok())
  {

	  ROS_INFO("[Kite] prints rudder:%f,  thrust:%f",msg.rudder,msg.thrust);

	  kite_publisher.publish(msg);

	  ros::spinOnce();

	  loop_rate.sleep();
  }

  return 0;
}
