#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <test_package/aircraft_controls.h>


class Teleop
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;
  ros::Subscriber test_subscriber_;

  ros::Publisher velocity_publisher_;
  geometry_msgs::Twist velocity_;

  struct Axis
  {
    int axis;
    double max;
  };

  struct Button
  {
    int button;
  };

  struct
  {
    Axis x;
    Axis y;
    Axis z;
    Axis yaw;
  } axes_;

public:
  Teleop()
  {
    ros::NodeHandle params("~");

	// change the name of the node you want to subscribe to 
	// /Kite will allow you to change variables through the joystick
	// /kite_controls allows you to change them via the simulator in test_package
    test_subscriber_ = node_handle_.subscribe<test_package::aircraft_controls>("/Kite", 100, boost::bind(&Teleop::kiteCallback, this, _1));
    velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

  ~Teleop()
  {
    stop();
  }
  
void kiteCallback(const test_package::aircraft_controlsConstPtr &msg)
  {
     // Variables sent to crazyflie are:
    velocity_.linear.y = msg->rudder;
    velocity_.linear.x = msg->elevator;
    velocity_.linear.z = msg->thrust;
    velocity_publisher_.publish(velocity_);
  }

  void stop()
  {
    if(velocity_publisher_.getNumSubscribers() > 0)
    {
      velocity_ = geometry_msgs::Twist();
      velocity_publisher_.publish(velocity_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie");

  Teleop teleop;
  ros::spin();

  return 0;
}
