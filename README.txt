Modeling and control of airborne wind energy systems

CAD:
 Contains the 3D model of the ground station


ROS_code:
 Launch file to test Crazyflie with the keyboard.
To use this launch files, you will need to install some repositories.
-Teleop_twist_keyboard: http://wiki.ros.org/teleop_twist_keyboard
-crazyflie: https://github.com/whoenig/crazyflie_ros


crazyflie-firmware:
 Contains the new fimware put in the crazyflie to control servo motors instead of DC motors,
 some pins were changed to output constanly Vcc to plug the servos to them in order to avoid
 plugging 4 connectors to the same pin.
 Other changes were done in order to control each motor independently from the others, as 
 we previously had to control all four of them in one go trough the sending of 4 parameters
 (roll, pitch, yaw, thrust).
Those changes are done in crazyflie-firmware/drivers/src/motors_f405.c and in 
crazyflie-firmware/modules/src/stabilizer.c

crazyflie_ros
 Contains nodes to send control and receive data from crazyflie. The data is sent via aerial_control 
 message. The parameters of this message can be controlled via a joystick, using the node called 
 Publish, or via the simulator from test_package. You need to change the subscription node in order to
 switch between the two.