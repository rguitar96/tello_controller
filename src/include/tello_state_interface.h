#include <iostream>

//// ROS  ///////
#include "ros/ros.h"

#include "socket_tello.h"

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <geometry_msgs/PointStamped.h>

class StateInterface
{
//Constructors and destructors
public:
    StateInterface();
    ~StateInterface();

    void setUp();
    void start();
    void stop();
protected:
    bool resetValues();
private:
    void get_state();
    std::thread* state_thread;

    std::string drone_namespace;   
    std::string tello_drone_model;
    int tello_drone_id;
    int drone_id;
    //TelloSocketServer* stateSocket;
    StateSocket* stateSocket;
    //Publisher
protected:
    ros::Publisher rotation_pub;
    ros::Publisher speed_pub;
    ros::Publisher accel_pub;
    ros::Publisher imu_pub;
    ros::Publisher battery_pub;
    ros::Publisher temperature_pub;
    ros::Publisher sea_level_pub;
    ros::Publisher altitude_pub;

    geometry_msgs::Vector3Stamped rotation_msg;
    geometry_msgs::TwistStamped speed_msg;
    geometry_msgs::AccelStamped accel_msg;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::BatteryState battery_msg;
    sensor_msgs::Temperature temperature_msg;
    geometry_msgs::PointStamped sea_level_msg;
    geometry_msgs::PointStamped altitude_msg;
};
