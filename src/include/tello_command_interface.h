#include <iostream>

//// ROS  ///////
#include "ros/ros.h"

#include "socket_tello.h"

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tello_controller/command.h"

#define ALIVE_INTERVAL 1 //alive interval in seconds

class TelloSocketClient;

class CommandInterface
{
//Constructors and destructors
public:
    CommandInterface();
    ~CommandInterface();

    void setUp();
    void start();
    void stop();
protected:
    bool resetValues();
private:

    void stay_alive();
    bool alive;
    std::thread* alive_thread;

    std::string drone_namespace;   
    std::string tello_drone_model;
    int tello_drone_id;
    int drone_id;
    CommandSocket* commandSocket;

protected:
    ros::Publisher command_pub;
    ros::Subscriber command_sub;
    void commandCallback(const std_msgs::String &msg);

    // RC COMMAND
    message_filters::Subscriber<geometry_msgs::PoseStamped> roll_pitch_sub;
    message_filters::Subscriber<geometry_msgs::TwistStamped> altitude_yaw_sub;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> sync_policy_velocity;
    message_filters::Synchronizer<sync_policy_velocity> sync;
    void rcCallback(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& twist);

    ros::Subscriber command_enum_sub;
    void commandEnumCallback(const tello_controller::command::ConstPtr& msg);

    std_msgs::String command_msg;
};
