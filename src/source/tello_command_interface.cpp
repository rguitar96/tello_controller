#include "tello_command_interface.h"

using namespace std;

CommandInterface::CommandInterface() :
    sync (sync_policy_velocity(1))
{
}

CommandInterface::~CommandInterface()
{
}

void CommandInterface::setUp()
{
    this->commandSocket = new CommandSocket(TELLO_CLIENT_ADDRESS, TELLO_COMMAND_PORT, PC_COMMAND_PORT);
    
    cout<<"[ROSNODE] Command setup"<<endl;

    ros::param::get("~tello_drone_id", tello_drone_id);
    ros::param::get("~tello_drone_model", tello_drone_model);
}

void CommandInterface::start()
{
    cout<<"[ROSNODE] Command sart"<<endl;

    this->commandSocket->send_command("command");
    usleep(200);
    this->commandSocket->send_command("streamon");

    ros::NodeHandle n;
    command_pub = n.advertise<std_msgs::String>("command", 1, true);
    command_sub = n.subscribe("command", 1, &CommandInterface::commandCallback, this);

    // STANDARD
    roll_pitch_sub.subscribe(n, "actuator_command/roll_pitch", 1);
    altitude_yaw_sub.subscribe(n, "actuator_command/altitude_rate_yaw_rate", 1);
    sync.connectInput(roll_pitch_sub, altitude_yaw_sub);
    sync.registerCallback(&CommandInterface::rcCallback, this);

    command_enum_sub = n.subscribe("command/high_level", 1, &CommandInterface::commandEnumCallback, this);

    this->alive_thread = new std::thread(&CommandInterface::stay_alive, this);
}

void CommandInterface::stay_alive()
{
    this->alive = true;
    while (this->alive)
    {
        sleep(ALIVE_INTERVAL);
        command_msg.data = "rc 0 0 0 0";
        command_pub.publish(command_msg);
    }
}

//Stop
void CommandInterface::stop()
{
    command_pub.shutdown();
    command_sub.shutdown();
}

//Reset
bool CommandInterface::resetValues()
{
    return true;
}

void CommandInterface::commandCallback(const std_msgs::String &msg)
{
    cout << "Command: " << msg.data << endl;
    this->commandSocket->send_command(msg.data.c_str());
    command_msg.data = msg.data;
}

void CommandInterface::rcCallback(const geometry_msgs::PoseStamped& pose_stamped, const geometry_msgs::TwistStamped& twist_stamped)
{
    float roll, pitch, yaw, altitude, t0, t1, t2, x, y, z, w;

    x = pose_stamped.pose.orientation.x;
    y = pose_stamped.pose.orientation.y;
    z = pose_stamped.pose.orientation.z;
    w = pose_stamped.pose.orientation.w;

    //roll/pitch: PoseStamped.Pose.Quaternion
    t0 = 2.0 * (w * x + y * z);
    t1 = 1.0 - 2.0 * (x * x + y * y);
    roll = atan2(t0, t1);
    t2 = 2.0 * (w * y - z * x);
    if (t2 > 1) t2 = 1;
    if (t2 < -1) t2 = -1;
    pitch = asin(t2);

    //yaw: TwistStamped.Twist.angular
    yaw = twist_stamped.twist.angular.z;
    //altitude: TwistStamped.Twist.linear
    altitude = twist_stamped.twist.linear.z;


    std::ostringstream rc;
    rc << "rc " << static_cast<int>(round(roll * 100))
    << " " << static_cast<int>(round(pitch * 100))
    << " " << static_cast<int>(round(altitude * 100))
    << " " << static_cast<int>(round(yaw * 100));

    this->commandSocket->send_command(rc.str().c_str());
    cout << "Command: " << rc.str() << endl;
}

void CommandInterface::commandEnumCallback(const tello_controller::command::ConstPtr& msg)
{
    std::string response;
    switch(msg->command)
    {
    case tello_controller::command::TAKE_OFF:
        cout << "[commandEnumCallback] Taking off" << endl;
        this->commandSocket->send_command("takeoff");
        break;
    case tello_controller::command::LAND:
        cout << "[commandEnumCallback] Landing" << response << endl;
        this->commandSocket->send_command("land");
        break;
    case tello_controller::command::HOVER:
        cout << "[commandEnumCallback] Hovering" << response << endl;
        this->commandSocket->send_command("rc 0 0 0 0");
        break;
    case tello_controller::command::EMERGENCY_STOP:
        cout << "[commandEnumCallback] Emergency" << response << endl;
        this->commandSocket->send_command("emergency");
        break;
    default:
        break;
    }

    return;
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "CommandInterface");

    cout<<"[ROSNODE] Starting CommandInterface"<<endl;

    //Vars
    CommandInterface command_interface;
    command_interface.setUp();
    command_interface.start();
    try
    {
        //Read messages
        ros::spin();
        return 1;
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}