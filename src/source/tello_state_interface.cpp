#include "tello_state_interface.h"

using namespace std;

StateInterface::StateInterface(){
}   

StateInterface::~StateInterface(){
}

void StateInterface::setUp() {
    this->stateSocket = new StateSocket(TELLO_STATE_PORT);

    ros::param::get("~tello_drone_id", tello_drone_id);
    ros::param::get("~tello_drone_model", tello_drone_model);
}

void StateInterface::start(){
    //Publisher
    ros::NodeHandle n;
    rotation_pub = n.advertise<geometry_msgs::Vector3Stamped>("sensor_measurement/rotation_angles", 1, true);

    speed_pub = n.advertise<geometry_msgs::TwistStamped>("sensor_measurement/speed", 1, true);

    accel_pub = n.advertise<geometry_msgs::AccelStamped>("sensor_measurement/accel", 1, true);

    imu_pub = n.advertise<sensor_msgs::Imu>("sensor_measurement/imu", 1, true);

    battery_pub = n.advertise<sensor_msgs::BatteryState>("sensor_measurement/battery_state", 1, true);

    temperature_pub = n.advertise<sensor_msgs::Temperature>("sensor_measurement/temperature", 1, true);

    sea_level_pub = n.advertise<geometry_msgs::PointStamped>("sensor_measurement/sea_level", 1, true);
    altitude_pub = n.advertise<geometry_msgs::PointStamped>("sensor_measurement/altitude", 1, true);

    this->state_thread = new std::thread(&StateInterface::get_state, this);
}

void StateInterface::get_state()
{
    ros::Time current_timestamp = ros::Time::now();
    ros::Time prev_timestamp = ros::Time::now();
    while (true)
    {
        std::string state = this->stateSocket->listen_once();


        if (!state.empty())
        {
            std::istringstream iss(state);

            std::string token;
            map<std::string, float> state_map;
            while (std::getline(iss, token, ';'))
            {
                if (!token.empty())
                {
                    std::string key;
                    std::string value_str;
                    std::istringstream iss_token(token);
                    std::getline(iss_token, key, ':');
                    std::getline(iss_token, value_str, ':');

                    state_map.insert(pair<std::string, float>(key, strtof(value_str.c_str(), 0)));
                }
            }

            current_timestamp = ros::Time::now();

            // ROTATION
            float pitch = state_map.find("pitch")->second;
            float roll = state_map.find("roll")->second;
            float yaw = state_map.find("yaw")->second;
            rotation_msg.header.stamp = current_timestamp;
            rotation_msg.vector.x = pitch;
            rotation_msg.vector.y = roll;
            rotation_msg.vector.z = yaw;
            rotation_pub.publish(rotation_msg);

            // VELOCITY
            float vgx = state_map.find("vgx")->second;
            float vgy = state_map.find("vgy")->second;
            float vgz = state_map.find("vgz")->second;

            float roll_rad = roll * M_PI / 180.0f;
            float pitch_rad = -pitch * M_PI / 180.0f;
            float yaw_rad = -yaw * M_PI / 180.0f;

            //current_time_ = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);
            double diffTime = (current_timestamp - prev_timestamp).nsec / 1E9;

            double dRoll = 0;
            double dPitch = 0;
            double dYaw = 0;
            if (diffTime != 0)
            {
                dRoll = roll_rad / diffTime;
                dPitch = pitch_rad / diffTime;
                dYaw = yaw_rad / diffTime;
            }

            float x = (1 * dRoll) - (sin(pitch_rad) * dYaw);
            float y = (cos(roll_rad) * dPitch) + (cos(pitch_rad)*sin(roll_rad) * dYaw);
            float z = (-sin(roll_rad) * dPitch) + (cos(roll_rad)*cos(pitch_rad) * dYaw);
            
            speed_msg.header.stamp = current_timestamp;
            speed_msg.twist.linear.x = vgx;
            speed_msg.twist.linear.y = vgy;
            speed_msg.twist.linear.z = vgz;
            speed_msg.twist.angular.x = x;
            speed_msg.twist.angular.y = y;
            speed_msg.twist.angular.z = z;
            speed_pub.publish(speed_msg);

            // ACCELERATION
            float agx = state_map.find("agx")->second;
            float agy = state_map.find("agy")->second;
            float agz = state_map.find("agz")->second;
            accel_msg.accel.linear.x = agx / -100; //pos delante
            accel_msg.accel.linear.y = agy / 100; //pos izq
            accel_msg.accel.linear.z = agz / -100;
            //accel_pub.publish(accel_msg);

            // BATTERY
            float battery = state_map.find("bat")->second;
            battery_msg.header.stamp = current_timestamp;
            battery_msg.voltage = 3.8;
            battery_msg.capacity = 1.1;
            battery_msg.percentage = battery;
            battery_pub.publish(battery_msg);

            // IMU
            imu_msg.header.stamp = current_timestamp;
            imu_msg.angular_velocity.x = x;
            imu_msg.angular_velocity.y = y;
            imu_msg.angular_velocity.z = z;
            imu_msg.linear_acceleration.x = agx / -100;
            imu_msg.linear_acceleration.y = agy / 100;
            imu_msg.linear_acceleration.z = agz / -100;

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll_rad, pitch_rad, yaw_rad);

            imu_msg.orientation.x = quaternion.getX();
            imu_msg.orientation.y = quaternion.getY();
            imu_msg.orientation.z = quaternion.getZ();
            imu_msg.orientation.w = quaternion.getW();

            imu_pub.publish(imu_msg);

            // TEMPERATURE
            float templ = state_map.find("templ")->second;
            float temph = state_map.find("temph")->second;
            float mean = (templ + temph) / 2;
            float variance = pow(temph - mean, 2.0);
            temperature_msg.header.stamp = current_timestamp;
            temperature_msg.temperature = mean;
            temperature_msg.variance = variance;
            temperature_pub.publish(temperature_msg);

            // SEA LEVEL
            float sea_level = state_map.find("baro")->second;
            sea_level_msg.header.stamp = current_timestamp;
            sea_level_msg.point.z = sea_level;
            sea_level_pub.publish(sea_level_msg);

            // ALTITUDE
            float altitude = state_map.find("h")->second;
            altitude_msg.header.stamp = current_timestamp;
            altitude_msg.point.z = altitude;
            altitude_pub.publish(altitude_msg);

            prev_timestamp = current_timestamp;
        }
    }
}

//Stop
void StateInterface::stop()
{
    rotation_pub.shutdown();
    speed_pub.shutdown();
    accel_pub.shutdown();
    battery_pub.shutdown();
    temperature_pub.shutdown();
    imu_pub.shutdown();
    sea_level_pub.shutdown();
    altitude_pub.shutdown();
}

//Reset
bool StateInterface::resetValues()
{
    return true;
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "StateInterface");

    cout<<"[ROSNODE] Starting StateInterface"<<endl;

    //Vars
    StateInterface state_interface;
    state_interface.setUp();
    state_interface.start();
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