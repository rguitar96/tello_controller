#include <iostream>

//// ROS  ///////
#include "ros/ros.h"

#include "socket_tello.h"

class CameraInterface
{
//Constructors and destructors
public:
    CameraInterface();
    ~CameraInterface();

    void setUp();
    void start();
    void stop();
protected:
    bool resetValues();
private:
    void get_camera();
    std::thread* camera_thread;

    std::string drone_namespace;   
    std::string tello_drone_model;
    int tello_drone_id;
    int drone_id;

    VideoSocket* cameraSocket;

protected:
    ros::Publisher camera_pub;
};
