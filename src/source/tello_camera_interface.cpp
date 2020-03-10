#include "tello_camera_interface.h"

using namespace std;

CameraInterface::CameraInterface(){
}   

CameraInterface::~CameraInterface(){
}

void CameraInterface::setUp() {
    ros::param::get("~tello_drone_id", tello_drone_id);
    ros::param::get("~tello_drone_model", tello_drone_model);
}

void CameraInterface::start(){
    //Publisher
    ros::NodeHandle n;
    camera_pub = n.advertise<sensor_msgs::Image>("sensor_measurement/camera", 1, true);

    this->cameraSocket = new VideoSocket(TELLO_CAMERA_PORT, camera_pub);
}

//Stop
void CameraInterface::stop()
{
    camera_pub.shutdown();
}

//Reset
bool CameraInterface::resetValues()
{
    return true;
}


int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "CameraInterface");

    cout<<"[ROSNODE] Starting CameraInterface"<<endl;

    //Vars
    CameraInterface camera_interface;
    camera_interface.setUp();
    camera_interface.start();
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