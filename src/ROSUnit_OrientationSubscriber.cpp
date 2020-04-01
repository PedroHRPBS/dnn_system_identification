#include "ROSUnit_OrientationSubscriber.hpp"
ROSUnit_OrientationSubscriber* ROSUnit_OrientationSubscriber::_instance_ptr = NULL;
Vector3DMessage ROSUnit_OrientationSubscriber::orientation_msg;

ROSUnit_OrientationSubscriber::ROSUnit_OrientationSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)  {

    _sub_orientation = t_main_handler.subscribe("uav_control/uav_orientation", 1, callbackOrientation);
    _instance_ptr = this;

}

ROSUnit_OrientationSubscriber::~ROSUnit_OrientationSubscriber() {

}

void ROSUnit_OrientationSubscriber::callbackOrientation(const geometry_msgs::Point& msg){

    Vector3D<float> tmp;
    tmp.x = msg.x;
    tmp.y = msg.y;
    tmp.z = msg.z;

    orientation_msg.setVector3DMessage(tmp);

    _instance_ptr->emitMsgUnicastDefault((DataMessage*) &orientation_msg); 

}

void ROSUnit_OrientationSubscriber::receiveMsgData(DataMessage* t_msg){

}