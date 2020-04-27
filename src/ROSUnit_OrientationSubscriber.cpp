#include "ROSUnit_OrientationSubscriber.hpp"
#include "common_srv/Timer.hpp"

ROSUnit_OrientationSubscriber* ROSUnit_OrientationSubscriber::_instance_ptr = NULL;
Vector3DMessage ROSUnit_OrientationSubscriber::orientation_msg;

Timer tempoROS;

ROSUnit_OrientationSubscriber::ROSUnit_OrientationSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)  {

    //TODO check queue size when using live data
    _sub_orientation = t_main_handler.subscribe("roll_provider", 1, callbackOrientation);
    _instance_ptr = this;

}

ROSUnit_OrientationSubscriber::~ROSUnit_OrientationSubscriber() {

}

void ROSUnit_OrientationSubscriber::callbackOrientation(const geometry_msgs::PointStamped& msg){

    //std::cout << "Tempo callbackOrientation: " << tempoROS.tockMicroSeconds() << "\n";

    Vector3D<float> tmp;
    tmp.x = msg.point.x;
    tmp.y = msg.point.y;
    tmp.z = msg.point.z;
    
    orientation_msg.setVector3DMessage(tmp);

    _instance_ptr->emitMsgUnicastDefault((DataMessage*) &orientation_msg); 
    //tempoROS.tick();

}

void ROSUnit_OrientationSubscriber::receiveMsgData(DataMessage* t_msg){

}