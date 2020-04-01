#include "ROSUnit_ControlOutputSubscriber.hpp"

ROSUnit_ControlOutputSubscriber* ROSUnit_ControlOutputSubscriber::_instance_ptr = NULL;
VectorDoubleMsg ROSUnit_ControlOutputSubscriber::_controloutput_msg;

ROSUnit_ControlOutputSubscriber::ROSUnit_ControlOutputSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {

    _sub_controloutput = t_main_handler.subscribe("control_system_output", 2, callbackControlOutput);
    _instance_ptr = this;
    std::cout << "CREATE: " << std::endl;

}

ROSUnit_ControlOutputSubscriber::~ROSUnit_ControlOutputSubscriber() {

}

void ROSUnit_ControlOutputSubscriber::callbackControlOutput(const std_msgs::Float64MultiArray& msg){

    std::cout << "callback: " << std::endl;
    _controloutput_msg.data = msg.data;
    _instance_ptr->emitMsgUnicastDefault((DataMessage*) &_controloutput_msg);
}

void ROSUnit_ControlOutputSubscriber::receiveMsgData(DataMessage* t_msg){

}