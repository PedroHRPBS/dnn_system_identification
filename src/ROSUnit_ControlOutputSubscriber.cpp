#include "ROSUnit_ControlOutputSubscriber.hpp"

ROSUnit_ControlOutputSubscriber* ROSUnit_ControlOutputSubscriber::_instance_ptr = NULL;
VectorDoubleMsg ROSUnit_ControlOutputSubscriber::_controloutput_msg;

ROSUnit_ControlOutputSubscriber::ROSUnit_ControlOutputSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {

    //TODO the new topic is called control_system_output, don't forget to change after mobing to the new code.
    _sub_controloutput = t_main_handler.subscribe("control_systems_output", 1, callbackControlOutput);
    _instance_ptr = this;

}

ROSUnit_ControlOutputSubscriber::~ROSUnit_ControlOutputSubscriber() {

}

void ROSUnit_ControlOutputSubscriber::callbackControlOutput(const std_msgs::Float64MultiArray& msg){

    _controloutput_msg.data = msg.data;
    _instance_ptr->emitMsgUnicastDefault((DataMessage*) &_controloutput_msg);
}

void ROSUnit_ControlOutputSubscriber::receiveMsgData(DataMessage* t_msg){

}