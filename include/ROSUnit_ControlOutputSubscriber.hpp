#pragma once
#include "common_srv/ROSUnit.hpp"
#include <std_msgs/Float64MultiArray.h>
#include "common_srv/VectorDoubleMsg.hpp"

class ROSUnit_ControlOutputSubscriber : public ROSUnit{

private:
    ros::Subscriber _sub_controloutput;
    static ROSUnit_ControlOutputSubscriber* _instance_ptr;
    static VectorDoubleMsg _controloutput_msg;
    static void callbackControlOutput(const std_msgs::Float64MultiArray& msg);

public:
    void receiveMsgData(DataMessage*);
    ROSUnit_ControlOutputSubscriber(ros::NodeHandle&);
    ~ROSUnit_ControlOutputSubscriber();


};