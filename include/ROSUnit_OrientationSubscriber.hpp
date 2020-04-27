#pragma once
#include "common_srv/ROSUnit.hpp"
#include <geometry_msgs/PointStamped.h>
#include "common_srv/Vector3DMessage.hpp"

class ROSUnit_OrientationSubscriber : public ROSUnit {

private:  
    ros::Subscriber _sub_orientation;
    static ROSUnit_OrientationSubscriber* _instance_ptr;
    static Vector3DMessage orientation_msg; 
    static void callbackOrientation(const geometry_msgs::PointStamped& msg);
       
public:

    void receiveMsgData(DataMessage*);
    ROSUnit_OrientationSubscriber(ros::NodeHandle&);
    ~ROSUnit_OrientationSubscriber();
};