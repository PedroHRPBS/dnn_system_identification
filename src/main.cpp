#include "ros/ros.h"
#include <iostream>
#include "Python.h"
#include "ROSUnit_ControlOutputSubscriber.hpp"
#include "IdentificationNode.hpp"
#include "MsgReceiver.hpp"
#include "ROSUnit_Factory.hpp"
#include "ROSUnit_OrientationSubscriber.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "dnn_sys_id_node");
    ros::NodeHandle nh;
    ros::Rate rate(120);

    ROSUnit* ros_controloutput_sub = new ROSUnit_ControlOutputSubscriber(nh);
    //TODO remove this, it's only for testing. I have no bag file with the new topic for roll
    ROSUnit* ros_orientation_sub = new ROSUnit_OrientationSubscriber(nh);


    ROSUnit_Factory ROSUnit_Factory_main{nh};
    ROSUnit* rosunit_x_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/x");
    ROSUnit* rosunit_y_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/y");
    ROSUnit* rosunit_z_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/z");
    ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/roll");
    ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pitch");
    ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    ROSUnit* rosunit_yaw_rate_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw_rate");

    IdentificationNode* roll_identification_node = new IdentificationNode(control_system::roll);
    IdentificationNode* pitch_identification_node = new IdentificationNode(control_system::pitch);
    IdentificationNode* z_identification_node = new IdentificationNode(control_system::z);

    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)roll_identification_node);
    ros_orientation_sub->addCallbackMsgReceiver((MsgReceiver*)roll_identification_node);   

    Timer tempo;
    while(ros::ok()){
        tempo.tick();
        
        ros::spinOnce();

        rate.sleep(); 
        //std::cout << "Tempo os::ok(): " << tempo.tockMicroSeconds() << "\n";

    }

    roll_identification_node->~IdentificationNode();
    pitch_identification_node->~IdentificationNode();
    z_identification_node->~IdentificationNode();

    return 0;
}
