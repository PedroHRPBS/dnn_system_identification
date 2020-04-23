#include "ros/ros.h"
#include <iostream>
#include "Python.h"
#include "ROSUnit_ControlOutputSubscriber.hpp"
#include "IdentificationNode.hpp"
#include "MsgReceiver.hpp"
#include "ROSUnit_Factory.hpp"
#include "ROSUnit_OrientationSubscriber.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "Timer.hpp"

int main(int argc, char** argv) {

    //There's a problem subscribing to the topics, for some reason they are all getting slowed down.
    //I have a bag file from when I used the drone connected via ethernet, and the providers topics are all 400Hz
    //But now, when I publish the raw output from the sensors only and run the node to genete the providers
    //topic, the topic is only reaching 100Hz.
    //With that being said, there's no problem with the Python slowing down communication, it's something on the network.

    ros::init(argc, argv, "dnn_sys_id_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    //TODO add enable service
    //TODO update PID
    ROSUnit* ros_controloutput_sub = new ROSUnit_ControlOutputSubscriber(nh);
    //TODO remove this, it's only for testing. I have no bag file with the new topic for roll
    ROSUnit* ros_orientation_sub = new ROSUnit_OrientationSubscriber(nh);
    ROSUnit* ros_updt_ctr = new ROSUnit_UpdateController(nh);

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
    ROSUnit* testando_tempo_loop = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/teste");


    IdentificationNode* roll_identification_node = new IdentificationNode(control_system::roll, 0.04);
    IdentificationNode* pitch_identification_node = new IdentificationNode(control_system::pitch, 0.04);
    IdentificationNode* z_identification_node = new IdentificationNode(control_system::z, 0.1);

    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)roll_identification_node);
    ros_orientation_sub->addCallbackMsgReceiver((MsgReceiver*)roll_identification_node);   
    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)pitch_identification_node);
    ros_orientation_sub->addCallbackMsgReceiver((MsgReceiver*)pitch_identification_node);   
    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)z_identification_node);
    ros_orientation_sub->addCallbackMsgReceiver((MsgReceiver*)z_identification_node);  

    roll_identification_node->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);
    pitch_identification_node->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);
    z_identification_node->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);

    roll_identification_node->addCallbackMsgReceiver((MsgReceiver*)ros_updt_ctr);
    pitch_identification_node->addCallbackMsgReceiver((MsgReceiver*)ros_updt_ctr); 
    z_identification_node->addCallbackMsgReceiver((MsgReceiver*)ros_updt_ctr); 



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
