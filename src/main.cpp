#include "ros/ros.h"
#include <iostream>
#include "Python.h"
#include "ROSUnit_ControlOutputSubscriber.hpp"
#include "IdentificationNode.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/ROSUnit_Factory.hpp"
#include "ROSUnit_OrientationSubscriber.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "common_srv/Timer.hpp"
#include "CheckCondition.hpp"

int main(int argc, char** argv) {

    //There's a problem subscribing to the topics, for some reason they are all getting slowed down.
    //I have a bag file from when I used the drone connected via ethernet, and the providers topics are all 400Hz
    //But now, when I publish the raw output from the sensors only and run the node to genete the providers
    //topic, the topic is only reaching 100Hz.
    //With that being said, there's no problem with the Python slowing down communication, it's something on the network.

    ros::init(argc, argv, "dnn_sys_id_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    ROSUnit* ros_controloutput_sub = new ROSUnit_ControlOutputSubscriber(nh);
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
    ROSUnit* rosunit_dnn_confirmation = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                           ROSUnit_msg_type::ROSUnit_Int,
                                                                           "/dnn_confirmation");


    IdentificationNode* roll_identification_node = new IdentificationNode(control_system::roll, 0.04, true); //TODO SWITCH TO TRUE ON FULL ID
    roll_identification_node->setDNNModelinPython("/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/inner/model.h5", 
                                                  "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/inner/systems_truth_table.csv");

    IdentificationNode* pitch_identification_node = new IdentificationNode(control_system::pitch, 0.04, true); //TODO SWITCH TO TRUE ON FULL ID
    pitch_identification_node->setDNNModelinPython("/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/inner/model.h5", 
                                                   "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/inner/systems_truth_table.csv");

    IdentificationNode* z_identification_node = new IdentificationNode(control_system::z, 0.107461735722299, true); //TODO SWITCH TO TRUE ON FULL ID
    z_identification_node->setDNNModelinPython("/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/z/model.h5", 
                                               "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/z/systems_truth_table.csv");

    IdentificationNode* x_identification_node = new IdentificationNode(control_system::x, 0.1, false); //TODO SWITCH TO FALSE ON FULL ID
    //WARNING - DO NOT CONSIDER THE GAINS IDENTIFIED FROM THE OUTER LOOP
    //WARNING - NOT USING THE CORRECT TRUTH TABLE
    x_identification_node->setDNNModelinPython("/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/12/model.h5", 
                                               "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/24/systems_truth_table.csv");

    IdentificationNode* y_identification_node = new IdentificationNode(control_system::y, 0.1, false); //TODO SWITCH TO FALSE ON FULL ID
    y_identification_node->setDNNModelinPython("/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/12/model.h5", 
                                               "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/24/systems_truth_table.csv");

    CheckCondition* check_current_height = new CheckCondition();
    CheckCondition* check_inner_loop = new CheckCondition();


    //This part is to enable roll, pitch and Z identification only above a certain threshold.
    rosunit_z_provider->addCallbackMsgReceiver((MsgReceiver*)check_current_height);
    check_current_height->addCallbackMsgReceiver((MsgReceiver*)roll_identification_node); //TODO UNCOMMENT FOR FULL ID
    check_current_height->addCallbackMsgReceiver((MsgReceiver*)pitch_identification_node); //TODO UNCOMMENT FOR FULL ID
    check_current_height->addCallbackMsgReceiver((MsgReceiver*)z_identification_node); //TODO UNCOMMENT FOR FULL ID

    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)roll_identification_node);
    rosunit_roll_provider->addCallbackMsgReceiver((MsgReceiver*)roll_identification_node);   
    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)pitch_identification_node);
    rosunit_pitch_provider->addCallbackMsgReceiver((MsgReceiver*)pitch_identification_node);   
    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)z_identification_node);
    rosunit_z_provider->addCallbackMsgReceiver((MsgReceiver*)z_identification_node);
    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)x_identification_node);
    rosunit_x_provider->addCallbackMsgReceiver((MsgReceiver*)x_identification_node); 
    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)y_identification_node);
    rosunit_y_provider->addCallbackMsgReceiver((MsgReceiver*)y_identification_node); 

    

    roll_identification_node->addCallbackMsgReceiver((MsgReceiver*)x_identification_node, (int)IdentificationNode::unicast_addresses::id_node);
    pitch_identification_node->addCallbackMsgReceiver((MsgReceiver*)y_identification_node, (int)IdentificationNode::unicast_addresses::id_node); 
    roll_identification_node->addCallbackMsgReceiver((MsgReceiver*)check_inner_loop, (int)IdentificationNode::unicast_addresses::id_node);
    pitch_identification_node->addCallbackMsgReceiver((MsgReceiver*)check_inner_loop, (int)IdentificationNode::unicast_addresses::id_node); 
    check_inner_loop->addCallbackMsgReceiver((MsgReceiver*)x_identification_node); //TODO UNCOMMENT FOR FULL ID
    check_inner_loop->addCallbackMsgReceiver((MsgReceiver*)y_identification_node); //TODO UNCOMMENT FOR FULL ID
    

    roll_identification_node->addCallbackMsgReceiver((MsgReceiver*)ros_updt_ctr, (int)IdentificationNode::unicast_addresses::ros);
    pitch_identification_node->addCallbackMsgReceiver((MsgReceiver*)ros_updt_ctr, (int)IdentificationNode::unicast_addresses::ros); 
    z_identification_node->addCallbackMsgReceiver((MsgReceiver*)ros_updt_ctr, (int)IdentificationNode::unicast_addresses::ros); 
    x_identification_node->addCallbackMsgReceiver((MsgReceiver*)ros_updt_ctr, (int)IdentificationNode::unicast_addresses::ros); 
    y_identification_node->addCallbackMsgReceiver((MsgReceiver*)ros_updt_ctr, (int)IdentificationNode::unicast_addresses::ros); 

    roll_identification_node->addCallbackMsgReceiver((MsgReceiver*)rosunit_dnn_confirmation, (int)IdentificationNode::unicast_addresses::ros);
    pitch_identification_node->addCallbackMsgReceiver((MsgReceiver*)rosunit_dnn_confirmation, (int)IdentificationNode::unicast_addresses::ros);
    z_identification_node->addCallbackMsgReceiver((MsgReceiver*)rosunit_dnn_confirmation, (int)IdentificationNode::unicast_addresses::ros);
    x_identification_node->addCallbackMsgReceiver((MsgReceiver*)rosunit_dnn_confirmation, (int)IdentificationNode::unicast_addresses::ros);
    y_identification_node->addCallbackMsgReceiver((MsgReceiver*)rosunit_dnn_confirmation, (int)IdentificationNode::unicast_addresses::ros);


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
    x_identification_node->~IdentificationNode();
    y_identification_node->~IdentificationNode();


    return 0;
}
