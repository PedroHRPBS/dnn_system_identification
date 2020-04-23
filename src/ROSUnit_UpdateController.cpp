#include "ROSUnit_UpdateController.hpp"

ROSUnit_UpdateController::ROSUnit_UpdateController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _update_controller_pid_client = t_main_handler.serviceClient<flight_controller::Update_Controller_PID>("update_controller/pid");

}

ROSUnit_UpdateController::~ROSUnit_UpdateController() {

}

void ROSUnit_UpdateController::receiveMsgData(DataMessage* t_msg){

}

void ROSUnit_UpdateController::receiveMsgData(DataMessage* t_msg, int t_channel){

    if(t_msg->getType() == msg_type::UPDATECONTROLLER){

        if(t_channel == receiving_channels::pid){

            ControllerMessage* _update_msg = (ControllerMessage*)t_msg;
        
            flight_controller::Update_Controller_PID srv;
            srv.request.controller_parameters.id = (int)(_update_msg->getID());
            srv.request.controller_parameters.pid_kp = _update_msg->getPIDParam().kp;
            srv.request.controller_parameters.pid_ki = _update_msg->getPIDParam().ki;
            srv.request.controller_parameters.pid_kd = _update_msg->getPIDParam().kd;
            srv.request.controller_parameters.pid_kdd = _update_msg->getPIDParam().kdd;
            srv.request.controller_parameters.pid_anti_windup = _update_msg->getPIDParam().anti_windup;
            srv.request.controller_parameters.pid_en_pv_derivation = _update_msg->getPIDParam().en_pv_derivation;
            bool success = _update_controller_pid_client.call(srv);
            if (success)
            {
                ROS_INFO("CONTROLLER UPDATED. id: %d", srv.request.controller_parameters.id);
            }
            else 
            {
                ROS_ERROR("Failed to call service /update_controller");
            }
        }
    }
}