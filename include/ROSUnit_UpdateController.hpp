#pragma once
#include <flight_controller/Update_Controller.h>
#include "ROSUnit.hpp"
#include "ControllerMessage.hpp"
#include <flight_controller/Controller_param.h>

class ROSUnit_UpdateController : public ROSUnit {

private:
    ros::ServiceClient _update_controller_client;

public:
    void receiveMsgData(DataMessage* t_msg);
    ROSUnit_UpdateController(ros::NodeHandle&);
    ~ROSUnit_UpdateController();
};