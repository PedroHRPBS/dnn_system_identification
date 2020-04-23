#pragma once
#include <flight_controller/Update_Controller_PID.h>
#include "ROSUnit.hpp"
#include "ControllerMessage.hpp"
#include <flight_controller/PID_param.h>

class ROSUnit_UpdateController : public ROSUnit {

private:
    ros::ServiceClient _update_controller_pid_client;

public:
    enum receiving_channels {pid, mrft, sm};
    void receiveMsgData(DataMessage* t_msg);
    void receiveMsgData(DataMessage* t_msg, int t_channel);

    ROSUnit_UpdateController(ros::NodeHandle&);
    ~ROSUnit_UpdateController();
};