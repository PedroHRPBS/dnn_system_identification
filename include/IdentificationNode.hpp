#pragma once
#include "common_srv/VectorDoubleMsg.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "Python.h"
#include "ros/ros.h"
#include "ControllerMessage.hpp"

class IdentificationNode : public MsgEmitter, public MsgReceiver{

private:
    double _PV, _u, _Kp, _Kd, _h_mrft;
    control_system _cs_type;
    PyObject* _my_identifier;
    bool _enabled = true;
    PID_parameters pid_data;

public:
    void receiveMsgData(DataMessage* t_msg);
    void callPython(double, double);
    void initializePython();
    IdentificationNode(control_system, double);
    ~IdentificationNode();
};