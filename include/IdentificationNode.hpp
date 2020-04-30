#pragma once
#include "common_srv/VectorDoubleMsg.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "Python.h"
#include "ros/ros.h"
#include "ControllerMessage.hpp"
#include "common_srv/IntegerMsg.hpp"

class IdentificationNode : public MsgEmitter, public MsgReceiver{

private:
    double _PV, _u, _Kp, _Kd, _h_mrft;
    int _system_class;
    control_system _cs_type;
    PyObject* _my_identifier;
    bool _enabled = true;
    PID_parameters pid_data;

public:
    enum unicast_addresses {id_node, ros};
    void receiveMsgData(DataMessage* t_msg);
    void callPython(double, double);
    void initializePython();
    void setDNNModelinPython(const char*, const char*);
    IdentificationNode(control_system, double, bool);
    ~IdentificationNode();
    
};