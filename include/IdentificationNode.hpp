#pragma once
#include "VectorDoubleMsg.hpp"
#include "Vector3DMessage.hpp"
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "Python.h"
#include "ros/ros.h"

class IdentificationNode : public MsgEmitter, public MsgReceiver{

private:
    double _PV, _u;
    control_system _cs_type;
    PyObject* _my_identifier;

public:
    void receiveMsgData(DataMessage* t_msg);
    void callPython(double, double);
    void initializePython();
    IdentificationNode(control_system);
    ~IdentificationNode();
};