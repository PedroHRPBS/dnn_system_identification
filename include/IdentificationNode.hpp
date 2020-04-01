#pragma once
#include "VectorDoubleMsg.hpp"
#include "Vector3DMessage.hpp"
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "Python.h"

class IdentificationNode : public MsgEmitter, public MsgReceiver{

private:
    double _PV, _u;
    control_system _cs_type;

public:
    void receiveMsgData(DataMessage* t_msg);
    void callPython(double, double);
    void initializePython();
    IdentificationNode(control_system);
    ~IdentificationNode();
};