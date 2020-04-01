#pragma once
#include "VectorDoubleMsg.hpp"
#include "Vector3DMessage.hpp"
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "Python.h"
#include "Timer.hpp"

class IdentificationNode : public MsgEmitter, public MsgReceiver{

private:
    double _PV, _u;
    control_system _cs_type;

Timer tempoU;
Timer tempoPV;
public:
    void receiveMsgData(DataMessage* t_msg);
    
    IdentificationNode(control_system);
    ~IdentificationNode();
};