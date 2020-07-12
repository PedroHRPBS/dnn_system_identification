#pragma once
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/BooleanMsg.hpp"
#include "common_srv/IntegerMsg.hpp"
#include "common_types.hpp"

class CheckCondition : public MsgEmitter, public MsgReceiver{

private:
    float _current_height, _min_height = -0.3;
    bool _roll_id_done = false, _pitch_id_done = false;

public:
    void receiveMsgData(DataMessage* t_msg);
    void receiveMsgData(DataMessage* t_msg, int t_channel);
    CheckCondition();
    ~CheckCondition();
};