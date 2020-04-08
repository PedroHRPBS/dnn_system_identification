#include "ControllerMessage.hpp"

ControllerMessage::ControllerMessage() {
    _type = msg_type::UPDATECONTROLLER;;
}

ControllerMessage::~ControllerMessage() {

}

msg_type ControllerMessage::getType(){
    return _type;
}

const int ControllerMessage::getSize()
{
    return sizeof(this);
}

void ControllerMessage::setPIDParam(PID_parameters t_param){

    _pid_param.kp = t_param.kp;
    _pid_param.ki = t_param.ki;
    _pid_param.kd = t_param.kd;
    _pid_param.kdd = t_param.kdd;
    _pid_param.anti_windup = t_param.anti_windup;
    _pid_param.en_pv_derivation = t_param.en_pv_derivation;
    _pid_param.id = t_param.id; //TODO check if this is needed
    if(t_param.id!=block_id::NULL_ID){
        _id = t_param.id;
    }
    
    
}

void ControllerMessage::setMRFTParam(MRFT_parameters t_param){

    _mrft_param.beta = t_param.beta;
    _mrft_param.relay_amp = t_param.relay_amp;
    _mrft_param.bias = t_param.bias;
    _mrft_param.id = t_param.id;
    if(t_param.id!=block_id::NULL_ID){
        _id = t_param.id;
    }
    
}

void ControllerMessage::set_dt(float t_dt){
    //TODO check if they can be together
    _mrft_param.dt = t_dt;
    _pid_param.dt = t_dt;
}
