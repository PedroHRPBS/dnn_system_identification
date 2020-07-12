#include "CheckCondition.hpp"

CheckCondition::CheckCondition() {
    _current_height = 0.0;
}

CheckCondition::~CheckCondition() {

}

void CheckCondition::receiveMsgData(DataMessage* t_msg){
    if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* vector3d_msg = (Vector3DMessage*)t_msg;
        _current_height = vector3d_msg->getData().x;

        BooleanMsg enable_msg;
        enable_msg.data = false;

        if(_current_height >= _min_height){
            enable_msg.data = true;
        }

        this->emitMsgUnicastDefault((DataMessage*)&enable_msg);
    }
}

void CheckCondition::receiveMsgData(DataMessage* t_msg, int t_channel){

    if(t_msg->getType() == msg_type::INTEGER){
        IntegerMsg* integer_msg = (IntegerMsg*)t_msg;
        int classification = integer_msg->data;

        if(t_channel == (int)control_system::roll){
            if(classification >= 0){
                _roll_id_done = true;
                // std::cout << "ROLL ID TRUE" << std::endl;
            }
        }else if(t_channel == (int)control_system::pitch){
            if(classification >= 0){
                _pitch_id_done = true;
                // std::cout << "PITCH ID TRUE" << std::endl;

            }
        }

        if(_pitch_id_done && _roll_id_done){
            BooleanMsg enable_msg;
            enable_msg.data = true;
            // std::cout << "ROLL AND PITCH TRUE, SENDING MSG TO X AND Y" << std::endl;
            
            this->emitMsgUnicastDefault((DataMessage*)&enable_msg);
        }

    }

}
