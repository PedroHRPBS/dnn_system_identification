#include "IdentificationNode.hpp"

IdentificationNode::IdentificationNode(control_system t_cs) {
    _cs_type = t_cs;
}

IdentificationNode::~IdentificationNode() {

}

void IdentificationNode::receiveMsgData(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::VECTORDOUBLE){
        VectorDoubleMsg* vector_double_msg = (VectorDoubleMsg*)t_msg;
        _u = vector_double_msg->data[(int)_cs_type];
        std::cout << "U: " << _u << "\n";
    
    }else if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* vector3d_msg = (Vector3DMessage*)t_msg;
        _PV = vector3d_msg->getData().x;
        std::cout << "_PV: " << _PV << "\n";
    }

}