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
    //The following vectors should be increased to contain all the paths to all the possible neural networks
    std::vector<const char*> dnn_model_paths = {"/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/model.h5",
                                                "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/model.h5"};
    
    std::vector<const char*> system_class_paths = {"/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/systems_truth_table.csv",
                                                   "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/systems_truth_table.csv"};



    enum unicast_addresses {id_node, ros};
    void receiveMsgData(DataMessage* t_msg);
    void callPython(double, double);
    void initializePython();
    void setDNNModelinPython(const char*, const char*);
    IdentificationNode(control_system, double, bool);
    ~IdentificationNode();
    
};