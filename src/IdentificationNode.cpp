#include "IdentificationNode.hpp"
#include "common_srv/Timer.hpp"

Timer tempo;
Timer tempo2;
Timer tempo3;

IdentificationNode::IdentificationNode(control_system t_cs, double t_h_mrft, bool t_enabled) {
    _cs_type = t_cs;
    _h_mrft = t_h_mrft;
    _enabled = t_enabled;
    this->initializePython();
}

IdentificationNode::~IdentificationNode() {
    // Destroy the Python interpreter.
    Py_Finalize();
}

void IdentificationNode::initializePython(){
    // Initialize the Python interpreter.    
    Py_Initialize();
    // Update Python path to include current folder
    PyObject* sysPath = PySys_GetObject((char*)"path");
    PyList_Append(sysPath, PyString_FromString("/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src"));
    
    printf("initializePython\n");   

    PyObject* py_filename = PyString_FromString("IdentificationClass");
    PyObject* py_module = PyImport_Import(py_filename); 
    PyObject* py_dictionary = PyModule_GetDict(py_module); 
    PyObject* py_receive_data_function = PyDict_GetItemString(py_dictionary, "return_instance");
    PyObject* py_h_mrft = PyTuple_New(1);
    PyTuple_SetItem(py_h_mrft, 0, PyFloat_FromDouble(_h_mrft));
    this->_my_identifier = PyObject_CallObject(py_receive_data_function, py_h_mrft);
}

void IdentificationNode::callPython(double t_pv, double t_u){

    if(_enabled){
        PyObject* py_pv = PyFloat_FromDouble(t_pv);
        PyObject* py_u = PyFloat_FromDouble(t_u);
        PyObject* time_now = PyFloat_FromDouble(ros::Time::now().toSec());
        PyObject* method_name = PyString_FromString("receive_data");
        PyObject* py_receive_data_return = PyObject_CallMethodObjArgs(this->_my_identifier, method_name, py_pv, py_u, time_now, NULL);
    
        _Kp = PyFloat_AsDouble(PyTuple_GetItem(py_receive_data_return, 0));
        _Kd = PyFloat_AsDouble(PyTuple_GetItem(py_receive_data_return, 1));
        _system_class = PyInt_AsLong(PyTuple_GetItem(py_receive_data_return, 2));
    
        if(_Kp > 0.0 && _Kd > 0.0){
            //printf("CS = %d: KP = %lf, KD = %lf\n", (int)_cs_type, _Kp, _Kd);
            _enabled = false;
            
            ControllerMessage pid_parameters_message;

            pid_data.kp = _Kp;
            pid_data.ki = -1.0;
            pid_data.kd = _Kd;
            pid_data.kdd = 0.0;
            pid_data.anti_windup = 0.0;
            pid_data.en_pv_derivation = 1.0;
            pid_data.id = static_cast<block_id>(_cs_type);

            pid_parameters_message.setPIDParam(this->pid_data);
            this->emitMsgUnicast((DataMessage*)&pid_parameters_message, unicast_addresses::ros);

            IntegerMsg system_class_msg;
            system_class_msg.data = _system_class;
            this->emitMsgUnicast((DataMessage*)&system_class_msg, unicast_addresses::id_node);
            
        }
    
        if(py_receive_data_return == NULL){
            printf("Calling the add method failed.\n");
        }

        if(_cs_type == control_system::x){
            std::cout << "Receiving X data "  << std::endl;
        }
        //tempo.tick();
    }
}

void IdentificationNode::setDNNModelinPython(const char* t_dnn_model_path, const char* t_system_classes_path){

    PyObject* dnn_model_path = PyString_FromString(t_dnn_model_path);
    PyObject* system_classes_path = PyString_FromString(t_system_classes_path);
    PyObject* method_name = PyString_FromString("update_dnn_model_and_system");
    PyObject_CallMethodObjArgs(this->_my_identifier, method_name, dnn_model_path, system_classes_path, NULL);

}

void IdentificationNode::receiveMsgData(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::VECTORDOUBLE){
        VectorDoubleMsg* vector_double_msg = (VectorDoubleMsg*)t_msg;
        _u = vector_double_msg->data[(int)_cs_type];
        //std::cout << "Tempo VECTORDOUBLE: " << tempo3.tockMicroSeconds() << "\n";
        //tempo3.tick();
    
    }else if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* vector3d_msg = (Vector3DMessage*)t_msg;
        //TODO change to .x after moving to the newest code
        _PV = vector3d_msg->getData().x;
        //std::cout << "Tempo VECTOR3D: " << tempo2.tockMicroSeconds() << "\n";
        //tempo2.tick();
        //this->emitMsgUnicastDefault(vector3d_msg);
        this->callPython(_PV, _u);

    }else if(t_msg->getType() == msg_type::INTEGER){
        IntegerMsg* integer_msg = (IntegerMsg*)t_msg;
        int classification = integer_msg->data;
        _enabled = true;
        std::cout << "Integer Msg received: " << classification << std::endl;

        //std::string dnn_model_path = "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/" +  std::to_string(classification) + "/model.h5";
        //std::string system_class_path = "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/" +  std::to_string(classification) + "/systems_truth_table.csv";
        std::string dnn_model_path = "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/" +  std::to_string(24) + "/model.h5";
        std::string system_class_path = "/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src/DNNs/" +  std::to_string(24) + "/systems_truth_table.csv";

        this->setDNNModelinPython(dnn_model_path.c_str(), system_class_path.c_str());

    }

}