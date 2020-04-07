#include "IdentificationNode.hpp"
#include "Timer.hpp"

Timer tempo;
Timer tempo2;
Timer tempo3;

IdentificationNode::IdentificationNode(control_system t_cs) {
    _cs_type = t_cs;
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
    this->_my_identifier = PyObject_CallObject(py_receive_data_function, NULL);
}

void IdentificationNode::callPython(double t_pv, double t_u){

    PyObject* py_pv = PyFloat_FromDouble(t_pv);
    PyObject* py_u = PyFloat_FromDouble(t_u);
    PyObject* time_now = PyFloat_FromDouble(ros::Time::now().toSec());
    PyObject* method_name = PyString_FromString("receive_data");
    PyObject* py_receive_data_return = PyObject_CallMethodObjArgs(this->_my_identifier, method_name, py_pv, py_u, time_now, NULL);



    if(py_receive_data_return == NULL){
        printf("Calling the add method failed.\n");
    }
    //tempo.tick();
    
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
        
    
    }

}