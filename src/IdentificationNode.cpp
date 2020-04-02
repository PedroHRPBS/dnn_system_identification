#include "IdentificationNode.hpp"
#include "Timer.hpp"

Timer tempo;
Timer tempo2;

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
    //Update Python path to include current folder
    PyObject* sysPath = PySys_GetObject((char*)"path");
    PyList_Append(sysPath, PyString_FromString("/home/pedrohrpbs/catkin_ws_tensorflow/src/dnn_system_identification/src"));
    //PyList_Append(sysPath, PyString_FromString("/home/pedrohrpbs/catkin_ws_tensorflow/venv/bin/python"));
    
    printf("initializePython\n");   
}

void IdentificationNode::callPython(double t_pv, double t_u){

    std::cout << "Tempo callPython: " << tempo.tockMicroSeconds() << "\n";
    
     // Convert the file name to a Python string.
    PyObject* py_filename = PyString_FromString("identification_functions");
    // Import the file as a Python module.
    PyObject* py_module = PyImport_Import(py_filename); 
    // Create a dictionary for the contents of the module.
    PyObject* py_dictionary = PyModule_GetDict(py_module); 
    // Get the name of the method from the dictionary.
    PyObject* py_receive_data_function = PyDict_GetItemString(py_dictionary, "receive_data");
    // Create a Python tuple to hold the arguments to the method.
    PyObject* receive_data_arguments = PyTuple_New(3);
    // Convert 2 to a Python integer.
    PyObject* py_pv = PyFloat_FromDouble(t_pv);
    PyObject* py_u = PyFloat_FromDouble(t_u);
    PyObject* time_now = PyFloat_FromDouble(ros::Time::now().toSec());
    // Set the Python int as the first and second arguments to the method.
    PyTuple_SetItem(receive_data_arguments, 0, py_pv);
    PyTuple_SetItem(receive_data_arguments, 1, py_u);
    PyTuple_SetItem(receive_data_arguments, 2, time_now);
    // Call the function with the arguments.
    PyObject* py_receive_data_return = PyObject_CallObject(py_receive_data_function, receive_data_arguments);
    // Print a message if calling the method failed.
    if(py_receive_data_return == NULL){
        printf("Calling the add method failed.\n");
    }
    tempo.tick();
    
}

void IdentificationNode::receiveMsgData(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::VECTORDOUBLE){
        VectorDoubleMsg* vector_double_msg = (VectorDoubleMsg*)t_msg;
        _u = vector_double_msg->data[(int)_cs_type];
    
    }else if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* vector3d_msg = (Vector3DMessage*)t_msg;
        //TODO change to .x after moving to the newest code
        _PV = vector3d_msg->getData().y;
        std::cout << "Tempo receiveMsgData: " << tempo2.tockMicroSeconds() << "\n";
        tempo2.tick();
        this->callPython(_PV, _u);
        
    
    }

}