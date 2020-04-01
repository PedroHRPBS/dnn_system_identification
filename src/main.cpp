#include "ros/ros.h"
#include <iostream>
#include "Python.h"
#include "ROSUnit_ControlOutputSubscriber.hpp"
#include "IdentificationNode.hpp"
#include "MsgReceiver.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "dnn_sys_id_node");
    ros::NodeHandle nh;
    ros::Rate rate(400);

    ROSUnit* ros_controloutput_sub = new ROSUnit_ControlOutputSubscriber(nh);
    IdentificationNode* roll_identification_node = new IdentificationNode(control_system::roll);

    ros_controloutput_sub->addCallbackMsgReceiver((MsgReceiver*)roll_identification_node);

    printf("Calling Python to find the sum of 2 and 2.\n");
    
    // Initialize the Python interpreter.
    
    Py_Initialize();

    PyObject* sysPath = PySys_GetObject((char*)"path");
    PyList_Append(sysPath, PyString_FromString("/home/pedrohrpbs/catkin_ws_NAVIO/src/dnn_system_identification/src"));
    
    // Create some Python objects that will later be assigned values.
    
    PyObject *pName, *pModule, *pDict, *pFunc, *pArgs, *pValue;
    
    // Convert the file name to a Python string.
    
    pName = PyString_FromString("Sample");
    // Import the file as a Python module.
    
    pModule = PyImport_Import(pName);
    
    // Create a dictionary for the contents of the module.

    pDict = PyModule_GetDict(pModule);
    // Get the add method from the dictionary.
    
    pFunc = PyDict_GetItemString(pDict, "add");
    
    // Create a Python tuple to hold the arguments to the method.
    
    pArgs = PyTuple_New(2);
    
    // Convert 2 to a Python integer.
    
    pValue = PyInt_FromLong(2);
    
    // Set the Python int as the first and second arguments to the method.
    
    PyTuple_SetItem(pArgs, 0, pValue);
    
    PyTuple_SetItem(pArgs, 1, pValue);
    
    // Call the function with the arguments.
    
    PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
    
    // Print a message if calling the method failed.
    
    if(pResult == NULL)
        printf("Calling the add method failed.\n");
    
    // Convert the result to a long from a Python object.
    
    long result = PyInt_AsLong(pResult);
    
    // Destroy the Python interpreter.
    
    Py_Finalize();
    
    // Print the result.
    
    printf("The result is %ld.\n", result);    

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep(); 
    }

    return 0;
}
