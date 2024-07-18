/*
 * C++ S-Function for PX4 SITL with Mavlink Conversions
 * Copyright (c) 2024 Ziyang Zhang
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME px4_sitl_connection

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <sstream>

#include <asio.hpp> // ASIO header
#include "mavlink/common/mavlink.h" // MAVLink header
#include "simstruc.h"

static asio::io_service ioService;
static asio::ip::tcp::socket socket_(ioService);
static std::string errorMsg;

// declare helper functions
static mavlink_message_t mavlinkMsg;
static mavlink_hil_actuator_controls_t hilActuatorControlsMsg;
void CreateHeartbeatMessage(mavlink_heartbeat_t *heartbeatMsg);

// Function: mdlInitializeSizes
// Purpose:  Initialize the sizes array
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)){
        return;
    }
    if (!ssSetNumInputPorts(S, 2)){
        return;
    }
    
    ssSetInputPortWidth(S, 0, 1024);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortWidth(S, 1, 1024);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);

    // Sensor Buffer

    if (!ssSetNumOutputPorts(S, 1)){
        return;
    }

    ssSetOutputPortWidth(S, 0, 16);
    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 2);
    ssSetOptions(S, 0);
}

// Function: mdlInitializeSampleTimes
// Purpose:  Initialize the sample times array

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START
#if defined(MDL_START)
// Function: mdlStart
// Purpose:  Initialize the socket connection to PX4 SITL
static void mdlStart(SimStruct *S)
{
    try{
        // initialize socket connection to PX4 SITL
        asio::ip::tcp::endpoint endpoint(asio::ip::address::from_string("0.0.0.0"), 4560); // any incoming ip that has port 4560
        asio::ip::tcp::acceptor acceptor(ioService, endpoint);
        mexPrintf("Waiting for connection to PX4 SITL\n");
        acceptor.accept(socket_);
        mexPrintf("Connected to PX4 SITL\n");
        ssSetPWorkValue(S, 0, (void *)&socket_); // store the socket in the PWork vector

        // initialize buffer to store data
        uint8_t *buffer = nullptr;
        buffer = (uint8_t *)calloc(1024, 1);
        ssSetPWorkValue(S, 1, (void *)buffer); // store the buffer in another PWork vector
    }
    catch(const std::exception& e){
        // forward error message to Simulink
        errorMsg = std::string(e.what());
        ssSetErrorStatus(S, errorMsg.c_str()); 
    }
}
#endif

// Function: mdlOutputs
// Purpose:  Send and recieve mavlink data
static void mdlOutputs(SimStruct *S, int_T tid)
{   
    try{   
        // run some checks
        asio::ip::tcp::socket *socket_ = (asio::ip::tcp::socket *)(ssGetPWorkValue(S, 0));
        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 1);
        
        if (buffer == NULL || socket_ == NULL){
            return;
        }

        // send data to PX4 SITL

        InputRealPtrsType sensorBytes = ssGetInputPortRealSignalPtrs(S, 0);
        InputRealPtrsType gpsBytes = ssGetInputPortRealSignalPtrs(S, 1);
        
        memset(buffer, 0, 1024);

        // create heartbeat message
        mavlink_heartbeat_t heartbeat_msg;
        CreateHeartbeatMessage(&heartbeat_msg);
        mavlink_msg_heartbeat_encode_chan(1, 200, MAVLINK_COMM_0, &mavlinkMsg, &heartbeat_msg);
        auto sendBytesLength = mavlink_msg_to_send_buffer(&buffer[0], &mavlinkMsg);

        // concatenate sensor data (74 bytes)
        std::copy(sensorBytes[0], sensorBytes[0] + 74, buffer + sendBytesLength);
        sendBytesLength += 74;

        // concatenate gps data (48 bytes)
        std::copy(gpsBytes[0], gpsBytes[0] + 48, buffer + sendBytesLength);
        sendBytesLength += 48;

        // send the data
        socket_->send(asio::buffer(buffer, sendBytesLength));
        
        // receive data from PX4 SITL
        
        uint8_t recievedBytesLength = socket_->available();
        if (recievedBytesLength > 0){
            memset(buffer, 0, 1024);
            auto recievedBytes = socket_->receive(asio::buffer(buffer, recievedBytesLength));
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < recievedBytesLength; i++){
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)){   
                    if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS){
                        mavlink_msg_hil_actuator_controls_decode(&msg, &hilActuatorControlsMsg);
                    }
                }
            }
        }
        real_T *pwm = ssGetOutputPortRealSignal(S, 0); 
        for (int i = 0; i < 16; i++){
            pwm[i] = (real_T)hilActuatorControlsMsg.controls[i];
        }
    }
    catch (const std::exception &e){
        errorMsg = std::string(e.what());
        ssSetErrorStatus(S, errorMsg.c_str());
    }
}

// Function: mdlTerminate
// Purpose:  Close the socket connection to PX4 SITL
static void mdlTerminate(SimStruct *S)
{
    if (ssGetPWork(S) != NULL){
        asio::ip::tcp::socket *socket_ = (asio::ip::tcp::socket *)ssGetPWorkValue(S, 0);
        if (socket_){
            socket_->close();
        }

        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 1);
        if (buffer){
            free(buffer);
        }
    }
}

// Function CreateHeartbeatMessage
// Purpose:  Title is pretty self-explanatory
void CreateHeartbeatMessage(mavlink_heartbeat_t *heartbeatMsg)
{
    heartbeatMsg->autopilot = (uint8_t)MAV_AUTOPILOT_GENERIC;
    heartbeatMsg->type = (uint8_t)MAV_TYPE_GENERIC;
    heartbeatMsg->system_status = (uint8_t)0;
    heartbeatMsg->base_mode = (uint8_t)0;
    heartbeatMsg->custom_mode = (uint32_t)0;
}

#if defined(MATLAB_MEX_FILE)

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

#endif
