/*
 * C++ S-Function for PX4 SITL with PX4 Toolbox Integration
 * Copyright (c) 2024 Ziyang Zhang
 */
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME px4_sitl_sfunc

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <sstream>

#include <asio.hpp>
#include "mavlink/common/mavlink.h" // MAVLink header
#include "simstruc.h"

static asio::io_service ioService;
static asio::ip::tcp::socket socket_(ioService);
static std::string errorMsg;
static mavlink_hil_actuator_controls_t hilActuatorControlsMsg;


// Function: mdlInitializeSizes
// Purpose:  Initialize the sizes array
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return;
    }

    if (!ssSetNumInputPorts(S, 1))
    {
        return;
    }

    // serialized MAVLink message from PX4 Toolbox: size is 1x146 matrix
    ssSetInputPortWidth(S, 0, 146);
    ssSetInputPortDataType(S, 0, SS_UINT8);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1))
    {
        return;
    }

    ssSetOutputPortWidth(S, 0, 1024);
    ssSetOutputPortDataType(S, 0, SS_UINT8);

    ssSetNumSampleTimes(S, 1);
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumPWork(S, 2);

    ssSetOptions(S, 0);
}

// Function: mdlInitializeSampleTimes
// Purpose:  Initialize the sample times array

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.01);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START
#if defined(MDL_START)
// Function: mdlStart
// Purpose:  Initialize the socket connection to PX4 SITL
static void mdlStart(SimStruct *S)
{
    try
    {
        asio::ip::tcp::endpoint endpoint(asio::ip::address::from_string("0.0.0.0"), 4560); // any incoming ip that has port 4560
        asio::ip::tcp::acceptor acceptor(ioService, endpoint);
        mexPrintf("Waiting for connection to PX4 SITL\n");

        acceptor.accept(socket_);
        mexPrintf("Connected to PX4 SITL\n");
        ssSetPWorkValue(S, 0, (void *)&socket_); // store the socket in the PWork vector

        uint8_t *buffer = nullptr;
        buffer = (uint8_t *)calloc(1024, 1);
        ssSetPWorkValue(S, 1, (void *)buffer); // store the buffer in another PWork vector
    }
    catch(const std::exception& e)
    {
        // forward error message to Simulink
        errorMsg = std::string(e.what());
        ssSetErrorStatus(S, errorMsg.c_str()); 
    }
}
#endif

// Function: mdlOutputs
// Purpose:  Send the MAVLink message to PX4 SITL
static void mdlOutputs(SimStruct *S, int_T tid)
{   
    try
    {
        asio::ip::tcp::socket *socket_ = (asio::ip::tcp::socket *)(ssGetPWorkValue(S, 0));
        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 1);
        mavlink_message_t msg;

        if (buffer == NULL || socket_ == NULL)
        {
            return;
        }

        // get the input port
        InputRealPtrsType mavlinkMsg = ssGetInputPortRealSignalPtrs(S, 0);
        
        memset(buffer, 0, 1024);
        auto bytes = mavlink_msg_to_send_buffer(buffer, (const mavlink_message_t *)mavlinkMsg[0]);
        auto sendBytes = socket_->send(asio::buffer(buffer, bytes));

        auto recievedBytesLength = socket_->available();
        if (recievedBytesLength > 0)
        {
            memset(buffer, 0, 1024);
            auto recievedBytes = socket_->receive(asio::buffer(buffer, recievedBytesLength));
            mavlink_status_t status;
            for (int i = 0; i < recievedBytesLength; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
                {   
                    // switch to handle additional messages in the future
                    switch (msg.msgid) 
                    {
                    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                        mavlink_msg_hil_actuator_controls_decode(&msg, &hilActuatorControlsMsg);
                        break;
                    default:
                        break;
                    }
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        errorMsg = std::string(e.what());
        ssSetErrorStatus(S, errorMsg.c_str());
    }
}

// Function: mdlTerminate
// Purpose:  Close the socket connection to PX4 SITL

static void mdlTerminate(SimStruct *S)
{
    if (ssGetPWork(S) != NULL)
    {
        asio::ip::tcp::socket *socket_ = (asio::ip::tcp::socket *)ssGetPWorkValue(S, 0);
        if (socket_)
        {
            socket_->close();
        }

        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 1);
        if (buffer)
        {
            free(buffer);
        }
    }
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif