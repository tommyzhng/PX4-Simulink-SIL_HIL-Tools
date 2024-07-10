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

    if (!ssSetNumInputPorts(S, 3))
    {
        return;
    }

    // serialized MAVLink message from PX4 Toolbox:
    // 1. Sensor data (GPS, IMU, etc.)
    ssSetInputPortWidth(S, 0, 76);
    ssSetInputPortDataType(S, 0, SS_UINT8);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    // 2. GPS data
    ssSetInputPortWidth(S, 1, 49);
    ssSetInputPortDataType(S, 1, SS_UINT8);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    // 3. Heartbeat
    ssSetInputPortWidth(S, 2, 21);
    ssSetInputPortDataType(S, 2, SS_UINT8);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    if (!ssSetNumOutputPorts(S, 1))
    {
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

        if (buffer == NULL || socket_ == NULL)
        {
            return;
        }

        // get the input port
        InputRealPtrsType sensorBytes = ssGetInputPortRealSignalPtrs(S, 0);
        InputRealPtrsType gpsBytes = ssGetInputPortRealSignalPtrs(S, 1);
        InputRealPtrsType heartbeatBytes = ssGetInputPortRealSignalPtrs(S, 2);
        
        // Ensure buffer is properly sized
        memset(buffer, 0, 1024);

        // Ensure correct type casting and pointer arithmetic
        const uint8_t* sensorPtr = reinterpret_cast<const uint8_t*>(sensorBytes[0]);
        const uint8_t* gpsPtr = reinterpret_cast<const uint8_t*>(gpsBytes[0]);
        const uint8_t* heartbeatPtr = reinterpret_cast<const uint8_t*>(heartbeatBytes[0]);

        // directly access the bytes of the input port and copy data
        std::copy(sensorPtr, sensorPtr + 76, buffer); // sensor data
        std::copy(gpsPtr, gpsPtr + 49, buffer + 76); // GPS data
        std::copy(heartbeatPtr, heartbeatPtr + 21, buffer + 125); // heartbeat

        // send the data to PX4 SITL
        asio::error_code ec;
        socket_->send(asio::buffer(buffer, 1024), 0, ec);
        
        // receive data from PX4 SITL
        auto recievedBytesLength = socket_->available();
        if (recievedBytesLength > 0)
        {
            memset(buffer, 0, 1024);
            auto recievedBytes = socket_->receive(asio::buffer(buffer, recievedBytesLength));
            mavlink_message_t msg;
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
        real_T *pwm = ssGetOutputPortRealSignal(S, 0);
        for (int i = 0; i < 16; i++)
        {
            pwm[i] = (real_T)hilActuatorControlsMsg.controls[i];
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