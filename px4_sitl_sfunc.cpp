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

#include <asio.hpp> // ASIO header
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

    // Accel
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    // Gyro
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    // Magnetometer
    ssSetInputPortWidth(S, 2, 3);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    // Barometer
    ssSetInputPortWidth(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    // GPS
    ssSetInputPortWidth(S, 4, 3); // lat,long,alt measured
    ssSetInputPortDirectFeedThrough(S, 4, 1);

    ssSetInputPortWidth(S, 5, 3); // xyz velocity
    ssSetInputPortDirectFeedThrough(S, 5, 1);

    ssSetInputPortWidth(S, 6, 1); // Ground Speed
    ssSetInputPortDirectFeedThrough(S, 6, 1);

    ssSetInputPortWidth(S, 7, 1); // Course
    ssSetInputPortDirectFeedThrough(S, 7, 1);

    // Time
    ssSetInputPortWidth(S, 8, 1);
    ssSetInputPortDirectFeedThrough(S, 8, 1);


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
    ssSetSampleTime(S, 0, 0.004);
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
    catch(const std::exception& e)
    {
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
    try
    {   
        // get the socket and buffer from the PWork vector
        asio::ip::tcp::socket *socket_ = (asio::ip::tcp::socket *)(ssGetPWorkValue(S, 0));
        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 1);

        if (buffer == NULL || socket_ == NULL)
        {
            return;
        }

        // read the input ports
        InputRealPtrsType acc = ssGetInputPortRealSignalPtrs(S, 0);
        InputRealPtrsType gyro = ssGetInputPortRealSignalPtrs(S, 1); 
        InputRealPtrsType mag = ssGetInputPortRealSignalPtrs(S, 2);
        InputRealPtrsType baro = ssGetInputPortRealSignalPtrs(S, 3);
        InputRealPtrsType LLA = ssGetInputPortRealSignalPtrs(S, 4);
        InputRealPtrsType velocity = ssGetInputPortRealSignalPtrs(S, 5);   
        InputRealPtrsType gndSpeed = ssGetInputPortRealSignalPtrs(S, 6);
        InputRealPtrsType course = ssGetInputPortRealSignalPtrs(S, 7);
        InputRealPtrsType time = ssGetInputPortRealSignalPtrs(S, 8);
        
        // Ensure buffer is properly sized
        memset(buffer, 0, 1024);
        mavlink_message_t msg;
        msg.magic = 51;
        msg.len = 146;
        msg.seq = 0;
        msg.sysid = 2;
        msg.compid = 51;
        msg.msgid = 93;
        for (int i = 0; i < 146; i++)
        {
            int blockIndex = i / 8;
            int byteOffset = i % 8;
            // Shift the byte to the correct position and OR it into the correct block
            msg.payload64[blockIndex] |= static_cast<uint64_t>(*mavlinkMsgSerialized[i]) << (byteOffset * 8);
        }


        // send the data to PX4 SITL
        socket_->send(asio::buffer(buffer, 146));
        
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


{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    for (int i = 0; i < 16; i++)
    {
        hilActuatorControlsMsg->controls[i] = (float)(*uPtrs[i]);
    }
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif