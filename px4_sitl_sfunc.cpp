/*
 * C++ S-Function for PX4 SITL with PX4 Toolbox Integration
 *
 * Copyright (c) 2024 Ziyang Zhang
 */

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <sstream>

#include <asio.hpp>
#include "mavlink/common/mavlink.h" // MAVLink header
#include "simstruc.h"

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  px4_sitl_sfunc

static asio::io_service io_service;
static asio::ip::tcp::socket socket(io_service);
statid std::string status;
static mavlink_hil_actuator_controls_t hil_actuator_controls;

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

    ssSetInputPortWidth(S, 0, 1);   // serialized MAVLink message from PX4 Toolbox
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1))
    {
        return;
    }

    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    ssSetOptions(S, 0);
}