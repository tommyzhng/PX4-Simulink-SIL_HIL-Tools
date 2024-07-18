/*
 * C++ S-Function for PX4 SITL with Mavlink Conversions
 * Copyright (c) 2024 Ziyang Zhang
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME px4_sitl_gps

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <sstream>

#include "mavlink/common/mavlink.h" // MAVLink header
#include "simstruc.h"

static std::string errorMsg;

// declare helper functions
static mavlink_message_t mavlinkMsg;
void CreateHILGPSMessage(mavlink_hil_gps_t *gpsMsg, const real_T* const time_usec, const real_T* const LLA, const real_T* const velocity, const real_T* const gndSpeed, const real_T* const course);

// Function: mdlInitializeSizes
// Purpose:  Initialize the sizes array
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)){
        return;
    }
    if (!ssSetNumInputPorts(S, 5)){
        return;
    }

    // GPS
    ssSetInputPortWidth(S, 0, 3); // lat,long,alt measured
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortWidth(S, 1, 3); // xyz velocity
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortWidth(S, 2, 1); // Ground Speed
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortWidth(S, 3, 1); // Course
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    // Time
    ssSetInputPortWidth(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);

    if (!ssSetNumOutputPorts(S, 1)){
        return;
    }
    ssSetOutputPortWidth(S, 0, 1024);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1);
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
        // initialize buffer to store data
        uint8_t *buffer = nullptr;
        buffer = (uint8_t *)calloc(1024, 1);
        ssSetPWorkValue(S, 0, (void *)buffer); // store the buffer in another PWork vector
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
        // get the buffer from the PWork vector
        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 0);

        if (buffer == NULL){
            return;
        }

        // read the input ports
        InputRealPtrsType LLA = ssGetInputPortRealSignalPtrs(S, 0);
        InputRealPtrsType velocity = ssGetInputPortRealSignalPtrs(S, 1);   
        InputRealPtrsType gndSpeed = ssGetInputPortRealSignalPtrs(S, 2);
        InputRealPtrsType course = ssGetInputPortRealSignalPtrs(S, 3);
        InputRealPtrsType time_ = ssGetInputPortRealSignalPtrs(S, 4);
        
        // set memory
        memset(buffer, 0, 1024);

        // create HIL_SENSOR message
         mavlink_hil_gps_t hil_gps_msg;
        CreateHILGPSMessage(&hil_gps_msg, time_[0], LLA[0], velocity[0], gndSpeed[0], course[0]);
        mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &mavlinkMsg, &hil_gps_msg);
        auto sendBytesLength = mavlink_msg_to_send_buffer(&buffer[0], &mavlinkMsg);

        // send the data
        real_T *gpsBuffer = ssGetOutputPortRealSignal(S, 0); 
        std::copy(buffer, buffer + sendBytesLength, gpsBuffer);

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
        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 0);
        if (buffer){
            free(buffer);
        }
    }
}

// Function: CreateHILGPSMessage
// Purpose:  Title is pretty self-explanatory
void CreateHILGPSMessage(mavlink_hil_gps_t *gpsMsg, const real_T* const time_usec, const real_T* const LLA, const real_T* const velocity, const real_T* const gndSpeed, const real_T* const course)
{
    gpsMsg->time_usec = (uint64_t)((time_usec[0]) * 1e6);
    gpsMsg->lat = (int32_t)(LLA[0] * 1e7);          // degE7
    gpsMsg->lon = (int32_t)(LLA[1] * 1e7);          // degE7
    gpsMsg->alt = (int32_t)(LLA[2] * 1e3);          // cm
    gpsMsg->eph = (uint16_t)300;                    // hdop of 1.0
    gpsMsg->epv = (uint16_t)400;
    gpsMsg->vel = (uint16_t)(velocity[0] * 100);    // cm/s for all vel
    gpsMsg->vn = (int16_t)(velocity[0] * 100);
    gpsMsg->ve = (int16_t)(velocity[1] * 100);
    gpsMsg->vd = (int16_t)(velocity[2] * 100);
    gpsMsg->cog = (uint16_t)(course[0] * 100);      
    gpsMsg->fix_type = (uint8_t)3;                  // 3D fix
    gpsMsg->satellites_visible = (uint8_t)10;
    gpsMsg->id = (uint8_t)0;
    gpsMsg->yaw = (uint16_t)0;
}


#if defined(MATLAB_MEX_FILE)

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

#endif