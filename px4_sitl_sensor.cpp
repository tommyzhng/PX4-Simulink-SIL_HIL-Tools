/*
 * C++ S-Function for PX4 SITL with Mavlink Conversions
 * Copyright (c) 2024 Ziyang Zhang
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME px4_sitl_sensor

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
void CreateHILSensorMessage(mavlink_hil_sensor_t *sensorMsg, const real_T* const time_usec, const real_T* const accel, const real_T* const gyro, const real_T* const mag, const real_T* const baro);

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

    // Accel
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    // Gyro
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    // Magnetometer
    ssSetInputPortWidth(S, 2, 3);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    // Barometer
    ssSetInputPortWidth(S, 3, 1);
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
        InputRealPtrsType acc = ssGetInputPortRealSignalPtrs(S, 0);
        InputRealPtrsType gyro = ssGetInputPortRealSignalPtrs(S, 1); 
        InputRealPtrsType mag = ssGetInputPortRealSignalPtrs(S, 2);
        InputRealPtrsType baro = ssGetInputPortRealSignalPtrs(S, 3);
        InputRealPtrsType time_ = ssGetInputPortRealSignalPtrs(S, 4);
        
        // set memory
        memset(buffer, 0, 1024);

        // create HIL_SENSOR message
        mavlink_hil_sensor_t hil_sensor_msg;
        CreateHILSensorMessage(&hil_sensor_msg, time_[0], acc[0], gyro[0], mag[0], baro[0]);
        mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &mavlinkMsg, &hil_sensor_msg);
        auto sendBytesLength = mavlink_msg_to_send_buffer(&buffer[0], &mavlinkMsg);
        
        // send the data
        real_T *sensorBuffer = ssGetOutputPortRealSignal(S, 0); 
        std::copy(buffer, buffer + sendBytesLength, sensorBuffer);
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

// Function: CreateHILSensorMessage
// Purpose:  Title is pretty self-explanatory
void CreateHILSensorMessage(mavlink_hil_sensor_t *sensorMsg, const real_T* const time_usec, const real_T* const accel, const real_T* const gyro, const real_T* const mag, const real_T* const baro)
{
    sensorMsg->time_usec = (uint64_t)((time_usec[0]) * 1e6);
    sensorMsg->xacc = (float)accel[0];
    sensorMsg->yacc = (float)accel[1];
    sensorMsg->zacc = (float)accel[2];
    sensorMsg->xgyro = (float)gyro[0];
    sensorMsg->ygyro = (float)gyro[1];
    sensorMsg->zgyro = (float)gyro[2];
    sensorMsg->xmag = (float)mag[0]*0.01;           // uT to Gauss
    sensorMsg->ymag = (float)mag[1]*0.01;
    sensorMsg->zmag = (float)mag[2]*0.01;
    sensorMsg->abs_pressure = (float)baro[0]*0.01;  // Pa to hPa
    sensorMsg->diff_pressure = (float)0;
    sensorMsg->temperature = (float)25;
    sensorMsg->fields_updated = (uint32_t)0x1FFF; // all fields are updated
    sensorMsg->id = (uint8_t)0;
}


#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif