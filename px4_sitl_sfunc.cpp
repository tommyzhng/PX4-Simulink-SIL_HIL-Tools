/*
 * C++ S-Function for PX4 SITL with Mavlink Conversions
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

// declare helper functions
static mavlink_message_t mavlinkMsg;
static mavlink_hil_actuator_controls_t hilActuatorControlsMsg;
void CreateHeartbeatMessage(mavlink_heartbeat_t *heartbeatMsg);
void CreateHILSensorMessage(mavlink_hil_sensor_t *sensorMsg, const real_T* const time_usec, const real_T* const accel, const real_T* const gyro, const real_T* const mag, const real_T* const baro);
void CreateHILGPSMessage(mavlink_hil_gps_t *gpsMsg, const real_T* const time_usec, const real_T* const LLA, const real_T* const velocity, const real_T* const gndSpeed, const real_T* const course);
void CreateRCInputsMessage(mavlink_hil_rc_inputs_raw_t *rcMsg, const real_T* const time_usec, const real_T* const rc);

// Function: mdlInitializeSizes
// Purpose:  Initialize the sizes array
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)){
        return;
    }
    if (!ssSetNumInputPorts(S, 10)){
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
    // GPS
    ssSetInputPortWidth(S, 4, 3); // lat,long,alt measured
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortWidth(S, 5, 3); // xyz velocity
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortWidth(S, 6, 1); // Ground Speed
    ssSetInputPortDirectFeedThrough(S, 6, 1);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortWidth(S, 7, 1); // Course
    ssSetInputPortDirectFeedThrough(S, 7, 1);
    ssSetInputPortDataType(S, 7, SS_DOUBLE);
    // Time
    ssSetInputPortWidth(S, 8, 1);
    ssSetInputPortDirectFeedThrough(S, 9, 1);
    ssSetInputPortDataType(S, 9, SS_DOUBLE);
    // RC
    // ssSetInputPortWidth(S, 8, 4); // 4 channels
    // ssSetInputPortDirectFeedThrough(S, 8, 1);
    // ssSetInputPortDataType(S, 8, SS_DOUBLE);

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
        // get the socket and buffer from the PWork vector
        asio::ip::tcp::socket *socket_ = (asio::ip::tcp::socket *)(ssGetPWorkValue(S, 0));
        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 1);

        if (buffer == NULL || socket_ == NULL){
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
        InputRealPtrsType time_ = ssGetInputPortRealSignalPtrs(S, 8);
        // InputRealPtrsType rc = ssGetInputPortRealSignalPtrs(S, 9);
        
        // send data to PX4 SITL
        memset(buffer, 0, 1024);

        // create heartbeat message
        mavlink_heartbeat_t heartbeat_msg;
        CreateHeartbeatMessage(&heartbeat_msg);
        mavlink_msg_heartbeat_encode_chan(1, 200, MAVLINK_COMM_0, &mavlinkMsg, &heartbeat_msg);
        auto sendBytesLength = mavlink_msg_to_send_buffer(&buffer[0], &mavlinkMsg);

        // create HIL_SENSOR message
        mavlink_hil_sensor_t hil_sensor_msg;
        CreateHILSensorMessage(&hil_sensor_msg, time_[0], acc[0], gyro[0], mag[0], baro[0]);
        mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &mavlinkMsg, &hil_sensor_msg);
        sendBytesLength += mavlink_msg_to_send_buffer(&buffer[sendBytesLength], &mavlinkMsg);

        // create HIL_GPS message
        mavlink_hil_gps_t hil_gps_msg;
        CreateHILGPSMessage(&hil_gps_msg, time_[0], LLA[0], velocity[0], gndSpeed[0], course[0]);
        mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &mavlinkMsg, &hil_gps_msg);
        sendBytesLength += mavlink_msg_to_send_buffer(&buffer[sendBytesLength], &mavlinkMsg);

        // create HIL_RC_INPUTS_RAW message
        // mavlink_hil_rc_inputs_raw_t hil_rc_msg;
        // CreateRCInputsMessage(&hil_rc_msg, time_[0], rc[0]);
        // mavlink_msg_hil_rc_inputs_raw_encode_chan(1, 200, MAVLINK_COMM_0, &mavlinkMsg, &hil_rc_msg);
        // sendBytesLength += mavlink_msg_to_send_buffer(&buffer[sendBytesLength], &mavlinkMsg);

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

// Helper functions to create mavlink messages

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

// Function: CreateRCInputsMessage
// Purpose: Title is pretty self-explanatory
void CreateRCInputsMessage(mavlink_hil_rc_inputs_raw_t *rcMsg, const real_T* const time_usec, const real_T* const rc)
{
    rcMsg->time_usec = (uint64_t)((time_usec[0]) * 1e6);
    rcMsg->chan1_raw = (uint16_t)rc[0];
    rcMsg->chan2_raw = (uint16_t)rc[1];
    rcMsg->chan3_raw = (uint16_t)rc[2];
    rcMsg->chan4_raw = (uint16_t)rc[3];
    rcMsg->chan5_raw = (uint16_t)1000;
    rcMsg->chan6_raw = (uint16_t)1000;
    rcMsg->chan7_raw = (uint16_t)1000;
    rcMsg->chan8_raw = (uint16_t)1000;
    rcMsg->chan9_raw = (uint16_t)1000;
    rcMsg->chan10_raw = (uint16_t)1000;
    rcMsg->chan11_raw = (uint16_t)1000;
    rcMsg->chan12_raw = (uint16_t)1000;
    rcMsg->rssi = (uint8_t)255;   
}


#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif