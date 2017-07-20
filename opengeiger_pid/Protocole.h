#include "Arduino.h"
#include "RFduinoBLE.h"

/** In packets types **/

#define IN_PACKET_STEALTH                   0x01
#define IN_PACKET_SILENT                    0x02
#define IN_PACKET_SET_TENSION               0x11
#define IN_PACKET_SEND_INFO                 0x12

/** Out packets types **/

#define OUT_PACKET_SERIAL_NB                0x01
#define OUT_PACKET_VERSION                  0x02
#define OUT_PACKET_SENSOR_TYPE              0x03
#define OUT_PACKET_ALIM_TENSION             0x04
#define OUT_PACKET_COUNT                    0x05
#define OUT_PACKET_TEMPERATURE              0x06

#define OUT_PACKET_TUBE_TYPE                0x10
#define OUT_PACKET_NOMINAL_TENSION          0x11
#define OUT_PACKET_ACTUAL_TENSION           0x12
#define OUT_PACKET_PWM_DUTY_CYCLE           0x13
#define OUT_PACKET_CALIB_COEFF              0x14

#define OUT_PACKET_DEBUG_BYTE1              0xD1
#define OUT_PACKET_DEBUG_BYTE2              0xD2
#define OUT_PACKET_DEBUG_FLOAT1             0xF1
#define OUT_PACKET_DEBUG_FLOAT2             0xF2
#define OUT_PACKET_DEBUG_STRING1            0xE1
#define OUT_PACKET_DEBUG_STRING2            0xE2

/** Send Methods **/

void sendFloat32Packet(byte type, float value);
void sendUint8Packet(byte key, byte value);
void sendLenString(byte key, char* str, int len);
void sendBuffer();


/** Receive Helpers **/

float decodeFloat(char *data, int offset);

