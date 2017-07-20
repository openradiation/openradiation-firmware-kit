#include "Protocole.h"

union _intToChar {
  int i;
  char c[2];
} intToChar_pro;

union _floatToChar {
  float f;
  char c[4];
} floatToChar_pro;

#define TX_BUFFER_MAX_LENGTH  20
char txBuff[TX_BUFFER_MAX_LENGTH];
int txBuffLength = 0;

void sendFloat32Packet(byte type, float value) {
  if ((txBuffLength+5) > TX_BUFFER_MAX_LENGTH) sendBuffer();
  floatToChar_pro.f = value;
  txBuff[txBuffLength++] = type;
  txBuff[txBuffLength++] = floatToChar_pro.c[0];
  txBuff[txBuffLength++] = floatToChar_pro.c[1];
  txBuff[txBuffLength++] = floatToChar_pro.c[2];
  txBuff[txBuffLength++] = floatToChar_pro.c[3];
}

void sendUint8Packet(byte key, byte value) {
  if ((txBuffLength+2) > TX_BUFFER_MAX_LENGTH) sendBuffer();
  txBuff[txBuffLength++] = key;
  txBuff[txBuffLength++] = value;
}

void sendLenString(byte key, char* str, int len) {
  if ((txBuffLength+2+len) > TX_BUFFER_MAX_LENGTH) sendBuffer();
  txBuff[txBuffLength++] = key;
  txBuff[txBuffLength++] = (byte) len;
  for(int i=0 ; i<len ; i++) {
    txBuff[txBuffLength++] = str[i];
  }
}

//send the current buffer and reset the cursor to 0
void sendBuffer() {
  RFduinoBLE.send(txBuff, txBuffLength);
  txBuffLength = 0;
}

// convert a 4 bytes array into a float, using IEEE754 Single precision 32-bit
float decodeFloat(char *data, int offset) {
  floatToChar_pro.c[0] = data[offset];
  floatToChar_pro.c[1] = data[offset+1];
  floatToChar_pro.c[2] = data[offset+2];
  floatToChar_pro.c[3] = data[offset+3];
  return floatToChar_pro.f;
}


