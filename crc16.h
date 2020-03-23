#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#define octaword __int128
#define uint128 __int128_t

// variants
#define BOLSTER 0x00
#define LUMBAR  0x0A
#define COMBI   0x14

// short cuts
#define REQ REQUEST_MEASUREMENT_DATA   
#define UP  SET_LUMBAR_SWITCH_UP       
#define DWN SET_LUMBAR_SWITCH_DOWN    
#define FOR SET_LUMBAR_SWITCH_FOR     
#define BCK SET_LUMBAR_SWITCH_BACK    
#define LIA SET_LUMBAR_SWITCH_INACTIVE
#define INF SET_LBV_SWITCH_INFLATE    
#define DEF SET_LBV_SWITCH_DEFLATE    
#define BIA SET_LBV_SWITCH_INACTIVE   

// operations
#define REQUEST_MEASUREMENT_DATA   0x11
#define SET_LUMBAR_SWITCH_UP       0x12
#define SET_LUMBAR_SWITCH_DOWN     0x13
#define SET_LUMBAR_SWITCH_FOR      0x14
#define SET_LUMBAR_SWITCH_BACK     0x15
#define SET_LUMBAR_SWITCH_INACTIVE 0x16
#define SET_LBV_SWITCH_INFLATE     0x17
#define SET_LBV_SWITCH_DEFLATE     0x18
#define SET_LBV_SWITCH_INACTIVE    0x19

#define debug() printf("DEBUGMODE_ACTIVE_FOR_SELFTEST\n");

int ecu;
int variant;
int operation;

const unsigned int polynom;
const unsigned int poly3;
const uint64_t poly5;
const uint64_t poly16;
unsigned int crc;
unsigned int cmd;
unsigned long long msg;
const uint8_t * p_1msg;
const uint16_t * p_crc;
uint8_t meas2crc[16];
uint8_t measData[16];
uint8_t rcv[8];
uint8_t ask[5];

uint16_t crc16meas();
uint64_t crc16cmd();
void askmsg();
void transfer();
void setTestStartTime();
double getTestStartTime();
void fail();
