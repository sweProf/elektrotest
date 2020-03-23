#include "crc16.h"


//////////////////////////////////////////////
// begin - functions implemented as defines //
///////////////////////////////////////////////////////////////////////////////////////////////////

// single step of polynom division
#define poldiv8B() \
   do\
{\
   *(uint64_t *) (meas2crc + 8) = \
   (( ~((uint64_t) (*(meas2crc + 15) >> 4) - 1) & poly16) ^ *(uint64_t *) (meas2crc + 8)) * 2;\
}\
while (0)
#define poldiv5B() \
   do\
{\
   *(uint64_t *) rcv = \
   (( ~( ((uint64_t)* (rcv + 5)) - 1) & (uint64_t) poly5) ^ *(uint64_t *) rcv) * 2;\
}\
while(0)

// calculates crc of 3 byte long cmd
#define crc16() \
   do\
{\
   crc = cmd;\
   crc <<= 3;\
   for ( int i=0 ; i<21 ; i++ )\
   {\
      crc <<= 1;\
      crc = ( ~( *p_1msg - 1 ) & poly3 ) ^ crc;\
   }\
   crc >>= 8;\
}\
while(0)

// constructs the full message from cmd and crc
#define cmsg()\
   do\
{\
   msg = (((long long) cmd << 16) | crc);\
}\
while(0)

// creates the command containing a variant and an ecu number
#define ccmd()\
   do\
{\
   cmd = ((variant + ecu) << 16) | (operation << 8) | 0xFF;\
}\
while(0)

// copies message to ask[] to be sent
#define cpy2tx() \
   do\
{\
   for ( int i=0 ; i<5 ; i++ ) *(ask + 4 - i) = *( ( (uint8_t *) &msg ) + i );\
}\
while(0)
///////////////////////////////////////////////////////////////////////////////////////////////////
// end - functions implemented as defines //
////////////////////////////////////////////


const unsigned int polynom = ( 1 << 16 ) | 0x1021;
const unsigned int poly3 = polynom << 8;
const uint64_t poly5 = (uint64_t) polynom << 24;
const uint64_t poly16 = (uint64_t) polynom << 44;
const uint8_t * p_1msg = (uint8_t *) (&crc) + 3;
const uint16_t * p_crc = (uint16_t *) &crc;

void askmsg()
{
   ccmd();
   crc16();
   cmsg();
   cpy2tx();
}

uint64_t crc16cmd()
{
//   uint64_t tst = 0x1511FF8622;
//   tst <<= 4;
   //    uint8_t rcv[8];
//   debug();
//   uint64_t tst = 0x1514FF79D4;
//   memcpy(rcv, &tst, 8);
   *(uint64_t *) rcv <<= 4;

   for ( int i=0 ; i<21 ; i++ ) poldiv5B();

   return *(uint64_t *) rcv;
}

// calculates if the crc16 from 16 byte long measurement data is valid
// returns 0 if crc16 is valid, otherwise a value != 0
uint16_t crc16meas()
{
   int i = 0;
   for ( ; i < (4 * 8) ; i++) poldiv8B();

   (*(uint64_t *) (meas2crc + 4)) <<= (4 * 8);
   (*(uint64_t *) meas2crc) <<= (4 * 8);

   for ( ; i < (8 * 8) ; i++) poldiv8B();

   (*(uint64_t *) (meas2crc + 4)) <<= (4 * 8);

   for ( ; i < (14 * 8 - 3) ; i++ ) poldiv8B();

   *(uint64_t *) (meas2crc + 8) <<= 3;
   return *(uint16_t *) (meas2crc + 14);
}
