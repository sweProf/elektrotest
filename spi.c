/*
 * @autor       : Jens Herbert
 * @e-mail      : Jens.Herbert@ingeneers.de
 *
 * @description : spi.c module controls ECUs / base boards using SPI to perform electrical tests according to BMW GS 95024-2-1
 * @dependency  : libcrc.c for CRC check of received messages and to contruct messages to send to base board
 * @dependency  : libeval.c for evaluation of measurements BMW if requirements are met
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "crc16.h"
#include "meas.h"

#define PY_SSIZE_T_CLEAN
#include <Python.h>

#define select set_gpio_out

#define BOARD_1 17
#define BOARD_2 27
#define BOARD_3 22

#define DELAY_AFTER_CMD 5000

#define CRC_ERROR_TOL 16

#define b1_low()\
   do\
{\
   sprintf(py_set_gpio_low, "GPIO.output(%d, GPIO.LOW)\n", BOARD_1);\
   PyRun_SimpleString(py_set_gpio_low);\
   printf("GPIO LOW\n");\
}\
while(0)

#define b2_low()\
   do\
{\
   sprintf(py_set_gpio_low, "GPIO.output(%d, GPIO.LOW)\n", BOARD_2);\
   PyRun_SimpleString(py_set_gpio_low);\
   printf("GPIO LOW\n");\
}\
while(0)

#define b3_low()\
   do\
{\
   sprintf(py_set_gpio_low, "GPIO.output(%d, GPIO.LOW)\n", BOARD_3);\
   PyRun_SimpleString(py_set_gpio_low);\
   printf("GPIO LOW\n");\
}\
while(0)

#define b1_high()\
   do\
{\
   sprintf(py_set_gpio_high, "GPIO.output(%d, GPIO.HIGH)\n", BOARD_1);\
   PyRun_SimpleString(py_set_gpio_high);\
   printf("GPIO HIGH\n");\
}\
while(0)

#define b2_high()\
   do\
{\
   sprintf(py_set_gpio_high, "GPIO.output(%d, GPIO.HIGH)\n", BOARD_2);\
   PyRun_SimpleString(py_set_gpio_high);\
   printf("GPIO HIGH\n");\
}\
while(0)

#define b3_high()\
   do\
{\
   sprintf(py_set_gpio_high, "GPIO.output(%d, GPIO.HIGH)\n", BOARD_3);\
   PyRun_SimpleString(py_set_gpio_high);\
   printf("GPIO HIGH\n");\
}\
while(0)

// sends 5 Bytes
#define snd()\
   do\
{\
      usleep(DELAY_AFTER_CMD - 1000);\
      askmsg();\
      for (int k = 0; k < 5; k++)\
      {\
         tx = ask[k];\
         ans = ioctl(fd, SPI_IOC_MESSAGE(1), &tf);\
         if (ans < 1)\
            fail("can't send spi message");\
         usleep(1);\
      }\
      usleep(DELAY_AFTER_CMD - 3000);\
}\
while(0)

// sends request for measurement data and receive 16 Byte longs answer
#define tf_req()\
   do\
{\
      operation = REQ;\
      snd();\
      for (int k = 0; k < 16; k++)\
      {\
         tx = 0x00;\
         ans = ioctl(fd, SPI_IOC_MESSAGE(1), &tf);\
         if (ans < 1)\
            fail("can't send spi message");\
         printf("0x%.2X ", rx);\
         *(measData + 15 - k) = *(meas2crc + 15 - k) = rx;\
         usleep(1);\
      }\
      puts("");\
      usleep(DELAY_AFTER_CMD - 4000);\
}\
while(0)

// sends 5 Bytes long command and receives 5 Bytes long answer
#define exec5B()\
   do\
{\
      snd();\
      usleep(DELAY_AFTER_CMD + 6500);\
      \
      for (int k = 0; k < 5; k++)\
      {\
         tx = 0x00;\
         ans = ioctl(fd, SPI_IOC_MESSAGE(1), &tf);\
         if (ans < 1)\
            fail("can't send spi message");\
         printf("0x%.2X ", rx);\
         *(rcv + 4 - k) = rx;\
         usleep(1);\
      }\
      puts("");\
      validans();\
      \
      usleep(DELAY_AFTER_CMD);\
}\
while(0)
      
// parses measurement data to variables for evaluation and log
#define parseMeas()\
   do\
{\
      *(t + i) = (mtime = gts() - st);\
      *(pv + i) = (pump_voltage = 0.1 * (float) *(measData + 13));\
      pressure = *(uint16_t *) (measData + 11);\
      if ( ( (*(measData + 10) ) < 0x80) ) supply_current = 0.001; else supply_current = 1;\
      *(rsc + i) = (supply_current *= (double) 0.001 * ((*(uint16_t *) (measData + 9)) & 0x7FFF));\
      cl30_voltage = 0.1 * *(measData + 8);\
}\
while(0)

#define spifail()\
do\
{\
   printf("0x%lX\n", *(uint64_t *) meas2crc);\
   printf("0x%lX\n", *( ((uint64_t *) meas2crc) + 1));\
   if ( ( (*(uint64_t *) meas2crc) & (*( ((uint64_t *) meas2crc) + 1)) ) == ~((uint64_t) 0) )\
   {\
      usleep(2000);\
      printf("FAIL SPI\n");\
   }\
   else\
   {\
      printf("SPI OK\n");\
      usleep(0);\
   }\
}\
while(0)

#define validans()\
   do\
{\
   if (*(rcv + 4) != (variant + ecu)) printf("!!!NAD_WRONG!!!\n");\
   switch (*(rcv + 3))\
   {\
      case 0xA5: printf("!!!NEGATIVE_RESPONSE_AFTER_0x%X_COMMAND!!!\n", operation); break;\
      case 0x00: printf("!!!NO_RESPONSE_AFTER_0x%X_COMMAND!!!\n", operation); break;\
      case 0x5A: printf("POSITIVE_RESPONSE_AFTER_0x%X_COMMAND\n", operation); setRefTime(); break;\
      default: printf("!!!MSG_ID_WRONG!!!\n");\
   }\
   validCrc8B();\
}\
while(0)

// validates CRC in answer message, tries to fix last two bits if at least one of them is broken / SPI error correction
#define validCrc8B()\
   do\
{\
   uint8_t cp_rcv[8];\
   memcpy(cp_rcv , rcv , 8);\
   int crcfail = 0;\
   for (int a=1 ; crc16cmd() ; a++)\
   {\
      memcpy(rcv , cp_rcv , 8);\
      *(rcv) = (*(rcv)) - a;\
      if (a > CRC_ERROR_TOL)\
      {\
         crcfail = 1;\
         break;\
      }\
   }\
   if(crcfail)\
   {\
      printf("!!!CRC_FAIL!!!\n");\
      resetPy();\
      continue;\
   }\
}\
while(0)

// validates answer message in RX of SPI: NAD , MSG_ID , CRC
#define validmeas()\
   do\
{\
   if (*(meas2crc + 15) != (variant + ecu))\
   printf("!!!NAD_WRONG!!!\n");\
   if (*(meas2crc + 14) != 0xA1)\
   printf("!!!MSG_ID_WRONG!!!\n");\
   validCrc16B();\
}\
while(0)

// validates CRC from RX of SPI, tries to fix last two bits if at least one of them is broken / SPI error correction
#define validCrc16B()\
   do\
{\
   int crcfail = 0;\
   for (int b=1 ; crc16meas() ; b++)\
   {\
      memcpy(meas2crc , measData , 16);\
      *(meas2crc) = (*(measData)) - b;\
      if (b > CRC_ERROR_TOL)\
      {\
         crcfail = 1;\
         break;\
      }\
   }\
   if(crcfail)\
   {\
      printf("!!!CRC_FAIL!!!\n");\
      resetPy();\
      fail("!!!!!SPI_FAIL!!!!!\n");\
      continue;\
   }\
}\
while(0)

// prints measurement values
#define printMeas()\
   do\
{\
   printf("\n            pump_U   = %f\n\
            p        = %d\n\
            supply_I = %lf\n\
            cl30_U   = %f\n",\
            pump_voltage,\
            pressure,\
            supply_current,\
            cl30_voltage);\
}\
while(0)


void fail(char *s)
{
   closelog();
   perror(s);
   abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 2 * 100 * 1000; // 200KHz advised
static uint16_t delay = 0;

static uint8_t gpio_out;
static char py_set_gpio_high[25]; //GPIO.output(22, GPIO.HIGH)\n
static char py_set_gpio_low[24];  //GPIO.output(22, GPIO.LOW)\n

int ans;                         // answer from spi device
uint8_t tx;                      // transmission byte to be stored
uint8_t rx;                      // receive byte to be stored
int fd;                          // file descriptor to spi device
int op;                          // actual operation after operation to inactivate switch


uint8_t set_gpio_out(uint8_t board)
{
   return gpio_out = board;
}

void demand(int action)
{
   op = action;
   transfer();
}

// selects (v)ariant and (e)cu
void slct(int v, int e)
{
   variant = v;
   ecu = e;
}

void transfer()
{
   struct spi_ioc_transfer tf = {
      .tx_buf = (unsigned long) &tx,
      .rx_buf = (unsigned long) &rx,
      .len = 1,
      .delay_usecs = delay,
      .speed_hz = speed,
      .bits_per_word = bits,
   };

///////////////////////////////////////////////
//// for debugging of 16 Bytes long answer   //
///////////////////////////////////////////////
//   printf("ANTWORT BYTES GET MEASUREMENT\n");
//   printf("NAD , MSGID , pump_voltage , pressure , supply_current , cl30_voltage , m_DIAG_01_content , 0xFF , CRC16B1 , CRC16B2\n1\t1\t1\t\t2\t\t2\t\t1\t\t5\t\t1\t1\t1\n");
///////////////////////////////////////////////

   if (op == REQ)
   {
      for(int i = 0; i < MEAS_CYCLES; i++)
      {
         // demands measurement data
         tf_req();

         //      //////////////////////////////////////
         //      // for debuging without working spi //
         //      //////////////////////////////////////
         //      debug();
         //      unsigned octaword tst = (uint128) 0x0CA100001F80E671 << 64 | (uint64_t) 0xFFFFFFFFFFFFEDDD;
         //      //unsigned octaword tst = (uint128) 0x15a1000020801c6f << 64 | (uint64_t) 0xffffffffffff9908; 
         //      memcpy(meas2crc, &tst, 16); 
         //      for (int k = 0; k < 16; k++)
         //         *(measData + k) = *(meas2crc + k);
         //      //////////////////////////////////////

         //   spifail();

         validmeas();   // validates measurement message from from SPI RX (e.g. CRC)

         parseMeas();   // parse measurement values from SPI RX message
         //      printMeas(); // for debugging measurement values
         wlog();        // print measurement values to log file

      }
      // evaluate electrical requirements according to BMW document GS 95024-2-1
      eval(); 
   }
   else
   {

      // resets switches before operation itself
      if ( (variant ==  COMBI) || (variant == BOLSTER) )
      {
         operation = BIA;
         exec5B();
      }
      if ( (variant ==  COMBI) || (variant == LUMBAR) )
      {
         operation = LIA;
         exec5B();
      }
 
      usleep(16000);

      // excecutes operation itself -> UP,DOWN,FOR,BACK,INFLATE,DEFLATE
      action = operation = op;
      exec5B();

//      // logs time stamp after receiving answer to last command
//      wtlog();
   }
}

int main(int argc, char *argv[])
{
   wchar_t *program = Py_DecodeLocale(argv[0], NULL);
   if (program == NULL) {
      fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
      exit(1);
   }
   Py_SetProgramName(program);  /* optional but recommended */
   Py_Initialize();
   PyRun_SimpleString("import sys\n"
         "print(sys.path)\n"
         "import RPi.GPIO as GPIO\n"
	 "GPIO.setmode(GPIO.BCM)\n"
	 "GPIO.setup(17, GPIO.OUT)\n"
	 "GPIO.setup(22, GPIO.OUT)\n"
	 "GPIO.setup(27, GPIO.OUT)\n"
	 "GPIO.setup(24, GPIO.IN)\n");
//   Py_Finalize();


   fd = open(device, O_RDWR | O_NONBLOCK);
   if (fd < 0)
      fail("can't open device");
/*
   for (int j=0 ; j < 100 ; j++)
   {
      b1_low();
      sleep(1);
      b1_high();
   }
*/


   //TODO TEST CRC @ OP MODE 2A
/*
   b1_low();
   slct(COMBI,5);

   setpath();
   
   openlog();
   demand(REQ);
   closelog();
   
   b1_high();
*/
/*
   // COMBI
   b1_low();
   slct(COMBI,5);

   setpath();
   openlog();
//   while (1)
   {
      demand(FOR);
      demand(REQ);

      demand(BCK);
      demand(REQ);

      demand(INF);
      demand(REQ);

      demand(DEF);
      demand(REQ);

      demand(UP);
      demand(REQ);

      demand(DWN);
      demand(REQ);

      demand(BCK);
      demand(REQ);
   }
   b1_high();
   closelog();


   // BOLSTER
   b2_low();
   slct(BOLSTER,6);

   setpath();
   openlog();

   demand(INF);
   demand(REQ);

   demand(DEF);
   demand(REQ);

   b2_high();
   closelog();
*/

   // LUMBAR
   b3_low();
   slct(LUMBAR,2);

   setTestStartTime();
   while(1)
   {
   setpath();
   openlog();

   demand(FOR);
   demand(REQ);

   demand(BCK);
   demand(REQ);

   demand(UP);
   demand(REQ);

   demand(DWN);
   demand(REQ);
   
   if((gts() - getTestStartTime()) > (9000 + 60)) break;
   }
   b3_high();
   closelog();


   close(fd);

   PyRun_SimpleString("GPIO.cleanup()\n");

   return 0;
}
