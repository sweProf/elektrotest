#include "eval.h"
#include "meas.h"
#include "crc16.h"

// sets start time
#define sst() st=gts();
#define TOLERANCE  0.2

#define FORBIDDEN  2
#define HYSTERESIS 1
#define NORMAL     0


double voltmin       =    9 + TOLERANCE; //    9V -> minimum clamp30 voltage for the DUT to operate
double voltmax       =   16 - TOLERANCE; //   16V -> maximum clamp30 voltage for the DUT to operate
double voltminhyst   =  8.5 - TOLERANCE; //  8.5V -> minimum clamp30 voltage for hysteresis area
double voltmaxhyst   = 16.5 + TOLERANCE; // 16.5V -> maximum clamp30 voltage for hysteresis area
int hyst;                                // hysteresis 1 (active) or 0 (inactive)
int forb;                                // forbidden area 1 (overvoltage / undervoltage) or 0
int state;                               // current state
// checks which current state now must be set in II.c mode
#define stateIIc()\
   do\
{\
   if ( (cl30_voltage >= voltmin) && (cl30_voltage <= voltmax) )\
   {\
      if (state) setRefTime();\
      state = NORMAL;\
   }\
   else if ( (cl30_voltage >= voltminhyst) && (voltmaxhyst >= cl30_voltage) )\
   {\
      if (state) state = HYSTERESIS;\
   }\
   else\
   {\
      state = FORBIDDEN;\
   }\
}\
while(0)

// averages raw measured current, each value is averaged over current index and its both neighbours left and right
#define avgcur()\
   do\
{\
   for (int i=1 ; i < ( MEAS_CYCLES - 1 ) ; i++)\
   *(asc + i) = ( *(rsc + i - 1) + *(rsc + i) + *(rsc + i + 1) ) / 3;\
   *asc = (*(rsc + 1) + *rsc) * 0.5;\
   *(asc + MEAS_CYCLES - 1) = ( *(rsc + MEAS_CYCLES - 1) + *(rsc + MEAS_CYCLES - 2) ) * 0.5;\
   printf("\n");\
   for (int i=0 ; i < MEAS_CYCLES ; i++)\
   printf("%lf ",*(rsc + i));\
   printf("\n");\
   for (int i=0 ; i < MEAS_CYCLES ; i++)\
   printf("%lf ",*(asc + i));\
}\
while (0)

double maxcur = 2.5; // maximum overall current 2.5A
// checks maximum current
#define mc()\
   do\
{\
   if (supply_current > maxcur)\
   {\
      fprintf( lf , "current more than %lfA!\n", maxcur);\
      printf("current more than %lfA!\n", maxcur);\
   }\
}\
while(0)

double refTime;
void setRefTime()
{
   refTime = gts();
}

int stateBefore = NORMAL;
// checks time conditions when switching on
// TODO

#define MAX_TIME_VALVE 0.2 // valve must be active after          200ms
#define MAX_TIME_PUMP  0.2 // pump  must be active after          400ms
#define MIN_TIME_PUMP  0.2 // pump  must be active after at least 200ms after valve
#define tc()\
   do\
{\
   printf("TODO\n");\
   if ((action == BCK) || (action == DEF))\
   {\
      if ((gts() - refTime) < 0.2)\
         printf("\ncheck valve\n");\
   }\
}\
while(0)

// evaluates if DUT meets all conditions
void eval()
{
   avgcur();   // average current
   mc();       // maximum current
   stateIIc(); // set currently required state meeting II.c conditions
   tc();       // time conditions
}

void setpath()
{
   time_t tnow;
   time(&tnow);
   struct tm * stime = localtime(&tnow); // only for tagging log file
//   variant = COMBI;
//   ecu = 2;
   setDUT();
   sprintf(path, "logs/%s_%d%02d%02d_%02d%02d.log\0", dut, 1900 + stime->tm_year, stime->tm_mon + 1, stime->tm_mday, stime->tm_hour, stime->tm_min);
   sst(); // start time for measurements
}

void setDUT()
{
   switch(variant)
   {
      case(LUMBAR):
         sprintf(dut, "lumbar_%d", ecu); break;
      case(BOLSTER):
         sprintf(dut, "bolster_%d", ecu); break;
      case(COMBI):
         sprintf(dut, "combi_%d", ecu); break;
      default:
         sprintf(dut, "unknown_%d", ecu);
   }
}

void openlog()
{
   setpath();
   lf = fopen(path, "a");
   fprintf( lf , "time, pump_voltage , pressure , supply_current , cl30_voltage" );
}

void closelog()
{
   fclose(lf);
}

void wtlog()
{
   fprintf( lf , "\n%lf" , gts() - st );
}

// write erroor to log file
void elog()
{
   fprintf( lf , "ELECTRICAL_TEST_FAIL " );
}

// write measurement values to log file
void wlog()
{
   fprintf( lf , "\n%13.6lf,%9.6lf,%d,%8.6lf,%9.6lf" , mtime , pump_voltage , pressure , supply_current , cl30_voltage );
}

// gets current time in seconds, microseconds
double gts()
{
    gettimeofday(&tv,NULL);
    return tv.tv_sec + ((double) tv.tv_usec)/1000000;
}

void setTestStartTime()
{
   testStartTime = gts();
}

double getTestStartTime()
{
   return testStartTime;
}
