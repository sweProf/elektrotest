#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "crc16.h"

static double startts; // start timestamp
static struct timeval tv;
static char path[50];// = "logs/combi_1_2020.log";
static char dut[10];

static double testStartTime;

void setpath();
void setDUT();
void openlog();
void closelog();
void wlog();
void wtlog();
