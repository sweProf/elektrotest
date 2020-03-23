#define MEAS_CYCLES 500

double t[MEAS_CYCLES];     // time stamps at measurement samples
double pv[MEAS_CYCLES];    // pump voltage
double rsc[MEAS_CYCLES];   // raw supply current measurement, to be averaged
double asc[MEAS_CYCLES];   // avg supply current measurement 

double st;
double mtime;
double pump_voltage; 
int pressure;
double supply_current;
double cl30_voltage;

void swtch(int schalter);
void slct(int v, int e);

double gts();
void setRefTime();
void setpath();
void setDUT();
void openlog();
void closelog();
void wlog();
void elog();
void wtlog();
void eval();

FILE * lf; // logfile
int action;
