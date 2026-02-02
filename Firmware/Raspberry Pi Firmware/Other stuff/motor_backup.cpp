#include <pigpio.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include "ads1115rpi.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <byteswap.h>
#include <vector>
#include <ctype.h>

#define GPIO1 17
#define GPIO2 27

using namespace std;

int abort_test(void){
    printf("Test aborted\n");
    
    gpioWrite(GPIO1,0); 
    gpioWrite(GPIO2,0);
    gpioTerminate(); 
    exit(0);
}




static volatile sig_atomic_t stop_now = 0;

static void sigint_handler(int sig)
{
    (void)sig;        /* silence -Wunused-parameter */
    stop_now = 1;     /* async-signal-safe: just set a flag */
}


int ADS1115_ADDRESS=0x48;
int ADS1115_HANDLE=-1;
float vRef = 5.0;
int   gain = 0;


long long sample=-1;
long long sampleStart=0;
long long lastSampleTimestamp=0;

unsigned long long currentTimeMillis() {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    return (unsigned long long)(currentTime.tv_sec) * 1000 +
        (unsigned long long)(currentTime.tv_usec) / 1000;
}


static inline float clamp(float v, float lo, float hi){
	if (v>hi){
		v = hi;
		return v;
		}
	if (v<lo){
		v=lo;
		return v;
	}
	return v;
	
}


class Sample {
  public:
  long long sample;
  long long timestamp;
  long long delta;
  long long offset;
  float     volts;
};

vector<Sample*> samples;



void getSample() {

  if (stop_now) {
    return;
  }
  ++sample;
  long long now     = currentTimeMillis();

  if (lastSampleTimestamp == 0) {
    lastSampleTimestamp = now;
  }

  long long offset    = now - sampleStart;
  long long delta     = now - lastSampleTimestamp;
  lastSampleTimestamp = now;
  float volts         = readVoltage(ADS1115_HANDLE);

  Sample *s = new Sample();

  s->sample     = sample;
  s->timestamp  = now;
  s->delta      = delta;
  s->offset     = offset;
  s->volts      = volts;

  samples.push_back(s);
}

float volts_to_displacement(float input){
	float output = input*5; 
	return output;}

void goto_voltage(float target, int chan, int handle){
while(!stop_now){
	int numtries = 0;
	float v=readVoltageSingleShot(handle, chan, gain);
	float v2;
	float val;
	float tol = 0.1;
	float error = (target - v);
	float last_val=0;
	int Kp = 1000; //gain control

	
	while (!stop_now && (fabs(error) > tol)){ //This loop runs until error is within tolerance
		if (numtries >= 10){
			stop_now = 1;
		}
		v=readVoltageSingleShot(handle, chan, gain);
		//v2 = readVoltageSingleShot(handle, 0, gain); //distance measurement
		int gpio1status = gpioRead(GPIO1);
		int gpio2status = gpioRead(GPIO2);
		val = v;
		error = (target - val);
		numtries = numtries + 1;
		int PWM = clamp((fabs(error) * Kp),200,255);
		PWM = 255;
		printf("numtries: %.4d  target: %.4f   val: %.4f   last_val: %.4f   tol:%.4f   pwm:%.4d   error:%.4f   pos:   %.4f  stopnow: %.4d  GPIO1: %.1d   GPIO2: %.1d\n",numtries,target,val,last_val,tol,PWM,error,v2*5,stop_now,gpio1status,gpio2status);
		if (error <= 0 ){
		printf("error <= 0\n");
		gpioWrite(GPIO1,0);
		gpioPWM(GPIO2,PWM);	
	}
		else if (error > 0) {
			gpioWrite(GPIO2,0);
			printf("error > 0\n");
			gpioPWM(GPIO1,PWM);
		
	}
		last_val = val;
		
	}  
	if (stop_now) {
	//printf("test"); 
    abort_test(); 
    exit(0); 
}

		printf("Arrived at target:  tol: %.4f   error: %.4f  \n",target,tol,error);
		gpioWrite(GPIO2,0);
		gpioWrite(GPIO1,0);
		return;
}
	//printf("test"); 
    abort_test(); 
    exit(0); 
}


int main(void)
{
	
	if (gpioInitialise() < 0)
	{	printf("pigpio initialisation failed \n");}
	else
	{printf("pigpio initialisation OK\n");}
    setbuf(stdout,NULL);
	signal(SIGINT, sigint_handler);
    gpioSetMode(GPIO1, PI_OUTPUT); 
    gpioSetMode(GPIO2, PI_OUTPUT);
    gpioSetPullUpDown(GPIO1, 1);
    gpioSetPullUpDown(GPIO2, 1);
    gpioWrite(GPIO2,0);
	gpioWrite(GPIO1,0);

      if (wiringPiSetup()!=0) {
    printf("cannot initialize WiringPi\n");
    return 1;
  }
  float v0;
  float v1;
  
  printf("accessing ads1115 chip on i2c address 0x%02x\n", ADS1115_ADDRESS);
  int ADS1115_HANDLE = getADS1115Handle(ADS1115_ADDRESS);
  v0=readVoltageSingleShot(ADS1115_HANDLE, 0, gain);
  v1=readVoltageSingleShot(ADS1115_HANDLE, 1, gain);
  printf("voltage on 0: %.4f \n",v0);
  printf("voltage on 1: %.4f \n",v1);

   float max=getADS1115MaxGain(gain);
   while (1){
	   
	goto_voltage(v1 +.15, 1, ADS1115_HANDLE);
	goto_voltage(v1 , 1, ADS1115_HANDLE);
}


 abort_test();
 return 0; 
}
  
  

// o = operation mode
// x = mux
// g = gain
// m = mode
//                oxxx gggm
// default 0x8583 1000 0101 1000 0011
//                1111 0101 1000 0011
//                1111 0101 1000 0011


    
