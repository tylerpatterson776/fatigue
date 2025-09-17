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
float seconds = 1;

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



float getSample() {

  if (stop_now) {
    return 0;
  }
  ++sample;
  long long now     = currentTimeMillis();

  if (lastSampleTimestamp == 0) {
    lastSampleTimestamp = now;
  }

  float volts         = readVoltage(ADS1115_HANDLE);
  return volts;
}

float volts_to_displacement(float input){
	float output = input*5; 
	return output;}

void goto_voltage(float target, int chan, int handle, int &Kp){
while(!stop_now){
	int numtries = 0;
	float v=getSample();
	float v2;
	float val;
	float tol = 0.1;
	float error = (target - v);
	float last_val=0;
	while (!stop_now && (fabs(error) > tol)){ //This loop runs until error is within tolerance
		if (numtries >= 150 && error > 0){
			Kp = Kp + 1;
		}
		
		
		if (numtries >= 1660){
			stop_now = 1;
		}
		//usleep(100);
		v=getSample();
		unsigned long long timestamp = currentTimeMillis() - sampleStart;
		int gpio1status = gpioRead(GPIO1);
		int gpio2status = gpioRead(GPIO2);
		val = v;
		error = (target - val);
		numtries = numtries + 1;
		int PWM = clamp((fabs(error) * Kp),0,255);
		printf("numtries: %.4d timestamp: %llu target: %.4f   val: %.4f   last_val: %.4f   tol:%.4f   pwm:%.4d   error:%.4f   pos:   %.4f  stopnow: %.4d Kp: %.4d  GPIO1: %.1d   GPIO2: %.1d\n",numtries,timestamp,target,val,last_val,tol,PWM,error,v2*5,stop_now,Kp,gpio1status,gpio2status);
		if (error <= 0 ){ //this means overshoot
		printf("error <= 0\n");
		gpioWrite(GPIO1,0);
		PWM = 0;
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

		printf("Arrived at target: %.4f tol: %.4f   error: %.4f  \n",target,tol,error);

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
    //gpioSetPullUpDown(GPIO1, PI_PUD_DOWN);
  //  gpioSetPullUpDown(GPIO2, PI_PUD_DOWN);
    gpioSetPWMfrequency(GPIO1, 320);
    gpioSetPWMfrequency(GPIO2, 320);


      if (wiringPiSetup()!=0) {
    printf("cannot initialize WiringPi\n");
    return 1;
  }
  
  
  ADS1115_HANDLE = wiringPiI2CSetup(ADS1115_ADDRESS);
  float v1=readVoltageSingleShot(ADS1115_HANDLE, 1, 0);
  sleep(0.5);
  printf("Initial voltage on Load Cell:  %.4f \n",v1);
  printf("accessing ads1115 chip on i2c address 0x%02x\n", ADS1115_ADDRESS);
  ADS1115_HANDLE = wiringPiI2CSetup(ADS1115_ADDRESS);
  setADS1115ContinuousMode(ADS1115_HANDLE, 1, 0, 7);
  sampleStart=currentTimeMillis();
  int Kp = 600;

   while (!stop_now){
	   gpioWrite(GPIO1,1);
	   //gpioPWM(GPIO1,255);
	   //gpioPWM(GPIO2,230);
	  // gpioWrite(GPIO2,0);
	  sleep(1000);
	   /*
	goto_voltage(v1 +1.3, 1, ADS1115_HANDLE,Kp);
	goto_voltage(v1, 1, ADS1115_HANDLE,Kp);
	*/
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


    
