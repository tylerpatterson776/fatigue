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
#include <wiringSerial.h>
#include <byteswap.h>
#include <vector>
#include <ctype.h>
#include <thread>
#include <cstdint>
#include <array>
#include <atomic>
#include <optional>
#include <sstream>
#include <thread>
#include "ring.h"

#define GPIO1 27
#define GPIO2 17

const float lbsPerVolt = 4.9165; //change this is load cell is recalibrated
const float newtonsPerVolt = 21.869; //change this is load cell is recalibrated
const float kgPerVolt = 2.2301;
const float mmPerVolt = 5;

std::uint64_t currentTimeMillis();

const int LOADCELL_ADS1115_ADDRESS=0x48;
const int LASER_ADS1115_ADDRESS=0x49;
int LOADCELL_ADS1115_HANDLE;
int LASER_ADS1115_HANDLE;

float vRef = 5.0;
int   gain = 0;
int serial;
  
long long sample=-1;
long long sampleStart=0;
long long lastSampleTimestamp=0;
float seconds = 1;


float voltsToPounds(float input, float reference, float conversionFactor = lbsPerVolt){ float result = (input - reference)*2*conversionFactor; return result;} //the *2 is to account for the fact we are voltage dividing 
float voltsToNewtons(float input, float reference, float conversionFactor = newtonsPerVolt){ float result = (input - reference)*2*conversionFactor; return result;}
float voltsToKg(float input, float reference, float conversionFactor = kgPerVolt){ float result = (input - reference)*2*conversionFactor; return result;}
float voltsToMM(float input, float reference, float conversionFactor = mmPerVolt){ float result = (input - reference)*conversionFactor; return result;}
float getSample(int HANDLE);

static volatile sig_atomic_t stop_now = 0;



int abort_test(void){
    printf("Test aborted\n");
	close(serial);
    gpioWrite(GPIO1,0); 
    gpioWrite(GPIO2,0);
    gpioTerminate(); 
    exit(0);
}


static void sigint_handler(int sig)
{
    (void)sig;        /* silence -Wunused-parameter */
    stop_now = 1;     /* async-signal-safe: just set a flag */
}

void measure(float loadcellReference, int loadcellHandle, float laserReference, int laserHandle, AdcBuf &buf)
{
	std::uint64_t sampleStart = currentTimeMillis();
	while(!stop_now){
		std::uint64_t time = currentTimeMillis();
		std::uint64_t time2 = (time - sampleStart);
		float loadcellSample = getSample(loadcellHandle);
		float laserSample = getSample(laserHandle); 
		buf.push(Sample{ADC::ADC1,loadcellSample,time2});
		buf.push(Sample{ADC::ADC2,laserSample,time2});	
		
	}
	return;
}
void send(AdcBuf &buf){
	while (!stop_now || !buf.empty())
	{
	const auto sample = buf.pop(true);
	serialPuts(serial,sample.to_string().c_str());

	};
	
	
	return;
	}



std::uint64_t currentTimeMillis() {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    return (std::uint64_t)(currentTime.tv_sec) * 1000 +
        (std::uint64_t)(currentTime.tv_usec) / 1000;
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

float getSample(int HANDLE) {

  if (stop_now) {
    return 0;
  }
  ++sample;
  long long now     = currentTimeMillis();

  if (lastSampleTimestamp == 0) {
    lastSampleTimestamp = now;
  }

  float volts         = readVoltage(HANDLE);
  return volts;
}

void goto_voltage(float target, int chan, int handle, int &Kp){
while(!stop_now){
	int numtries = 0;
	float v=getSample(handle);
	float v2;
	float val;
	float tol = 0.1;
	float error = (target - v);
	float last_val=0;
	while (!stop_now && (fabs(error) > tol)){ //This loop runs until error is within tolerance
		/*
		if (numtries >= 150 && error > 0){
			Kp = Kp + 1;
		}
		*/
		
		if (numtries >= 1660){
			stop_now = 1;
		}
		usleep(1000);
		v=getSample(LOADCELL_ADS1115_HANDLE);
		std::uint64_t timestamp = currentTimeMillis() - sampleStart;
		int gpio1status = gpioRead(GPIO1);
		int gpio2status = gpioRead(GPIO2);
		val = v;
		error = (target - val);
		numtries = numtries + 1;
		int PWM = clamp((fabs(error) * Kp),0,255);
		printf("numtries: %.4d timestamp: %lu target voltage: %.4f   load cell voltage: %.4f   last_val: %.4f   tol:%.4f   pwm:%.4d   error:%.4f   pos:   %.4f  stopnow: %.4d Kp: %.4d  GPIO1: %.1d   GPIO2: %.1d\n",numtries,timestamp,target,val,last_val,tol,PWM,error,v2*5,stop_now,Kp,gpio1status,gpio2status);
		if (error <= 0 ){ //this means overshoot
			float overshoot = val - target;
			printf("overshoot: %.4f\n",overshoot);
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
    abort_test(); 
    exit(0); 
}
		printf("Arrived at target: %.4f tol: %.4f   error: %.4f  \n",target,tol,error);
		return;
}
    abort_test(); 
    exit(0); 
}

float get_vector_average(const std::vector <float> &ptr){
	float sum = 0;
	for (long unsigned int i=0; i < ptr.size(); i++){
		sum = sum + ptr.at(i);
	}
	float average = sum/ptr.size();
	return average;
}

void frequency_test(int freq_hz, int numcycles){
    unsigned int half_us = (unsigned int) llround(500000.0 / freq_hz);
    //long long beginningtime = currentTimeMillis();
    //float v;
	//std::vector <float> datas;
    int completedcycles = 0;
	while(!stop_now){
		while (completedcycles < numcycles){
			gpioPWM(GPIO2, 0);
      //  usleep(1000);              // 1 ms deadtime

			gpioPWM(GPIO1, 200);
			usleep(half_us);           // high for half period
			
			//printf("2: %.4f\n",v);
			//datas.push_back(v);
			
			gpioPWM(GPIO1, 0);
		  //  usleep(1000);              // 1 ms deadtime

			gpioPWM(GPIO2, 0);
			usleep(half_us);           // high for half period
			//v = getSample();
			//printf("4: .%4f\n",v);

			completedcycles++;
		 //  printf("cycle: %.2d\n", completedcycles);
    
}
gpioWrite(GPIO1,0);
gpioWrite(GPIO2,0);
//float average = get_vector_average(datas);
//printf("Average load cell voltage over %.2d cycles at %.1d Hz: %.3f V.\n", completedcycles, freq_hz, average);

return;
}
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
    gpioSetPWMfrequency(GPIO1, 320);
    gpioSetPWMfrequency(GPIO2, 320);
  
    if ((serial = serialOpen("/dev/ttyAMA0", 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

      if (wiringPiSetup()!=0) {
    printf("cannot initialize WiringPi\n");
    return 1;
  }
  
  
  AdcBuf buf;
  LOADCELL_ADS1115_HANDLE = wiringPiI2CSetup(LOADCELL_ADS1115_ADDRESS);
  printf("%.4d",LOADCELL_ADS1115_HANDLE);
  sleep(0.5);
  LASER_ADS1115_HANDLE = wiringPiI2CSetup(LASER_ADS1115_ADDRESS);
  printf("%.4d",LASER_ADS1115_HANDLE);

  std::vector <float> loadcellAveraging;
  std::vector <float> laserAveraging;
  for (int i=0; i<100; i++){
	  float data=readVoltageSingleShot(LOADCELL_ADS1115_HANDLE, 1, 0);
	  loadcellAveraging.push_back(data);
  }
    for (int i=0; i<100; i++){
	  float data2=readVoltageSingleShot(LASER_ADS1115_HANDLE, 1, 0);
	  laserAveraging.push_back(data2);
  }
  
  float loadcellRefVoltage = get_vector_average(loadcellAveraging);
  float laserRefVoltage = get_vector_average(laserAveraging);
  sleep(0.5);
  std::thread measureThread(measure,loadcellRefVoltage,LOADCELL_ADS1115_HANDLE,laserRefVoltage,LASER_ADS1115_HANDLE,std::ref(buf));
  std::thread serialThread(send,std::ref(buf)); 
  printf("Initial voltage on Load Cell:  %.4f \n",loadcellRefVoltage);
  printf("accessing ads1115 chip on i2c address 0x%02x\n", LOADCELL_ADS1115_ADDRESS);
  
  printf("Initial voltage on Laser:  %.4f \n",laserRefVoltage);
  printf("accessing ads1115 chip on i2c address 0x%02x\n", LASER_ADS1115_ADDRESS);
  setADS1115ContinuousMode(LOADCELL_ADS1115_HANDLE, 1, 0, 7);
  setADS1115ContinuousMode(LASER_ADS1115_HANDLE, 1, 0, 7);
  sleep(0.5);
  sampleStart=currentTimeMillis();
  
	


	
  
  while(!stop_now){
	getSample(LOADCELL_ADS1115_HANDLE);
	getSample(LASER_ADS1115_HANDLE);
  
  
  //int Kp = 350;



	//for (int i=10; i<=15; i++)
	//frequency_test(30,10);
	//sleep(2);



}
 abort_test();
measureThread.join();
serialThread.join();
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




    
