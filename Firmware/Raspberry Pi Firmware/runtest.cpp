#include <pigpio.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <cmath>
#include <map>
#include <deque>
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
#include <algorithm>
#include <ctype.h>
#include <thread>
#include <cstdint>
#include <array>
#include <atomic>
#include <optional>
#include <sstream>
#include <thread>
#include "ring.h"
#include <cassert>

#define GPIO1 18
#define GPIO2 27
#define GPIO3 17

const float lbsPerVolt = 10; //change this is load cell is recalibrated
const float newtonsPerVolt = 44.48; //change this is load cell is recalibrated
const float kgPerVolt = 4.535;
const float mmPerVolt = 4;

std::uint64_t currentTimeMillis();


const int LOADCELL_ADS1115_ADDRESS=0x48;
const int LASER_ADS1115_ADDRESS=0x49;
int LOADCELL_ADS1115_HANDLE;
int LASER_ADS1115_HANDLE;

bool doLoadcellAveraging = 1;
bool doLaserAveraging = 1;
int doCycleWriting;
std::vector<float> currentLoadCellCycle;
std::vector<float> currentLaserCycle;
float vRef = 5.0;
int   gain = 0;
int serial;
  
const int laserStopDistance = 5; 
const int minFrequency = 0;const int maxFrequency = 50;
long long sample=-1;
long long sampleStart=0;
long long lastSampleTimestamp=0;
float seconds = 1;
float loadcellRefVoltage;
float laserRefVoltage;


float voltsToPounds(float input, float reference, float conversionFactor = lbsPerVolt){ float result = (input - reference)*conversionFactor; return result;} //the *2 is to account for the fact we are voltage dividing 
float voltsToNewtons(float input, float reference, float conversionFactor = newtonsPerVolt){ float result = (input - reference)*conversionFactor; return result;}
float voltsToKg(float input, float reference, float conversionFactor = kgPerVolt){ float result = (input - reference)*conversionFactor; return result;}
float voltsToMM(float input, float reference, float conversionFactor = mmPerVolt){ float result = (input - reference)*conversionFactor; return result;}
float getSample(int HANDLE);

static volatile sig_atomic_t stop_now = 0;


float get_vector_average(const std::vector <float> &ptr){
	float sum = 0;
	for (long unsigned int i=0; i < ptr.size(); i++){
		sum = sum + ptr.at(i);
	}
	float average = sum/ptr.size();
	return average;
}


float get_vector_median(std::vector <float> &ptr){
	size_t n = ptr.size()/2;
	std::nth_element(ptr.begin(),ptr.begin()+n,ptr.end());
	return ptr[n];
	
}

float get_vector_max(std::vector <float> &ptr){
	float max = *std::max_element(ptr.begin(), ptr.end());
	return max;
}

float get_vector_min(std::vector <float> &ptr){
	float max = *std::min_element(ptr.begin(), ptr.end());
	return max;
}



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

void measure(float loadcellReference, int loadcellHandle, float laserReference, int laserHandle, AdcBuf &buf, int doCycleWriting, std::vector<float> &currentLoadCellCycle,std::vector <float> &currentLaserCycle)
{
	std::uint64_t lastPushedTime = 0;
	std::uint64_t sampleStart = currentTimeMillis(); //time when measuring began
	while(!stop_now){
		std::uint64_t time = currentTimeMillis(); //time when we are taking the measurement
		std::uint64_t time2 = (time - sampleStart); 
		float Sample = (getSample(loadcellHandle));
		float loadcellSample = voltsToNewtons(Sample,loadcellReference);
		float laserSample = voltsToMM(getSample(laserHandle),laserReference);
		if (time2 != lastPushedTime){ //samples are only pushed to buffer if timestamp has changed to avoid duplicates 
		buf.push({ADC::ADC1,loadcellSample,time2});
		//if (doCycleWriting == 1){currentCycle.push_back(loadcellSample);}
		currentLoadCellCycle.push_back(loadcellSample);
		currentLaserCycle.push_back(laserSample);
		//printf("loadcell Voltage : %.3f   loadcell Newtons : %.3f   laser: %.3f  mm    timestamp:%.2ld   ms\n",Sample,loadcellSample, laserSample, time2);
		//buf2.push(Sample{ADC::ADC2,laserSample,time2});
		//buf2.push(Sample{ADC::ADC1,loadcellSample,time2});
		lastPushedTime = time2;	
	}
	
		if (abs(laserSample) >= laserStopDistance) {
			printf("Stopping test due to laser leaving bounds of %.4d, distance causing trip was %.4f mm ", laserStopDistance, laserSample); (stop_now=1);} //this detects if laser has gone out of range
	}
	return;
}
void send(AdcBuf &buf){
	while (!stop_now || !buf.empty())
	{
	const auto sample = buf.pop(true);
	//serialPuts(serial,sample.to_string().c_str());
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


void frequency_test(int freq_hz){
	
	if (freq_hz == 0 ){return;}
    unsigned int half_us = (unsigned int) llround(500000.0 / freq_hz);
    
			gpioPWM(GPIO2, 0);

			gpioPWM(GPIO1,255);
			usleep(half_us);           // high for half period

			gpioPWM(GPIO1, 0);
			
			gpioPWM(GPIO2, 0);
			usleep(half_us);           // high for half period
    

return;
}



int main(int argc, char *argv[])
{
	int pwmFreq = 8000;
	std::string arg1(argv[1]);
	std::string arg2(argv[2]);
	std::string arg3(argv[3]);
	if (gpioInitialise() < 0)
	{	printf("pigpio initialisation failed \n");}
	else
	{printf("pigpio initialisation OK\n");}
    setbuf(stdout,NULL);
	signal(SIGINT, sigint_handler);
    gpioSetMode(GPIO1, PI_OUTPUT); 
    gpioSetMode(GPIO2, PI_OUTPUT);
    gpioSetPWMfrequency(GPIO1, pwmFreq);
    gpioSetPWMfrequency(GPIO2, pwmFreq);
  


      if (wiringPiSetup()!=0) {
    printf("cannot initialize WiringPi\n");
    return 1;
  }
  
      if ((serial = serialOpen("/dev/serial0", 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }
  fflush(stdout);

  
  AdcBuf buf;
  LOADCELL_ADS1115_HANDLE = wiringPiI2CSetup(LOADCELL_ADS1115_ADDRESS);
  sleep(0.1);
  LASER_ADS1115_HANDLE = wiringPiI2CSetup(LASER_ADS1115_ADDRESS);
  
  
  if (doLoadcellAveraging == 1) {
  std::vector <float> loadcellAveraging;
  
  for (int i=0; i<500; i++){
	  float data=readVoltageSingleShot(LOADCELL_ADS1115_HANDLE, 3, 0);
	  loadcellAveraging.push_back(data);
  }
   loadcellRefVoltage = get_vector_average(loadcellAveraging);
} else { loadcellRefVoltage = 0;}

  if (doLaserAveraging == 1) {
  std::vector <float> laserAveraging;
  
  for (int i=0; i<100; i++){
	  float data=readVoltageSingleShot(LASER_ADS1115_HANDLE, 0, 0);
	  //printf("%.2f\n",data);
	  laserAveraging.push_back(data);
  }
   laserRefVoltage = get_vector_average(laserAveraging);
} 

  sleep(0.25);
  printf("Initial voltage on Load Cell:  %.4f \n",loadcellRefVoltage);
  printf("accessing ads1115 chip on i2c address 0x%02x\n", LOADCELL_ADS1115_ADDRESS);
  
  printf("Initial voltage on Laser:  %.4f \n",laserRefVoltage);
  printf("accessing ads1115 chip on i2c address 0x%02x\n", LASER_ADS1115_ADDRESS );
  setADS1115ContinuousMode(LOADCELL_ADS1115_HANDLE, 3, 0, 7);
  setADS1115ContinuousMode(LASER_ADS1115_HANDLE, 0, 0, 7);
  sleep(0.25);
  std::thread measureThread(measure,loadcellRefVoltage,LOADCELL_ADS1115_HANDLE,laserRefVoltage,LASER_ADS1115_HANDLE,std::ref(buf), doCycleWriting, std::ref(currentLoadCellCycle),std::ref(currentLaserCycle));
  std::thread serialThread(send,std::ref(buf)); 

  sampleStart=currentTimeMillis();
  assert(("Invalid test frequency" && std::stoi(arg1) <= maxFrequency && std::stoi(arg1) >=minFrequency));



int frequency = std::stoi(arg1);
int PWM = std::stoi(arg2);
int desiredForce = std::stoi(arg3);//in newtons
int completedCycles = 0;
printf("Beginning test at %.2d Hz and %.3d PWM \n", frequency, PWM);
 while(!stop_now){
	if (frequency != 0){/*
		if (completedCycles == 3 && PWM >=480000){
			completedCycles = 0;
			PWM = PWM - 5000;
			sleep(0.1);
		}
		* //*/
			
    unsigned int half_us = (unsigned int) llround(500000.0 / frequency);
			gpioHardwarePWM(GPIO1,40000 ,PWM);
			gpioWrite(GPIO2, 1);
			gpioWrite(GPIO3,0);
			
			//run a thing here that collects until cycle is over
			doCycleWriting = 1;
			currentLoadCellCycle.clear();
			currentLaserCycle.clear();
			usleep(half_us);   
			//printf("%.2ld", currentCycle.size());
			//doCycleWriting = 0;
			/*for (  float i=0; i<currentCycle.size(); i++){
				printf("%.2f \n", currentCycle[i]);
			}*/
			completedCycles ++;
			printf("Cycle # %.d   LOAD CELL: Median of cycle: %.2f N   Max of cycle:%.2f  Min of cycle:%.2f   PWM: %.d           ",completedCycles,get_vector_median(currentLoadCellCycle),get_vector_max(currentLoadCellCycle),get_vector_min(currentLoadCellCycle),PWM); 
			printf("LASER: Median of cycle: %.2f    Max of cycle:%.2f  Min of cycle:%.2f   PWM: %.d\r",get_vector_median(currentLaserCycle),get_vector_max(currentLaserCycle),get_vector_min(currentLaserCycle),PWM); 
			 //calculate median of the previous cycle, that is the value. then input that value into control loop
		//	printf("Braking begins now\n");
			gpioWrite(GPIO1,0);
			gpioWrite(GPIO2, 0);
			gpioWrite(GPIO3,0);
			usleep(half_us);        
			currentLoadCellCycle.clear(); 
			currentLaserCycle.clear();
			  // low for half period
			//printf("Pushing begins now\n");
			
    
}

	/*
	if (serialDataAvail (serial)){
	printf(" %3d", serialGetchar (serial));
	}
	*/
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


    
