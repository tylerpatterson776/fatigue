
#include <pigpio.h>
#include <nlohmann/json.hpp>
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
#include <termios.h>
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

using json = nlohmann::json;

#define GPIO1 27
#define GPIO2 17
#define GPIO3 22
#define GPIO4 23

const float lbsPerVolt = 10; //change this is load cell is recalibrated
const float newtonsPerVolt = 44.48; //change this is load cell is recalibrated
const float kgPerVolt = 4.535;
const float mmPerVolt = 1;

std::uint64_t currentTimeMillis();


const int LOADCELL_ADS1115_ADDRESS=0x48;
const int LASER_ADS1115_ADDRESS=0x4b;
int LOADCELL_ADS1115_HANDLE;
int LASER_ADS1115_HANDLE;

bool doLoadcellAveraging = 1;
bool doLaserAveraging = 0;
int doCycleWriting;
std::vector<float> currentLoadCellCycle;
std::vector<float> currentLaserCycle;
float vRef = 5.0;
int   gain = 0;
int serial;

  
const int laserStopDistance = 5;  //the test immediately stops if the laser reads this distance from where the test started (in mm)
const int minFrequency = 0;const int maxFrequency = 50;
long long sample=-1;
long long sampleStart=0;
long long lastSampleTimestamp=0;
float seconds = 1;
float loadcellRefVoltage;
float laserRefVoltage;


float voltsToPounds(float input, float reference, float conversionFactor = lbsPerVolt){ float result = (input - reference)*conversionFactor; return result;} //the *2 is to account for the fact we are voltage dividing 
float voltsToNewtons(float input, float reference, float conversionFactor = newtonsPerVolt){ float result = -((input - reference)*conversionFactor); return result;}
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
    gpioWrite(GPIO3,0);
    gpioWrite(GPIO4,0);
    gpioTerminate(); 
    exit(0);
}


static void sigint_handler(int sig)
{
    (void)sig;        /* silence -Wunused-parameter */
    stop_now = 1;     /* async-signal-safe: just set a flag */
}

void measure(float loadcellReference, int loadcellHandle, float laserReference, int laserHandle, AdcBuf &buf, int doCycleWriting, std::vector<float> &currentLoadCellCycle,std::vector <float> &currentLaserCycle, float &cycleCount)
{
	std::uint64_t lastPushedTime = 0;
	std::uint64_t sampleStart = currentTimeMillis(); //time when measuring began
	while(!stop_now){
			
		std::uint64_t time = currentTimeMillis(); //time when we are taking the measurement
		std::uint64_t time2 = (time - sampleStart); 
		double loadcellSample = voltsToNewtons(getSample(loadcellHandle),loadcellReference);
		double laserSample = voltsToMM(getSample(laserHandle),laserReference);
		loadcellSample = std::round(loadcellSample * 1000.0)/1000.0;
		laserSample = std::round(laserSample * 1000.0)/1000.0;
		
		if (time2 != lastPushedTime){ //samples are only pushed to buffer if timestamp has changed to avoid duplicates 
			json packet = {
			{"Data",{"Timestamp", time2},
					{"Cycle #", cycleCount},
					{"Load",loadcellSample},
					{"Distance" , laserSample},
			}
			};
			
			std::string s = packet.dump();
			buf.push({s});
		currentLoadCellCycle.push_back(loadcellSample);
		currentLaserCycle.push_back(laserSample);
		lastPushedTime = time2;	
	}
	
		if (abs(laserSample) >= laserStopDistance) {
			printf("Stopping test due to laser leaving bounds of %.4d, distance causing trip was %.4f mm ", laserStopDistance, laserSample); (stop_now=1);} //this detects if laser has gone out of range
	}
	return;
}
void send(AdcBuf &buf){
	printf("Send thread started\n");
	while (!stop_now || !buf.empty())
	{
	const auto sample = buf.pop(true);
	serialPuts(serial,sample.to_string().c_str());
	};	
	return;
	}
	
	
void receive(Serialbuf &receivebuffer, int &serialobj){
	int character;
	int endcharacter = 13;
	printf("Receive thread started\n");
	while (!stop_now) {
	character = serialGetchar(serialobj);
		while (character != endcharacter){
	//printf("%d\n",character);
	receivebuffer.push(character);
		};
	};
	return;
}



std::uint64_t currentTimeMillis() {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    return (std::uint64_t)(currentTime.tv_sec) * 1000 +
        (std::uint64_t)(currentTime.tv_usec) / 1000;
}

std::uint64_t currentTimeMicros() {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    return (std::uint64_t)(currentTime.tv_usec) * 100000 +
        (std::uint64_t)(currentTime.tv_usec) / 100000;
}

float clamp(float v, float lo, float hi){
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


int main(int argc, char *argv[])
{
	gpioCfgClock(1,1,0);
	std::string arg1(argv[1]);
	int frequency = std::stoi(arg1);
	std::string arg2(argv[2]);
	int dutyCycle = std::stoi(arg2);
	std::string arg3(argv[3]);
	float setpoint = std::stoi(arg3); //the amount of force (in newtons) we are trying to achieve with each cycle.
	if (gpioInitialise() < 0)
	{	printf("pigpio initialisation failed \n");}
	else
	{printf("pigpio initialisation OK\n");}
    setbuf(stdout,NULL);
	signal(SIGINT, sigint_handler);
    gpioSetMode(GPIO1, PI_OUTPUT); 
    gpioSetMode(GPIO2, PI_OUTPUT);
    gpioSetMode(GPIO3, PI_OUTPUT); 
    gpioSetMode(GPIO4, PI_OUTPUT);
    
    gpioWrite(GPIO3, 1); //enables one side of H bridge
    gpioWrite(GPIO4, 1); //enables other side of H bridge
  
  
  


      if (wiringPiSetup()!=0) {
    printf("cannot initialize WiringPi\n");
    return 1;
  }
	int pwmFrequency = 20000; //frequency at which PWM operates
	gpioSetPWMfrequency(GPIO1,pwmFrequency); //set the PWM frequency
	gpioSetPWMrange(GPIO1,100); //PWM will be out of 100, so 100 = full duty cycle, 50 = 50% duty cycle, etc.
  
  
      if ((serial = serialOpen("/dev/serial0", 921600)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }
  fflush(stdout);
  


  
  AdcBuf buf;
  Serialbuf receivebuffer;
  
  LOADCELL_ADS1115_HANDLE = getADS1115Handle(0x48);
  sleep(0.1);
  LASER_ADS1115_HANDLE = getADS1115Handle(0x4b);
  
  
  std::vector <float> loadcellAveraging;
  
  for (int i=0; i<100; i++){
	  float data=readVoltageSingleShot(LOADCELL_ADS1115_HANDLE, 1, 0);
	  printf("%.3f \n",data);
	  loadcellAveraging.push_back(data);
  }
   loadcellRefVoltage = get_vector_average(loadcellAveraging);
	//loadcellRefVoltage = 2.5; //loadcellRefVoltage should be around 2.5. Ideally it is exactly 2.5, but theres usually a small deviation.

  if (doLaserAveraging == 1) {
  std::vector <float> laserAveraging;
  
  for (int i=0; i<100; i++){
	  float data=readVoltageSingleShot(LASER_ADS1115_HANDLE, 1, 0);
	  printf("%.2f\n",data);
	  laserAveraging.push_back(data);
  }
   laserRefVoltage = get_vector_average(laserAveraging);
} 

  sleep(0.25);
  printf("Initial voltage on Load Cell:  %.4f \n",loadcellRefVoltage);
  printf("accessing ads1115 chip on i2c address 0x%02x\n", LOADCELL_ADS1115_ADDRESS);
  
  printf("Initial voltage on Laser:  %.4f \n",laserRefVoltage);
  printf("accessing ads1115 chip on i2c address 0x%02x\n", LASER_ADS1115_ADDRESS );
  setADS1115ContinuousMode(LOADCELL_ADS1115_HANDLE, 1, 0, 7);
  setADS1115ContinuousMode(LASER_ADS1115_HANDLE, 1, 0, 7);
  sleep(0.25);
  
 
	float cycleCount = 0;

	
	double Pgain = 0.1;
	double Dgain = 0;
	double Igain = 0.1 ;//gain for control variables
	
	double P = 0; double D = 0; double I = 0;
	double prev_error = 0;
	float feedback = 0;
	double time;
	double prevtime = 0;
	unsigned int half_us = (unsigned int) llround(500000.0/frequency);
	std::thread measureThread(measure,loadcellRefVoltage,LOADCELL_ADS1115_HANDLE,laserRefVoltage,LASER_ADS1115_HANDLE,std::ref(buf), doCycleWriting, std::ref(currentLoadCellCycle),std::ref(currentLaserCycle), std::ref(cycleCount));
	std::thread serialThread(send,std::ref(buf)); 
	std::thread receiveThread(receive,std::ref(receivebuffer),std::ref(serial));
	sampleStart=currentTimeMillis(); //sampleStart is the time at which the experiment starts
	time = sampleStart; //this time variable will change inside of the PID loop

	int minFrequency = 1; int maxFrequency = 50;
	float minLoad = 10; float maxLoad = 100;
	assert(("Invalid test frequency" && std::stoi(arg1) <= maxFrequency && std::stoi(arg1) >=minFrequency));
	assert(("Invalid force. Please choose between 0 and 100 newtons." && std::stoi(arg3) <= maxLoad && std::stoi(arg3) >=minLoad));


    
printf("Beginning test at %.2d Hz\n", frequency);
gpioSetPWMfrequency(GPIO1,pwmFrequency);
gpioSetPWMrange(GPIO1,1000);
 while(!stop_now){
	 

	if (frequency != 0){
			double newPWM = feedback;
			
			gpioPWM(GPIO1,newPWM); 
			gpioWrite(GPIO2, 0); 
			cycleCount += 0.5;
			
			usleep(half_us);   
			
			time = currentTimeMillis();
			double dt = currentTimeMillis() - prevtime;
			prevtime = currentTimeMillis();
			
			printf("Cycle # %.f   LOAD CELL: Median of cycle: %.2f N   Max of cycle:%.2f  Min of cycle:%.2f   PWM: %.f \n",cycleCount,get_vector_median(currentLoadCellCycle),get_vector_max(currentLoadCellCycle),get_vector_min(currentLoadCellCycle),newPWM); 
			printf("LASER: Median of cycle: %.2f    Max of cycle:%.2f  Min of cycle:%.2f",get_vector_median(currentLaserCycle),get_vector_max(currentLaserCycle),get_vector_min(currentLaserCycle)); 
			printf("time: %.2f\n", time-sampleStart);
			
			double currentValue = get_vector_max(currentLoadCellCycle); //this is the control variable, the median force over the current cycle
			double error = setpoint - currentValue;
			
			 P = error*Pgain;
			if (cycleCount > 3) {
				 D = (((error - prev_error)/dt)*Dgain);
				 I += (error*dt*0.01);
			}
			
			
			feedback =   clamp(P + D + I*Igain, setpoint, 550);
			double feedbacknoclamp = (P + D + I*Igain); //this is what the feedback value would be without clamping. just to see
			printf("Feedback : %.3f, Feedback without clamping: %.4f, P: %.2f,  D: %.2f,  Iout:%.2f\n", feedback,feedbacknoclamp, P,D,I*Igain);
			printf("new PWM: %.2f      error: %.2f  prev error: %.2f   setpoint: %.2f     current Value: %.2f\n",newPWM,error,prev_error,setpoint,currentValue);
			prev_error = error;
			
			
			
			
			
			
			
			
			
			
			gpioWrite(GPIO1,0);
			gpioWrite(GPIO2, 0);
			cycleCount += 0.5;
			usleep(half_us);        
			currentLoadCellCycle.clear(); 
			currentLaserCycle.clear();
    
}

}


abort_test();
measureThread.join();
serialThread.join();
receiveThread.join();

 return 0; 
}

  
