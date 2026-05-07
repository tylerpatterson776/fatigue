#include <pigpio.h>
#include <nlohmann/json.hpp>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <stdint.h>
#include <iostream>
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
double loadcellSample = 0;
double laserSample = 0;

static volatile sig_atomic_t stop_now = 0;
int LOADCELL_ADS1115_HANDLE = getADS1115Handle(0x48);
int LASER_ADS1115_HANDLE = getADS1115Handle(0x4b);


std::uint64_t currentTimeMillis() {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    return (std::uint64_t)(currentTime.tv_sec) * 1000 +
        (std::uint64_t)(currentTime.tv_usec) / 1000;
}


double getSample(int HANDLE) {
	double volts         = readVoltage(HANDLE);
	return volts;
}

void measure(int loadcellHandle, int laserHandle, AdcBuf &buffer, uint64_t startTime){
	while (!stop_now){
	loadcellSample = readVoltageSingleShot(loadcellHandle,0,0); //std::round(getSample(loadcellHandle) * 1000.0)/1000.0;
	sleep(1);
	laserSample = readVoltageSingleShot(laserHandle,1,0); //std::round(getSample(laserHandle) * 1000.0)/1000.0;
			json packet = {
			{"Data",{"Timestamp", currentTimeMillis() - startTime},
					{"Cycle #", 1},
					{"Load",loadcellSample},
					{"Distance" , laserSample},
			}
			};
			
			std::string s = packet.dump();
			std::cout << packet.dump() << "\n";
			buffer.push({s});
}
}

int main(){
	std::uint64_t starttime = currentTimeMillis();
	AdcBuf buffer;
	int a;
	int b;
	if ((a = wiringPiI2CSetup(0x48)) < 0) {
		printf("cannot initialize WiringPi\n");
		return 1;
	}
	sleep(1);
		if ((b = wiringPiI2CSetup(0x4b)) < 0) {
		printf("cannot initia\n");
		return 1;
	}
	printf("Loadcell Handle: %d \n",LOADCELL_ADS1115_HANDLE);
	printf("Laser Handle: %d \n",LASER_ADS1115_HANDLE);
	setADS1115ContinuousMode(LOADCELL_ADS1115_HANDLE, 1, 0, 7); //loadcell, ch 0, 0 gain (+6.144v), 860 sps
	sleep(1);
	setADS1115ContinuousMode(LASER_ADS1115_HANDLE, 1, 0, 7);//loadcell, ch 1, 0 gain (+6.144v), 860 sps
	sleep(0.1);
	int count = 0;
	//std::thread measureThread(measure,LOADCELL_ADS1115_HANDLE,LASER_ADS1115_HANDLE, std::ref(buffer), starttime);
	while(!stop_now){
		count += 1;
		loadcellSample = std::round(getSample(a) * 1000.0)/1000.0;
		laserSample = std::round(getSample(b) * 1000.0)/1000.0;
			json packet = {
			{"Data",{"Timestamp", currentTimeMillis() - starttime},
					{"Cycle #", 1},
					{"Load",loadcellSample},
					{"Distance" , laserSample},
			}
			};
			
			std::string s = packet.dump();
			if (count % 100 == 1){
				std::cout << packet.dump() << "\n";
		}
			buffer.push({s});
		
		}
//measureThread.join();
return 0;
}
