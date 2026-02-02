#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <byteswap.h>
#include <unistd.h>
#include <ctype.h>
#include <stdlib.h>
#include <signal.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <ads1115rpi.h>

#include <vector>

using namespace std;


int ADS1115_ADDRESS=0x48;
int ADS1115_HANDLE=-1;

int   debug = 0;
int   channel = 1;
int   gain = 0;
float seconds = 1;
int   sps = 860;
int   ok2run = 1;

long long sample=-1;
long long sampleStart=0;
long long lastSampleTimestamp=0;

long long currentTimeMillis() {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    return (unsigned long long)(currentTime.tv_sec) * 1000 +
        (unsigned long long)(currentTime.tv_usec) / 1000;
}


void intHandler(int dummy) {
  fprintf(stderr,"\ninterrupt received; shutting down...\n");

  ok2run = 0;
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

  if (!ok2run) {
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


int main(int argc, char **argv) {


  if (wiringPiSetup()!=0) {
    fprintf(stderr,"cannot initialize WiringPi\n");
    return 1;
  }
  adsDebug(debug);


  fprintf(stderr,"accessing ads1115 chip on i2c address 0x%02x\n", ADS1115_ADDRESS);
  ADS1115_HANDLE = wiringPiI2CSetup(ADS1115_ADDRESS);

  signal(SIGINT, intHandler);

  

  setADS1115ContinuousMode(ADS1115_HANDLE, 0, 0, 7);


  sampleStart=currentTimeMillis();
  long long end = sampleStart + (seconds * 1000.0);
  fprintf(stderr,"now:  %lld\n",sampleStart);
  fprintf(stderr,"end:  %lld\n",end);

 // wiringPiISR(2,INT_EDGE_FALLING, getSample);
 
 
	for (int i=0; i< 100; i++){
		sleep(1/10);
		getSample();
		
	}
  
  while (ok2run && (seconds<0 || currentTimeMillis()<end)) {
    usleep(1000);
  }

  adsReset(ADS1115_HANDLE);
  fprintf(stderr,"exit: %lld; samples taken: %lld\n", currentTimeMillis(), sample);

  for (Sample *s: samples) {
      printf("%lld,%lld,%lld,%lld,%f\n", s->sample, s->timestamp, s->delta, s->offset, s->volts);
  }
}

