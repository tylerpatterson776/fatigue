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

#define GPIO1 1  /* BCM 0 : IN1 */
#define GPIO2 0   /* BCM 1 : IN2 */



static volatile sig_atomic_t stop_now = 0;
static void sigint_handler(int sig)
{
    (void)sig;        /* silence -Wunused-parameter */
    stop_now = 1;     /* async-signal-safe: just set a flag */
}

void init(){
		if (gpioInitialise() < 0) //pigpio init
	{printf("pigpio initialisation failed \n");}
	else
	{printf("pigpio initialisation OK\n");}
    setbuf(stdout,NULL);
    int ADC = i2cOpen(1,0x48,0); //handle for the I2C connection to the ADC
    if (ADC >= 0){ //i2c init
		printf("I2c open OK \n");
	}
	signal(SIGINT, sigint_handler); //signal init, this is responsible for abort_test
    gpioSetMode(GPIO1, PI_OUTPUT); gpioSetMode(GPIO2, PI_OUTPUT); 
    gpioSetPWMrange(GPIO1, 255);
    gpioSetPWMrange(GPIO2, 255);
    
    int test = i2cWriteWordData(ADC,0,1);
    printf("%.4f \n",test);
	
	int test2 = i2cReadByteData(ADC,0);
    printf("%.4f \n",test2);
	}

float ads1115_read_voltage(int chan)
{ float dummy = 1;
	return dummy;
	}


static inline float clamp(float v, float lo, float hi)
{ return (v < lo) ? lo : (v > hi ? hi : v); }


void print_voltage(void)
{
    float v_laser = ads1115_read_voltage(0);   /* AIN0 */
    float v_cell  = ads1115_read_voltage(1);   /* AIN1 */
    printf("laser voltage: %.4f V   load cell voltage: %.4f V\n",
           v_laser, v_cell);
}

int abort_test(void){
    printf("Test aborted\n");
    
    gpioPWM(GPIO1,0); 
    gpioPWM(GPIO2,0);
    gpioTerminate(); 
}

void motor_goto_voltage_pid(float target_v, int chan,
                            float kP, float kI, float kD,
                            float tol_v, unsigned timeout_s)
{
    /* pigpio PWM setup once */


    const float duty_max = 1;
    const float duty_min = 0.00;     /* Keep some torque */
    const float integral_limit = 2.0;/* VÂ·s antiâwindup  */

    float prev_err = 0.0, integral = 0.0;

    struct timespec t_prev, t0; clock_gettime(CLOCK_MONOTONIC, &t_prev); t0 = t_prev;


    while (!stop_now) 
    {
        /* --- timing ------------------------------------------------- */
        struct timespec t_now; clock_gettime(CLOCK_MONOTONIC, &t_now);
        float dt = (t_now.tv_sec - t_prev.tv_sec) + (t_now.tv_nsec - t_prev.tv_nsec)/1e9;
        t_prev = t_now;
        if (dt <= 0.0) dt = 1e-3;

        /* --- sensor ------------------------------------------------- */
        float v = ads1115_read_voltage(chan);
        //float v2 = ads1115_read_voltage(1);
        if (isnan(v)) continue;
        //printf("%.4f mm\n", v*4); 
      //  fflush(stdout);

        /* --- PID ---------------------------------------------------- */
        float err = target_v - v;
        if (v<-0.1) { stop_now = 1;}
        if (fabs(err) <= tol_v) { 
			printf("within tolerance");
			break; }

        integral += err * dt; integral = clamp(integral, -integral_limit, integral_limit);
        float derivative = (err - prev_err) / dt; prev_err = err;
        float output = kP*err + kI*integral + kD*derivative;

        float duty = clamp(fabs(output), duty_min, duty_max);
        float pwm = (duty * 255);
        printf("target v: %.4f  current v: %.4f  output: %.4f dist: %.4f mm   err:%.3f  duty: %.4f   pwm: %.3f \n",target_v,v,output,v*4,err,duty,pwm);

        if (output > 0) 
        { gpioPWM(GPIO1, pwm); gpioPWM(GPIO2, 0); }
        
        else            { gpioPWM(GPIO1, 0);   gpioPWM(GPIO2, pwm); }

        /* --- timeout ------------------------------------------------ */
        if (timeout_s) {
            float t = (t_now.tv_sec - t0.tv_sec) + (t_now.tv_nsec - t0.tv_nsec)/1e9;
            if (t > timeout_s) { fprintf(stderr,"PID timeout %.1f s\n",t); break; }
        }

        //gpioDelay(2000); /* 2 ms loop */
    }
    /* stop motor */ gpioPWM(GPIO1,0); gpioPWM(GPIO2,0);
}

void fullbeans(int dir){
	
	        if (dir > 0) { gpioPWM(GPIO1, 255); gpioPWM(GPIO2, 0); }
        
        else            { gpioPWM(GPIO1, 0);   gpioPWM(GPIO2, 255); }
	}

int main(void)
{
	init();
 
    /* Move to 1.00 V (â4 mm) using PID */
    float pos = 3;
    float pos2 = 4;
    float kP = 10;
    float kP2 = 10;
    float kI = 1.00;
    float kI2 = 1.00;
    float kD = 0;
    float kD2 = 0.00;
    float tol = 0.025 ;//tolerance for distance measurement
    float tol2 = 0.025;
    float timeout = 1; //timeout before motor stops moving
    
    
    while(!stop_now){
		print_voltage();
		//fullbeans(1);
		usleep(5000000);
		//oscillate(20,5000);
      
//motor_goto_voltage_pid(pos/4, /*chan*/0,
       //                /*kP*/kP, /*kI*/kI, /*kD*/kD,
         //            /*tol*/tol, /*timeout*/timeout);
    //usleep(50000);
   //print_voltage(1);
 //motor_goto_voltage_pid(pos2/4, /*chan*/0,
   //              /*kP*/kP2, /*kI*/kI2, /*kD*/kD2,
     //            /*tol*/tol2, /*timeout*/timeout);
                     
                         
    //usleep(5000000);
      //  print_voltage(0);

        }
  //  abort_test();
}
