#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <ctype.h>
#include <math.h>
extern "C" {
#include "model526.h"
#include "ftconfig.h"
#include "rtsetup.h"
}
#include "innfos_can_functions.hpp"
  
#define TASK_PERIOD_SECONDS 0.01

static double loop_usage_array[(long)(1.0/TASK_PERIOD_SECONDS)];



int can_init(struct threadArg &canArg1,struct threadArg &canArg2)
//this function initializes the the CAN sockets and frames
//Some dummy values were included for easy testing

{
  //initialse the can parameters
  int s,s2;
  struct sockaddr_can addrSource,addr1,addr2;
  struct can_frame frame,frame2;
  struct ifreq ifr;
  struct ifreq ifr2;

  const char *ifname = "can0";
  const char *ifname2 = "can1";


  long long InputData,InputData2,InputData3,InputData4;
  uint size, i=0;

  if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket");
    return -1;
  }
  //experimenting with 2 separate socket
  if((s2 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket");
    return -1;
  }

  strcpy(ifr.ifr_name, ifname);
  ioctl(s, SIOCGIFINDEX, &ifr);
  strcpy(ifr2.ifr_name, ifname2);
  ioctl(s2, SIOCGIFINDEX, &ifr2);

  addr1.can_family  = AF_CAN;
  addr1.can_ifindex = ifr.ifr_ifindex;
  //addr1.can_ifindex = 0;
  addr2.can_family  = AF_CAN;
  addr2.can_ifindex = ifr2.ifr_ifindex;
  //addr2.can_ifindex = 0;



  printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
  printf("%s at index %d\n", ifname2, ifr2.ifr_ifindex);

  if(bind(s, (struct sockaddr *)&addr1, sizeof(addr1)) < 0) {
    perror("Error in socket bind");
    return -2;
  }
  if(bind(s2, (struct sockaddr *)&addr2, sizeof(addr2)) < 0) {
    perror("Error in socket bind2");
    return -2;
  }

  frame.can_dlc = size;
  printf("bytes_arr is %d\n", size);
  frame2.can_dlc = size;

  canArg1.socket= s;
  canArg1.frame= frame;
  canArg2.socket= s2;
  canArg2.frame= frame2;
  return 0;
}


/******* Real-Time Thread *********/
void *thread_func(void *data)
{
    struct period_info pinfo;

    printf("Initializing Real-Time Task...\n");
    const long task_period_ns = (long) (TASK_PERIOD_SECONDS*(1.0e9));
    printf("**********\nStep time is: %ld ns = %f s\n**********\n", task_period_ns, TASK_PERIOD_SECONDS);
    periodic_task_init(&pinfo, task_period_ns);
    static struct timespec curtime;
    static struct timespec tic;
    static struct timespec toc;
    int loops_per_sec = 1.0/TASK_PERIOD_SECONDS;
    int loop_ctr = 0;
    printf("Loops per second: %d\n", loops_per_sec);
    printf("Done.\n");
    

    printf("Initializing DAQ hardware...\n");
    s526_init();

    // Iniitalize counter channel 0 for pulse timing.
    s526_init_pulse_timer(0);
    int counter_val = 0; 
    double pulse_width = 0;

    // Initialize the ADC Hardware
    int32_t ADC_CHANNELS[] = {0, 1, 2, 3, 4, 5};
    const int NUM_ADC_CHANNELS = 6;
    double adc_data[NUM_ADC_CHANNELS];
    s526_adc_init(ADC_CHANNELS, NUM_ADC_CHANNELS);

    // Initialize the force torque sensor library
    float FT[6];            // This array will hold the resultant force/torque vector.
    unsigned short index=1; // index of calibration in file (second parameter; default = 1)
	Calibration *cal;		// struct containing calibration information
    short sts;              // return value from functions
    float sample_bias[7]    = {0.2651,-0.0177,-0.0384,-0.0427,-0.1891,0.1373,-3.2423};
	float sample_reading[7] = {-3.2863,0.3875,-3.4877,0.4043,-3.9341,0.5474,-3.2106};

    // create Calibration struct
	cal=createCalibration("calibration/FT22694.cal", index);
	if (cal==NULL) {
		printf("\nSpecified calibration could not be loaded.\n");
		//scanf(".");
		return 0;
	}
	
	// Set force units.
	// This step is optional; by default, the units are inherited from the calibration file.
	sts=SetForceUnits(cal,"N");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid force units"); return 0;
		default: printf("Unknown error"); return 0;
	}


	// Set torque units.
	// This step is optional; by default, the units are inherited from the calibration file.
	sts=SetTorqueUnits(cal,"N-m");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid torque units"); return 0;
		default: printf("Unknown error"); return 0;
	}

    // Store an unloaded measurement
    //s526_adc_read(ADC_CHANNELS, NUM_ADC_CHANNELS, sample_bias);
    // Calibrate
    //Bias(cal, sample_bias);
    
    printf("Done.\n");

    while (1) {
        // Get the time
        clock_gettime(CLOCK_MONOTONIC, &tic);
        /*** DO RT STUFF HERE ***/
      
        // Read pulse count
        counter_val = s526_counter_read(0);
        // Get pulse timing
        pulse_width = counter_val/27e6;
        // Read ADC
        s526_adc_read(ADC_CHANNELS, NUM_ADC_CHANNELS, adc_data);
        // Convert into forces and torques
        for (int tmp_ctr = 0; tmp_ctr < NUM_ADC_CHANNELS; tmp_ctr++) {
            sample_reading[tmp_ctr] = (float) adc_data[tmp_ctr];
        }
        ConvertToFT(cal, sample_reading, FT);


        /*** DO RT STUFF HERE ***/
        
        // Get the time again
        clock_gettime(CLOCK_MONOTONIC, &toc);
        int delta_t_s = toc.tv_sec-tic.tv_sec;
        double delta_t_ms = (toc.tv_nsec - tic.tv_nsec)/1000000.0;
        loop_usage_array[loop_ctr%loops_per_sec] = delta_t_ms;
        

        if (loop_ctr%loops_per_sec == 0) {

            // Print ADC data
            printf("ADC Data :: [");
            for(int j=0; j<NUM_ADC_CHANNELS; j++)
            {
                printf(" %.2f ", FT[j]);
            }
            printf("]\n");

            printf("Pulse Width: %.3e\n", pulse_width);
            clock_gettime(CLOCK_MONOTONIC, &curtime);
            printf("Loop tasks took: %d s and %f ms.\n", delta_t_s, delta_t_ms);
        }
        
        loop_ctr += 1;

        wait_rest_of_period(&pinfo);
    }

    return NULL;
}
/******* Real-Time Thread *********/


int main(int argc, char* argv[])
{
    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread;
    int ret;

    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        printf("mlockall failed: %m\n");
        exit(-2);
    }

    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }

    /* Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
        printf("pthread setschedparam failed\n");
        goto out;
    }
    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
        printf("pthread setinheritsched failed\n");
        goto out;
    }

    /* Create a pthread with specified attributes */
    ret = pthread_create(&thread, &attr, thread_func, NULL);
    if (ret) {
        printf("create pthread failed\n");
        goto out;
    }

    /* Join the thread and wait until it is done */
    ret = pthread_join(thread, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

    out:
    return ret;
}
