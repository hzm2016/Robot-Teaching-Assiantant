#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <ctype.h>
#include <math.h>
#include "model526.h"
#include "ftconfig.h"
#include "rtsetup.h"

#define TASK_PERIOD_SECONDS 0.01

static double loop_usage_array[(long)(1.0/TASK_PERIOD_SECONDS)];

/******* Real-Time Thread *********/
void *thread_func(void *data)
{
  struct period_info pinfo;

  printf("Initializing Real-Time Task...\n");
  const long task_period_ns = (long) (TASK_PERIOD_SECONDS*(1.0e9));
  printf("**********\nStep time is: %d ns = %f s\n**********\n", task_period_ns, TASK_PERIOD_SECONDS);
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
  
  printf("Done.\n");

  while (1) {
    // Get the time
    clock_gettime(CLOCK_MONOTONIC, &tic);
      /*** DO RT STUFF HERE ***/
    
      // Read pulse count
      counter_val = s526_counter_read(0);
      // Get pulse timing
      pulse_width = counter_val/27e6;



      /*** DO RT STUFF HERE ***/
      
      // Get the time again
      clock_gettime(CLOCK_MONOTONIC, &toc);
      int delta_t_s = toc.tv_sec-tic.tv_sec;
      double delta_t_ms = (toc.tv_nsec - tic.tv_nsec)/1000000.0;
      loop_usage_array[loop_ctr%loops_per_sec] = delta_t_ms;
      

      if (loop_ctr%loops_per_sec == 0) {
	
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
