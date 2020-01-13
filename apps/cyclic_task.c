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

/******* Functions for Real-Time task setup *********/
struct period_info {
  struct timespec next_period;
  long period_ns;
};

struct timespec curtime;

static void inc_period(struct period_info *pinfo)
{
  pinfo->next_period.tv_nsec += pinfo->period_ns;

  while (pinfo->next_period.tv_nsec >= 1000000000) {
    /* timespec nsec overflow */
    pinfo->next_period.tv_sec++;
    pinfo->next_period.tv_nsec -= 1000000000;
  }
}

static void periodic_task_init(struct period_info *pinfo)
{
  /* for simplicity, hardcoding a 1ms period */
  pinfo->period_ns = 10000000;

  clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

static void wait_rest_of_period(struct period_info *pinfo)
{
  inc_period(pinfo);

  /* for simplicity, ignoring possibilities of signal wakes */
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

/******* Functions for Real-Time task setup *********/


/******* Real-Time Thread *********/
void *thread_func(void *data)
{
  struct period_info pinfo;

  printf("Initializing Real-Time Task...\n");
  periodic_task_init(&pinfo);
  printf("Done.\n");
  

  printf("Initializing DAQ hardware...\n");
  s526_init();

  // Iniitalize counter channel 0 for pulse timing.
  s526_init_pulse_timer(0);
  int counter_val = 0; 
  double pulse_width = 0;
  
  printf("Done.\n");

  while (1) {

      // Read pulse count
      counter_val = s526_counter_read(0);
      // Get pulse timing
      pulse_width = counter_val/27e6;
      
      
      printf("Pulse Width: %.3e\n", pulse_width);
      clock_gettime(CLOCK_MONOTONIC, &curtime);
      //printf("Current time: %d s and %f ms.\n", curtime.tv_sec, curtime.tv_nsec/1000000.0);
    

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
