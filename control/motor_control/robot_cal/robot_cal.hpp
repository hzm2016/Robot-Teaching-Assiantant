#include<linux/can.h>
#include<sys/socket.h>
#include <unistd.h>
#include<sys/types.h>
#include<sys/ioctl.h>
#include<pthread.h>
#include<stdio.h>
#include<stdint.h>
#include<iostream>
#include <math.h>
#include <net/if.h>  
#include <errno.h> 
#include<cstring> 

#define PI 3.1415926 
const double ctl_ratio_1 = -2000.0/32;     
const double ctl_ratio_2 = 2000.0/32;  
const double d_t = 0.001; 
const double L_1 = 0.30, L_2 = 0.25;  

double clip(double angle, double lower_bound, double upper_bound); 

void Jacobian(double theta_1_t, double theta_2_t); 

void split(const string& s, vector<string>& tokens, const string& delim); 

int vic_optimization(
double stiffness, double damping,  
double q_1_target, double q_2_target,  
double q_1_initial, double q_2_initial,  
double theta_1_initial, double theta_2_initial,  
double dist_threshold  
); 

double read_initial_angle_1(); 

double read_initial_angle_2(); 

double read_initial_angle_3(); 

double read_angle_1(double theta_1_initial); 

double read_angle_2(double theta_2_initial, double theta_1_t); 

double read_angle_3(double theta_3_initial); 

void load_path_data(double *theta_1_list, double *theta_2_list); 

int motor_two_link_stop(); 

int motor_3_stop(); 


double set_two_link_position(double theta_1_initial, double theta_2_initial, 
int32_t angle_1, int32_t angle_2); 