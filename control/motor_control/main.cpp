#include <iostream> 
#include <fstream> 
#include <string>
#include <vector>
#include <stdlib.h> 
#include <pybind11/pybind11.h> 
#include "gyems_can_functions.h" 
#include "renishaw_can_functions.hpp" 
using namespace std; 

#include <unistd.h> 
#include <signal.h>  
#include <cmath> 
#include<stdio.h>

#include <pybind11/numpy.h>
namespace py = pybind11;

#define PI 3.1415926 
const double ctl_ratio_1 = -2000.0/32;    
const double ctl_ratio_2 = 2000.0/32;  
const double d_t = 0.001;   

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)


int run_on;

void 
sigint_1_step(int dummy) {
    if (run_on == 1) 
		run_on = 0; 
}


void split(const string& s, vector<string>& tokens, const string& delim=",") 
{
    tokens.clear(); 
    size_t lastPos = s.find_first_not_of(delim, 0); 
    size_t pos = s.find(delim, lastPos); 
    while (lastPos != string::npos) {
        tokens.emplace_back(s.substr(lastPos, pos - lastPos)); 
        lastPos = s.find_first_not_of(delim, pos); 
        pos = s.find(delim, lastPos); 
    }  
}  

double clip(double angle, double lower_bound, double upper_bound)
{
    double clip_angle;

    if (angle < lower_bound)
    {
        clip_angle = lower_bound; 
    } 
    else if(angle > upper_bound)
    {
        clip_angle = upper_bound; 
    }
    else
    {
        clip_angle = angle; 
    }
    return clip_angle; 
}

int read_initial_encode() 
{ 
    ////////////////////////////////////////////
    // Read original encoder
    ////////////////////////////////////////////
    controller_renishaw encoder("can2");  

    float encoder_arr[2];  

	encoder.read_ang_encoder(encoder_arr);   
  	double q_1 = (double) encoder_arr[1]*PI/180.0;  
  	double q_2 = (double) encoder_arr[0]*PI/180.0;  
    
    printf("Encoder 1 position: %f\n", q_1);  
    printf("Encoder 2 position: %f\n", q_2);  

    return 1;  
} 

double read_initial_angle_1()
{
    ////////////////////////////////////////////
    // Read motor original angle 1
    ////////////////////////////////////////////

    CANDevice can1((char *) "can1");   
    can1.begin();   

    Gcan motor_1(can1);   
    motor_1.begin();   
    
    double theta_1 = motor_1.read_sensor(2);   
    printf("Motor 1 original position: %f\n", theta_1);   

    return theta_1;   
}

double read_initial_angle_2()   
{
    ////////////////////////////////////////////
    // Read motor original angle 2
    ////////////////////////////////////////////

    CANDevice can0((char *) "can0");   
    can0.begin();   

    Gcan motor_2(can0);   
    motor_2.begin();   

    // double theta_2 = motor_2.read_sensor(1);   
    double theta_2 = motor_2.read_sensor(1);  
    printf("Motor 2 original position: %f\n", theta_2);   

    return theta_2;   
}

double read_initial_angle_3()   
{
    ////////////////////////////////////////////
    // Read motor original angle 3
    ////////////////////////////////////////////
    CANDevice can3((char *) "can3");   
    can3.begin();   

    Gcan motor_3(can3);   
    motor_3.begin();   

    double theta_3 = motor_3.read_single_turn(1);  
    printf("Motor 3 original position: %f\n", theta_3);   

    return theta_3;   
}

double read_angle_1(double theta_1_initial) 
{
    //////////////////////////////////////////// 
    // Read motor angle 1
    //////////////////////////////////////////// 

    CANDevice can1((char *) "can1");  
    can1.begin();  

    Gcan motor_1(can1);   
    motor_1.begin();  
    
    double theta_1 = motor_1.read_sensor(2) - theta_1_initial;    
    // printf("Motor 1 position: %f\n", theta_1);   

    return theta_1;   
}

double read_angle_2(double theta_2_initial, double theta_1_t)   
{
    ////////////////////////////////////////////
    // Read motor angle 2
    ////////////////////////////////////////////

    CANDevice can0((char *) "can0");   
    can0.begin();   

    Gcan motor_2(can0);   
    motor_2.begin();   

    double theta_2 = -1 * (motor_2.read_sensor(1) + theta_1_t - theta_2_initial);   
    // printf("Motor 2 position: %f\n", theta_2);   

    return theta_2;   
}

double read_angle_3(double theta_3_initial)    
{
    ////////////////////////////////////////////
    // Read motor angle 3 
    ////////////////////////////////////////////
    CANDevice can3((char *) "can3");   
    can3.begin();   

    Gcan motor_3(can3);    
    motor_3.begin();   
    // printf("can 3 begin !!!\n"); 

    // double theta_2 = -1 * (motor_2.read_sensor(1) + theta_1_t - theta_2_initial);   
    // printf("Motor 2 position: %f\n", theta_2);   
    // double theta_3 = motor_3.read_sensor(1);  
    // printf("Motor 3 position: %f\n", theta_3);   

    double theta_3_t; 

    theta_3_t = motor_3.read_single_turn(1) - theta_3_initial;   
    printf("Motor 3 rad: %f, deg %f\n", theta_3_t, theta_3_t * 180/3.14);  

    // int32_t angle = theta_3_t + 10; 
    // uint16_t max_speed = 20; 
    // motor_3.pack_position_2_cmd(1, angle, max_speed); 

    // motor_3.read_encoder(1); 
    // motor_3.readcan(); 
    // motor_3.set_torque(1, 0, &d_theta_1_t, &torque_1_t); 
    // motor_3.pack_stop_cmd(1); 

    return theta_3_t;   
}

double set_position(double theta_3_initial, int32_t angle)   
{
    ////////////////////////////////////////////
    // Read motor angle 3 
    ////////////////////////////////////////////
    CANDevice can0((char *) "can3");   
    can0.begin();   

    Gcan motor_3(can0);   
    motor_3.begin();  
 
    double theta_3_t;   

    theta_3_t = motor_3.read_single_turn(1);  
    printf("Motor 3 position: %f\n", theta_3_t/3.14*180);   

    // int32_t angle = theta_3_t + 10; 
    uint16_t max_speed = 20; 
    int32_t angle_fixed = 200; 

    motor_3.pack_position_2_cmd(1, angle, max_speed); 
    // motor_3.pack_position_1_cmd(1, angle_fixed); 

    // motor_3.read_encoder(1); 
    // motor_3.readcan(); 

    run_on = 1;  
    // Catch a Ctrl-C event:
	void  (*sig_h)(int) = sigint_1_step;   // pointer to signal handler

    // while(run_on) 
    // {
    //     // Catch a Ctrl-C event: 
    //     signal(SIGINT, sig_h);  

    //     motor_3.set_torque(1, 50, &d_theta_1_t, &torque_1_t); 
    // }
    
    // motor_3.pack_stop_cmd(1); 

    return theta_3_t;   
}


int motor_3_stop()
{
    ////////////////////////////////////////////
    // Read motor angle 3 
    ////////////////////////////////////////////
    CANDevice can0((char *) "can3");   
    can0.begin();   

    Gcan motor_3(can0);   
    motor_3.begin();  
 
    double theta_3_t;   

    theta_3_t = motor_3.read_single_turn(1);  
    printf("Motor 3 position: %f\n", theta_3_t);   

    motor_3.pack_stop_cmd(1);  
    
    return 1; 
}

double read_link_angle_1(double q_1_initial)   
{
    ////////////////////////////////////////////  
    // Read link angle 1
    ////////////////////////////////////////////  

    controller_renishaw encoder("can2");  

    float encoder_arr[2];  

	encoder.read_ang_encoder(encoder_arr);  

  	double q_1 = (double) encoder_arr[1]*PI/180.0 + q_1_initial;  

    return q_1;  
} 


double read_link_angle_2(double q_2_initial)   
{
    ////////////////////////////////////////////   
    // Read link angle 2    
    ////////////////////////////////////////////   

    controller_renishaw encoder("can2");  

    float encoder_arr[2];  

	encoder.read_ang_encoder(encoder_arr);  

  	double q_2 = (double) encoder_arr[0]*PI/180.0 + q_2_initial;  

    return q_2;  
}  


int move_to_target(double stiffness, double damping,  
double q_1_target[], double q_2_target[], int N,   
double theta_1_initial, double theta_2_initial,  
double dist_threshold  
)
{
    ////////////////////////////////////////////////////////
    //// Initial Encoder and Motor CAN
    //////////////////////////////////////////////////////// 

    CANDevice can0((char *) "can0");    
    can0.begin();   
    CANDevice can1((char *) "can1");     
    can1.begin();   

    Gcan motor_1(can1);   
    Gcan motor_2(can0);   
    motor_1.begin();   
    motor_2.begin();   

    printf("Move to target start !!!!\n");   

    ////////////////////////////////////////////////////////
    // One loop control demonstration
    ////////////////////////////////////////////////////////

    string output_angle = "move_target_angle_list.txt";    
    ofstream OutFileAngle(output_angle);    
    OutFileAngle << "angle_1" << "," << "angle_2" << "\n";    

    string output_torque = "move_target_torque_list.txt";    
    ofstream OutFileTorque(output_torque);    
    OutFileTorque << "torque_1" << "," << "torque_2" << "\n";    

    double torque_lower_bound = -1.5;    
    double torque_upper_bound = 1.5;    
    
    // double ctl_ratio_1 = -2000.0/32;   
    // double ctl_ratio_2 = 2000.0/32;   

    double theta_1_t = 0.0;   
    double theta_2_t = 0.0;   

    double d_theta_1_t = 0.0;    
    double d_theta_2_t = 0.0;    

    double theta_1_e = 0.0;   
    double theta_2_e = 0.0;   

    double d_theta_1_e = 0.0;   
    double d_theta_2_e = 0.0;   

    double torque_1 = 0.0;   
    double torque_2 = 0.0;   

    double torque_1_t = 0.0;   
    double torque_2_t = 0.0;   

    double pos_1 = 0.0;      
    double pos_2 = 0.0;      

    double dist = 0.0; 
    int initial_index = 0;    
    int max_index = 10000;   

    /////////////////////////////////////////////////////
    /////  avoid large motion at starting points  ///////
    /////////////////////////////////////////////////////
    for(int index=0; index<5; index=index+1) 
    {
        pos_1 = motor_1.set_torque(2, 0.0, &d_theta_1_t, &torque_1_t); 
        pos_2 = motor_2.set_torque(1, 0.0, &d_theta_2_t, &torque_2_t); 
    }

    run_on = 1; 

    // Catch a Ctrl-C event:
	void  (*sig_h)(int) = sigint_1_step;   // pointer to signal handler

    // Catch a Ctrl-C event: 
    signal(SIGINT, sig_h);  
 
    // dist > dist_threshold && initial_index < max_index
    while(run_on)  
    {
        theta_1_t = motor_1.read_sensor(2) - theta_1_initial;  
        theta_2_t = -1 * (motor_2.read_sensor(1) + theta_1_t - theta_2_initial);   

        dist = sqrt(pow((theta_1_t - q_1_target[0]), 2) + pow((theta_2_t - q_2_target[0]), 2));   

        printf(" theta_1_t: %f\n", theta_1_t);    
        printf(" theta_2_t: %f\n", theta_2_t);    

        /////////////////////////////////////////////////////
        // calculate torque control command 
        ///////////////////////////////////////////////////// 
        torque_1 = clip(-1 * stiffness * (q_1_target[0] - theta_1_t) - damping * (d_theta_1_e - d_theta_1_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_1; 
        torque_2 = clip(-1 * stiffness * (q_2_target[0] - theta_2_t) - damping * (d_theta_2_e - d_theta_2_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_2; 

        // double torque_1_o = - K_p_1 * (theta_1_e - theta_1_t) - K_d_1 * (d_theta_1_e - d_theta_1_t);  
        // double torque_2_o = - K_p_2 * (theta_2_e - theta_2_t) - K_d_2 * (d_theta_2_e - d_theta_2_t);  

        OutFileAngle << theta_1_t << "," << theta_2_t << "\n";   

        // pos_1 = motor_1.set_torque(2, torque_1, &d_theta_1_t, &torque_1_t);    
        // pos_2 = motor_2.set_torque(1, torque_2, &d_theta_2_t, &torque_2_t);    

        pos_1 = motor_1.set_torque(2, 0.0, &d_theta_1_t, &torque_1_t);   
        pos_2 = motor_2.set_torque(1, torque_2, &d_theta_2_t, &torque_2_t);   

        OutFileTorque << torque_1_t << "," << torque_2_t << "\n";   

        // OutFileVel << d_theta_1_t << " " << d_theta_2_t << "\n";   

        // printf("d_theta_1_t: %f\n", d_theta_1_t);   
        // printf("d_theta_2_t: %f\n", d_theta_2_t);   
    }

    printf("Move to target done !!!! \n"); 

    OutFileAngle.close();   
    OutFileTorque.close();       

    motor_1.pack_stop_cmd(2);   
    motor_2.pack_stop_cmd(1);   

    return 1;  
}


int vic_optimization(
double stiffness, double damping,  
double q_1_target, double q_2_target,  
double q_1_initial, double q_2_initial,  
double theta_1_initial, double theta_2_initial,  
double dist_threshold  
)
{
    ////////////////////////////////////////////////////////
    //// Initial Encoder and SEA Motor CAN
    //////////////////////////////////////////////////////// 

    CANDevice can0((char *) "can0");    
    can0.begin();   
    CANDevice can1((char *) "can1");   
    can1.begin();   

    Gcan motor_1(can1);   
    Gcan motor_2(can0);   
    motor_1.begin();   
    motor_2.begin();   

    printf("Move to target start !!!!\n");  

    ///////////////////// encoder reading //////////////////
    controller_renishaw encoder("can2");  
    float encoder_arr[2];  

    ////////////////////////////////////////////////////////
    // One loop control demonstration   
    ////////////////////////////////////////////////////////

    string output_angle = "move_target_angle_list.txt";    
    ofstream OutFileAngle(output_angle);    
    OutFileAngle << "angle_1" << "," << "angle_2" << "\n";    

    string output_torque = "move_target_torque_list.txt";    
    ofstream OutFileTorque(output_torque);    
    OutFileTorque << "torque_1" << "," << "torque_2" << "\n";    

    double torque_lower_bound = -1.5;    
    double torque_upper_bound = 1.5;   
    
    double ctl_ratio_1 = -2000.0/32;   
    double ctl_ratio_2 = 2000.0/32;   

    double theta_1_t = 0.0;   
    double theta_2_t = 0.0;   

    double d_theta_1_t = 0.0;    
    double d_theta_2_t = 0.0;    

    double theta_1_e = 0.0;   
    double theta_2_e = 0.0;   

    double d_theta_1_e = 0.0;   
    double d_theta_2_e = 0.0;   

    double torque_1 = 0.0;   
    double torque_2 = 0.0;   

    double torque_1_t = 0.0;  
    double torque_2_t = 0.0;  

    double pos_1 = 0.0;  
    double pos_2 = 0.0;      

    double q_1_t = 0.0; 
    double q_2_t = 0.0; 

    double dist = 0.0; 
    int initial_index = 0; 
    int max_index = 10000; 

    /////////////////////////////////////////////////////
    /////  avoid large motion at starting points  ///////
    /////////////////////////////////////////////////////
    for(int index=0; index<5; index=index+1) 
    {
        pos_1 = motor_1.set_torque(2, 0.0, &d_theta_1_t, &torque_1_t); 
        pos_2 = motor_2.set_torque(1, 0.0, &d_theta_2_t, &torque_2_t); 
    }

    run_on = 1; 

    // Catch a Ctrl-C event:
	void  (*sig_h)(int) = sigint_1_step;   // pointer to signal handler

    // Catch a Ctrl-C event: 
    signal(SIGINT, sig_h);  
 
    // dist > dist_threshold && initial_index < max_index
    while(run_on)  
    {
        ////////////////////////////////////////////////////////
        //// Initial Encoder and SEA Motor CAN
        ////////////////////////////////////////////////////////
        theta_1_t = motor_1.read_sensor(2) - theta_1_initial;  
        theta_2_t = -1 * (motor_2.read_sensor(1) + theta_1_t - theta_2_initial);  

        encoder.read_ang_encoder(encoder_arr);  

        q_1_t = (double) encoder_arr[1]*PI/180.0 - q_1_initial;  
        q_2_t = (double) encoder_arr[0]*PI/180.0 - q_2_initial;  

        dist = sqrt(pow((theta_1_t - q_1_target), 2) + pow((theta_2_t - q_2_target), 2));   

        printf(" theta_1_t: %f\n", theta_1_t);    
        printf(" theta_2_t: %f\n", theta_2_t);    

        /////////////////////////////////////////////////////
        //// calculate torque control command 
        ///////////////////////////////////////////////////// 
        torque_1 = clip(- stiffness * (q_1_target - theta_1_t) - damping * (d_theta_1_e - d_theta_1_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_1; 
        torque_2 = clip(- stiffness * (q_2_target - theta_2_t) - damping * (d_theta_2_e - d_theta_2_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_2; 

        // double torque_1_o = - K_p_1 * (theta_1_e - theta_1_t) - K_d_1 * (d_theta_1_e - d_theta_1_t);  
        // double torque_2_o = - K_p_2 * (theta_2_e - theta_2_t) - K_d_2 * (d_theta_2_e - d_theta_2_t);  

        OutFileAngle << theta_1_t << "," << theta_2_t << "\n";   

        // pos_1 = motor_1.set_torque(2, torque_1, &d_theta_1_t, &torque_1_t);    
        // pos_2 = motor_2.set_torque(1, torque_2, &d_theta_2_t, &torque_2_t);    

        pos_1 = motor_1.set_torque(2, 0.0, &d_theta_1_t, &torque_1_t);   
        pos_2 = motor_2.set_torque(1, torque_2, &d_theta_2_t, &torque_2_t);   

        OutFileTorque << torque_1_t << "," << torque_2_t << "\n";   

        // OutFileVel << d_theta_1_t << " " << d_theta_2_t << "\n";   

        // printf("d_theta_1_t: %f\n", d_theta_1_t);   
        // printf("d_theta_2_t: %f\n", d_theta_2_t);   
    }

    printf("Move to target done !!!! \n"); 

    OutFileAngle.close();   
    OutFileTorque.close();       

    motor_1.pack_stop_cmd(2);   
    motor_2.pack_stop_cmd(1);    

    return 1;  
}


void load_path_data(double *theta_1_list, double *theta_2_list)
{
    ////////////////////////////////////////////////////////
    // Load path from txt file
    ////////////////////////////////////////////////////////   
    int index_point = 0;    

    // int Num_waypoints = 19999;  
    // double theta_1_list[Num_waypoints];    
    // double theta_2_list[Num_waypoints];     

    string input_path = "angle_list.txt";    
    ifstream input_angle_list;    
    input_angle_list.open(input_path);    
    string s;  
    vector<string> angle_list;    

    while(getline(input_angle_list, s))  
    {
        // cout << s << endl;  
        split(s, angle_list, ",");   
        theta_1_list[index_point] = atof(angle_list[0].c_str());   
        // printf("theta 1: %f\n", theta_1_list[index_point]);  

        theta_2_list[index_point] = atof(angle_list[1].c_str());   
        // printf("theta 2: %f\n", theta_2_list[index_point]);  

        index_point += 1;  
    }
    printf("Num_waypoints: %d\n", index_point);   
    input_angle_list.close();   
}


py::array_t<double> get_demonstration(double theta_1_initial, double theta_2_initial, py::array_t<double> buff_size) 
{
    ////////////////////////////////////////////////////////
    //// Initial Encoder and Motor CAN
    ////////////////////////////////////////////////////////

    CANDevice can0((char *) "can0");  
    can0.begin();  
    CANDevice can1((char *) "can1");  
    can1.begin();  

    // motor_1(can1);  
    // motor_2(can0);  
    Gcan motor_1(can1);   
    Gcan motor_2(can0);   
    motor_1.begin();   
    motor_2.begin();   
    printf("Get demonstration start !!!!\n");  

    ////////////////////////////////////////////////////////
    // One loop control demonstration
    ////////////////////////////////////////////////////////

    // string output_torque = root_path + "torque_list.txt";  
    // ofstream OutFileTorque(output_torque);  
    // OutFileTorque << "torque_1" << " " << "torque_2" << "\n";  

    string output_angle = "demonstrated_angle_list.txt";    
    ofstream OutFileAngle(output_angle);    
    OutFileAngle << "angle_1" << "," << "angle_2" << "\n";   

    double theta_1_t = 0.0;   
    double theta_2_t = 0.0;   

    double d_theta_1_t = 0.0;    
    double d_theta_2_t = 0.0;    

    double torque_1 = 0.0;  
    double torque_2 = 0.0;  

    double torque_1_t = 0.0;  
    double torque_2_t = 0.0;  

    double pos_1 = 0.0;   
    double pos_2 = 0.0;   

    run_on = 1;   

    // Catch a Ctrl-C event:  
    // pointer to signal handler
	void (*sig_h)(int) = sigint_1_step;   

    py::buffer_info buff_size_list = buff_size.request(); 

    // allocate the output buffer
	py::array_t<double> result = py::array_t<double>(buff_size_list.size); 

    result.resize({buff_size_list.shape[0], buff_size_list.shape[1]}); 

    // acquire buffer info 
	py::buffer_info result_buf = result.request(); 

    double *theta_list = (double *)result_buf.ptr; 

    int index = 0; 
    int index_buff = 0; 
    int buff_length = 10000; 

    printf("Print enter for starting record !!!!\n");  
    getchar(); 

    while(run_on)  
    {
        // Catch a Ctrl-C event: 
        signal(SIGINT, sig_h); 
          
        theta_1_t = motor_1.read_sensor(2) - theta_1_initial;  
        theta_2_t = -1 * (motor_2.read_sensor(1) + theta_1_t - theta_2_initial);  

        printf(" theta_1_t: %f\n", theta_1_t);  
        printf(" theta_2_t: %f\n", theta_2_t);  

        OutFileAngle << theta_1_t << "," << theta_2_t << "\n";  

        index_buff = index%buff_length;  
        // theta_list[index_buff][0] = theta_1_t;  
        // theta_list[index_buff][1] = theta_2_t;  

        pos_1 = motor_1.set_torque(2, 0, &d_theta_1_t, &torque_1_t);   
        pos_2 = motor_2.set_torque(1, 0, &d_theta_2_t, &torque_2_t);   

        theta_list[index_buff*result_buf.shape[1] + 0] = theta_1_t; 
        theta_list[index_buff*result_buf.shape[1] + 1] = theta_2_t; 

        // for(i=0; i<result_buf.shape[0]; i++)
        // {
        //     for(j=0; j<result_buf.shape[1]; j++)
        //     {
                
        //     }
        // }
        // OutFileTorque << torque_1_t << " " << torque_2_t << "\n";   

        // OutFileVel << d_theta_1_t << " " << d_theta_2_t << "\n";   

        // printf("d_theta_1_t: %f\n", d_theta_1_t);   
        // printf("d_theta_2_t: %f\n", d_theta_2_t);   
        index = index + 1;    
    }

    return result;  
}  


int move_to_target_point(double stiffness_1, double stiffness_2,  
double damping_1, double damping_2,  
py::array_t<double> q_1_target, py::array_t<double> q_2_target, int N, 
double theta_1_initial, double theta_2_initial,  
double dist_threshold   
)
{
    ////////////////////////////////////////////////////////
    //// Initial Encoder and Motor CAN
    //////////////////////////////////////////////////////// 
    CANDevice can0((char *) "can0");  
    can0.begin();   
    CANDevice can1((char *) "can1");  
    can1.begin();   

    Gcan motor_1(can1);   
    Gcan motor_2(can0);   
    motor_1.begin();   
    motor_2.begin();   

    // printf("Move to target point start !!!!\n");   

    ////////////////////////////////////////////////////////
    // One loop control demonstration
    ////////////////////////////////////////////////////////
    string output_angle = "move_target_angle_list.txt";    
    ofstream OutFileAngle(output_angle);    
    OutFileAngle << "angle_1" << "," << "angle_2" << "\n";    

    string output_torque = "move_target_torque_list.txt";    
    ofstream OutFileTorque(output_torque);    
    OutFileTorque << "torque_1" << "," << "torque_2" << "\n";    

    double torque_lower_bound = -1.5;    
    double torque_upper_bound = 1.5;   
    
    // double ctl_ratio_1 = -2000.0/32;   
    // double ctl_ratio_2 = 2000.0/32;   

    double theta_1_t = 0.0;   
    double theta_2_t = 0.0;   

    double d_theta_1_t = 0.0;    
    double d_theta_2_t = 0.0;    

    double theta_1_e = 0.0;   
    double theta_2_e = 0.0;   

    double d_theta_1_e = 0.0;   
    double d_theta_2_e = 0.0;   

    double torque_1 = 0.0;    
    double torque_2 = 0.0;    

    double torque_1_t = 0.0;   
    double torque_2_t = 0.0;   

    double pos_1 = 0.0;      
    double pos_2 = 0.0;      
 
    double dist = 1.0;   
    // int initial_index = 0;   
    // int max_index = 10000;   

    py::buffer_info q_1_list_buf = q_1_target.request();  
    py::buffer_info q_2_list_buf = q_2_target.request();  
    double *q_1_list = (double *)q_1_list_buf.ptr; 
    double *q_2_list = (double *)q_2_list_buf.ptr;  

    // for(int i=0; i < N; i = i + 1)  
    // {
    //     printf("theta_1_t: %f\n", q_1_list[i]);      
    //     printf("theta_2_t: %f\n", q_2_list[i]);  
    //     // OutFileAngle << q_1_list[i] << "," << q_2_list[i] << "\n"; 
    // }

    /////////////////////////////////////////////////////
    /////  avoid large motion at starting points  ///////
    ///////////////////////////////////////////////////// 
    for(int index=0; index<5; index=index+1)  
    {
        pos_1 = motor_1.set_torque(2, 0.0, &d_theta_1_t, &torque_1_t); 
        pos_2 = motor_2.set_torque(1, 0.0, &d_theta_2_t, &torque_2_t); 
    }

    run_on = 1;  

    // Catch a Ctrl-C event:
	void  (*sig_h)(int) = sigint_1_step;   // pointer to signal handler
 
    int index = 0; 

    // dist > dist_threshold && initial_index < max_index
    while(run_on && index<N)  
    {
        // Catch a Ctrl-C event: 
        signal(SIGINT, sig_h);  

        theta_1_t = motor_1.read_sensor(2) - theta_1_initial;  
        theta_2_t = -1 * (motor_2.read_sensor(1) + theta_1_t - theta_2_initial);   

        dist = sqrt(pow((theta_1_t - q_1_list[index]), 2) + pow((theta_2_t - q_2_list[index]), 2));    
        // printf("theta_1_t: %f\n", theta_1_t);     
        // printf("theta_2_t: %f\n", theta_2_t);    

        /////////////////////////////////////////////////////
        // calculate torque control command 
        ///////////////////////////////////////////////////// 
        torque_1 = clip(-1 * stiffness_1 * (q_1_list[index] - theta_1_t) - damping_1 * (d_theta_1_e - d_theta_1_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_1; 
        torque_2 = clip(-1 * stiffness_2 * (q_2_list[index] - theta_2_t) - damping_2 * (d_theta_2_e - d_theta_2_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_2; 

        // double torque_1_o = - K_p_1 * (theta_1_e - theta_1_t) - K_d_1 * (d_theta_1_e - d_theta_1_t);  
        // double torque_2_o = - K_p_2 * (theta_2_e - theta_2_t) - K_d_2 * (d_theta_2_e - d_theta_2_t);  

        OutFileAngle << theta_1_t << "," << theta_2_t << "\n";   

        // pos_1 = motor_1.set_torque(2, torque_1, &d_theta_1_t, &torque_1_t);    
        // pos_2 = motor_2.set_torque(1, torque_2, &d_theta_2_t, &torque_2_t);    

        pos_1 = motor_1.set_torque(2, torque_1, &d_theta_1_t, &torque_1_t);   
        pos_2 = motor_2.set_torque(1, torque_2, &d_theta_2_t, &torque_2_t);   

        OutFileTorque << torque_1_t << "," << torque_2_t << "\n";   

        // OutFileVel << d_theta_1_t << " " << d_theta_2_t << "\n";   

        // printf("d_theta_1_t: %f\n", d_theta_1_t);   
        // printf("d_theta_2_t: %f\n", d_theta_2_t);  
        index = index + 1;  
    } 


    printf("dist : %f\n", dist);  

    // printf("dist : %f\n", dist); 

    OutFileAngle.close();   
    OutFileTorque.close();       

    motor_1.pack_stop_cmd(2);   
    motor_2.pack_stop_cmd(1);   

    // printf("Move to target point done !!!! \n");   

    return 1;  
}


int rotate_to_target(
    double stiffness, double damping,  
    double theta_target,  
    double theta_initial,   
    double dist_threshold,    
    int32_t torque_cmd
)
{
    ////////////////////////////////////////////////////////
    //// Initial Encoder and Motor CAN
    //////////////////////////////////////////////////////// 
    CANDevice can3((char *) "can3");   
    can3.begin();   

    Gcan motor_3(can3);   
    motor_3.begin();  

    printf("Rotate to target point !!!!\n");   

    double torque_lower_bound = -1.0;      
    double torque_upper_bound = 1.0;      

    double ctl_ratio = 2000.0/32;   

    double theta_t = 0.0;   

    double d_theta_t = 0.0;    

    double theta_e = 0.0;  

    double d_theta_e = 0.0;   

    double torque = 0.0;  

    double torque_t = 0.0;   

    double pos_1 = 0.0;  
    double curr_t = 0.0;        

    double dist = 0.0;  

    string output_angle = "real_angle_list.txt";   
    ofstream OutFileAngle(output_angle);   
    OutFileAngle << "angle_1" << "," <<  "torque_1" << "\n";   

    /////////////////////////////////////////////////////
    /////  avoid large motion at starting points  ///////
    /////////////////////////////////////////////////////
    run_on = 1;  

    // Catch a Ctrl-C event: 
	void  (*sig_h)(int) = sigint_1_step;    

    // Catch a Ctrl-C event:  
    signal(SIGINT, sig_h);    
 
    // dist > dist_threshold && initial_index < max_index
    while(run_on)  
    {
        theta_t = motor_3.read_single_turn(1) - theta_initial;    
        printf("theta_t: %f\n", theta_t);   

        ////////////////////////////////////////////////////////
        // Save Data
        ////////////////////////////////////////////////////////

        dist = sqrt(pow((theta_t - theta_target), 2));    

        /////////////////////////////////////////////////////
        // calculate torque control command 
        ///////////////////////////////////////////////////// 
        torque = clip(-1 * stiffness * (theta_target - theta_t) - damping * (d_theta_e - d_theta_t), torque_lower_bound, torque_upper_bound) * ctl_ratio;   
        curr_t = -1 * stiffness * (theta_target - theta_t) - damping * (d_theta_e - d_theta_t); 

        pos_1 = motor_3.set_torque(1, torque, &d_theta_t, &torque_t);   

        OutFileAngle << theta_t << "," << curr_t << "\n";  

    }

    printf("Rotate to target point !!!! \n");   
    OutFileAngle.close(); 
    motor_3.pack_stop_cmd(1);   

    return 1;  
}


int run_one_loop(double stiffness_1, double stiffness_2,  
double damping_1, double damping_2,  
py::array_t<double> theta_1_target, py::array_t<double> theta_2_target, int Num_waypoints,  
double theta_1_initial, double theta_2_initial, int num_episodes
)  
{
    //////////////////////////////////////////////////////// 
    // give way points :::: 
    //////////////////////////////////////////////////////// 
    double K_p_1 = stiffness_1;  
	double K_d_1 = damping_1;  

    double K_p_2 = stiffness_2;  
	double K_d_2 = damping_2;  

    ////////////////////////////////////////////////////////
    // Initial hardware ::: can device
    ////////////////////////////////////////////////////////

    // std::cout << "Initial Can0 and Can1 !!!" << std::endl;  
    CANDevice can0((char *) "can0");  
    can0.begin();  
    CANDevice can1((char *) "can1");  
    can1.begin();  

    Gcan motor_1(can1);   
    Gcan motor_2(can0);   
    motor_1.begin();   
    motor_2.begin();   

    std::cout << "Run One Loop !!!" << std::endl;   

    ////////////////////////////////////////////////////////
    // Define file to store data
    ////////////////////////////////////////////////////////
    string root_path = "";  

    string output_torque = root_path + "real_torque_list.txt";  
    ofstream OutFileTorque(output_torque);   
    OutFileTorque << "torque_1" << "," << "torque_2" << "\n";  

    string output_angle = root_path + "real_angle_list.txt";   
    ofstream OutFileAngle(output_angle);  
    OutFileAngle << "angle_1" << "," << "angle_2" << "\n";  

    string output_vel = root_path + "real_angle_vel_list.txt";   
    ofstream OutFileVel(output_vel);   
    OutFileVel << "vel_1" << "," << "vel_2" << "\n";   

    ////////////////////////////////////////////////////////
    // Impedance Parameters ::: input 
    ////////////////////////////////////////////////////////

    double torque_lower_bound = -2.0;   
    double torque_upper_bound = 2.0;   
    
    double theta_1_t = 0.0;   
    double theta_2_t = 0.0;   

    double d_theta_1_t = 0.0;    
    double d_theta_2_t = 0.0;    

    double theta_1_e = 0.0;   
    double theta_2_e = 0.0;   

    double d_theta_1_e = 0.0;  
    double d_theta_2_e = 0.0;  

    double torque_1 = 0.0;  
    double torque_2 = 0.0;  

    double torque_1_t = 0.0;  
    double torque_2_t = 0.0;  

    double pos_1 = 0.0;  
    double pos_2 = 0.0;  
    

    run_on = 1; 

    // Catch a Ctrl-C event:
	void  (*sig_h)(int) = sigint_1_step;   // pointer to signal handler

    // for(int index=0; index<Num_waypoints; index=index+1) 
    // {
    //     theta_1_e = theta_1_list[index]; 
    //     theta_2_e = theta_2_list[index]; 

    //     cout << "index" << index << "\n";
    //     cout << "theta_1_e" << theta_1_e << "\n";
    //     cout << "theta_2_e" << theta_2_e << "\n";
    // }

    ///////////////// load data ///////////////////////////
    ///////////////////////////////////////////////////////
    // int Num_waypoints = 19999;   
    // double theta_1_list[Num_waypoints] = {0};     
    // double theta_2_list[Num_waypoints] = {0};   
    // load_path_data(theta_1_list, theta_2_list);   

    py::buffer_info theta_1_list_buf = theta_1_target.request();   
    py::buffer_info theta_2_list_buf = theta_2_target.request();   
    double *theta_1_list = (double *)theta_1_list_buf.ptr;   
    double *theta_2_list = (double *)theta_2_list_buf.ptr;    

    // printf("Theta_1_list :: %f\n", theta_1_list);    
    // printf("Theta_2_list :: %f\n", theta_2_list);  

    // for(int index=0; index<Num_waypoints; index=index+1)  
    // {
    //     theta_1_e = theta_1_list[index];   
    //     theta_2_e = theta_2_list[index];   

    //     printf("theta_1_e :: %f\n", theta_1_e);    
    //     printf("theta_2_e :: %f\n", theta_2_e);   

    //     OutFileAngle << theta_1_e << "," << theta_2_e << "\n";    
    // }

    ///////////////////////////////////////////////////////
    // avoid large motion at starting points 
    ///////////////////////////////////////////////////////
    for(int index=0; index<20; index=index+1)  
    {
        pos_1 = motor_1.set_torque(2, 0.0, &d_theta_1_t, &torque_1_t);  
        pos_2 = motor_2.set_torque(1, 0.0, &d_theta_2_t, &torque_2_t);  
    }

    run_on = 1; 
    for (int epi=0; epi < num_episodes; epi=epi+1)
    {
        // Catch a Ctrl-C event: 
        signal(SIGINT, sig_h); 

        for (int index = 0; index<Num_waypoints; index=index+1)
        { 

            theta_1_e = theta_1_list[index];   
            theta_2_e = theta_2_list[index];   

            // printf("theta_1_e :: %f\n", theta_1_e);    
            // printf("theta_2_e :: %f\n", theta_2_e);    

            d_theta_1_e = 0.0;   
            d_theta_2_e = 0.0;   

            // if(index==0) 
            // {
            //     d_theta_1_e = 0.0;  
            //     d_theta_2_e = 0.0;  
            // }
            // else  
            // {
            //     d_theta_1_e = (theta_1_list[index] - theta_1_list[index-1])/d_t;  
            //     d_theta_2_e = (theta_2_list[index] - theta_2_list[index-1])/d_t;  
            // }

            // read joint angles 
            theta_1_t = motor_1.read_sensor(2) - theta_1_initial;   
            theta_2_t = -1 * (motor_2.read_sensor(1) + theta_1_t - theta_2_initial);    

            /////////////////////////////////////////////////////
            // set torque control command 
            ///////////////////////////////////////////////////// 
            torque_1 = clip(- K_p_1 * (theta_1_e - theta_1_t) - K_d_1 * (d_theta_1_e - d_theta_1_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_1; 
            torque_2 = clip(- K_p_2 * (theta_2_e - theta_2_t) - K_d_2 * (d_theta_2_e - d_theta_2_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_2; 

            double torque_1_o = - K_p_1 * (theta_1_e - theta_1_t) - K_d_1 * (d_theta_1_e - d_theta_1_t);   
            double torque_2_o = - K_p_2 * (theta_2_e - theta_2_t) - K_d_2 * (d_theta_2_e - d_theta_2_t);   

            // printf("input_torque_1_t: %f\n", torque_1);   
            // printf("input_torque_2_t: %f\n", torque_1);   

            pos_1 = motor_1.set_torque(2, torque_1, &d_theta_1_t, &torque_1_t);   
            pos_2 = motor_2.set_torque(1, torque_2, &d_theta_2_t, &torque_2_t);   

            ////////////////////////////////////////////////////////
            // Save Data
            ////////////////////////////////////////////////////////
            OutFileAngle << theta_1_t << "," << theta_2_t << "\n";   

            OutFileTorque << torque_1_o << "," << torque_2_o << "\n";   

            OutFileVel << d_theta_1_t << "," << d_theta_2_t << "\n"; 
            
            if (run_on==0)
            {
                break; 
            } 
            else
            {

            }
        }
        if (run_on==0)
        {
            break; 
        } 
        else
        {

        }
        printf("Episode: %\n", epi);    
    }    
    // for(int index=0; index<Num_waypoints; index=index+1)  
    // {
    //     theta_1_e = theta_1_list[index];   
    //     theta_2_e = theta_2_list[index];   

    //     // printf("theta_1_e :: %f\n", theta_1_e);    
    //     // printf("theta_2_e :: %f\n", theta_2_e);    

    //     d_theta_1_e = 0.0;   
    //     d_theta_2_e = 0.0;   

    //     // if(index==0) 
    //     // {
    //     //     d_theta_1_e = 0.0;  
    //     //     d_theta_2_e = 0.0;  
    //     // }
    //     // else  
    //     // {
    //     //     d_theta_1_e = (theta_1_list[index] - theta_1_list[index-1])/d_t;  
    //     //     d_theta_2_e = (theta_2_list[index] - theta_2_list[index-1])/d_t;  
    //     // }

    //     // read joint angles 
    //     theta_1_t = motor_1.read_sensor(2) - theta_1_initial;   
    //     theta_2_t = -1 * (motor_2.read_sensor(1) + theta_1_t - theta_2_initial);  

    //     /////////////////////////////////////////////////////
    //     // set torque control command 
    //     ///////////////////////////////////////////////////// 
    //     torque_1 = clip(- K_p_1 * (theta_1_e - theta_1_t) - K_d_1 * (d_theta_1_e - d_theta_1_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_1; 
    //     torque_2 = clip(- K_p_2 * (theta_2_e - theta_2_t) - K_d_2 * (d_theta_2_e - d_theta_2_t), torque_lower_bound, torque_upper_bound) * ctl_ratio_2; 

    //     double torque_1_o = - K_p_1 * (theta_1_e - theta_1_t) - K_d_1 * (d_theta_1_e - d_theta_1_t);  
    //     double torque_2_o = - K_p_2 * (theta_2_e - theta_2_t) - K_d_2 * (d_theta_2_e - d_theta_2_t);  

    //     printf("input_torque_1_t: %f\n", torque_1);   
    //     // printf("input_torque_2_t: %f\n", torque_1);   

    //     pos_1 = motor_1.set_torque(2, torque_1, &d_theta_1_t, &torque_1_t);   
    //     pos_2 = motor_2.set_torque(1, 0.0, &d_theta_2_t, &torque_2_t);   

    //     // pos_1 = motor_1.set_torque(2, torque_1, &d_theta_1_t, &torque_1_t);   
    //     // pos_2 = motor_2.set_torque(1, torque_2, &d_theta_2_t, &torque_2_t);  

    //     ////////////////////////////////////////////////////////
    //     // Save Data
    //     ////////////////////////////////////////////////////////
    //     OutFileAngle << theta_1_t << "," << theta_2_t << "\n";   

    //     OutFileTorque << torque_1_o << "," << torque_2_o << "\n";   

    //     OutFileVel << d_theta_1_t << "," << d_theta_2_t << "\n";   
    // }
    
    OutFileTorque.close();   
    OutFileAngle.close();   
    OutFileVel.close();   

    motor_1.pack_stop_cmd(2);    
    motor_2.pack_stop_cmd(1);     

    return 0;  
}

// void hardware_reset(Gcan *motor_1, Gcan *motor_2) 
// {
//     ////////////////////////////////////////////////////////
//     // Initial Encoder and Motor CAN
//     ////////////////////////////////////////////////////////

//     CANDevice can0((char *) "can0");  
//     can0.begin();  
//     CANDevice can1((char *) "can1");  
//     can1.begin();  

//     motor_1(can1);  
//     motor_2(can0);  
//     // Gcan motor_1(can1);  
//     // Gcan motor_2(can0);   
//     motor_1.begin();  
//     motor_2.begin();  


//     ///////////// initial encoder ////////
//     //////////////////////////////////////

//     // controller_renishaw encoder("can2");  
    
//     // float encoder_arr[2];  
// 	// encoder.read_ang_encoder(encoder_arr);   
//   	// double theta_1 = (double) encoder_arr[1]*PI/180.0;   
//   	// double theta_2 = (double) encoder_arr[0]*PI/180.0;    
//     // printf("Encoder 1 position: %f\n", theta_1);   
//     // printf("Encoder 2 position: %f\n", theta_2);  
// }


// int reset(double *theta_1_initial, double *theta_2_initial)
// {
//     ////////////////////////////////////////////////////////
//     // Initial Calibration  
//     ////////////////////////////////////////////////////////
//     // double theta_1_initial = motor_1.read_sensor(2); 
//     // printf(" Motor 1 initial position: %f\n", theta_1_initial); 

//     // double theta_2_initial = motor_2.read_sensor(1); 
//     // printf(" Motor 2 initial position: %f\n", theta_2_initial); 

//     theta_1_initial = motor_1.read_sensor(2);   
//     printf("Motor 1 initial position: %f\n", theta_1_initial);   

//     theta_2_initial = motor_2.read_sensor(1);  
//     printf("Motor 2 initial position: %f\n", theta_2_initial);   
    
//     return 1;  
// } 


namespace py = pybind11; 

PYBIND11_MODULE(motor_control, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: cmake_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers

        Some other explanation about the subtract function.
    )pbdoc"); 

    // m.def(
    //     "reset", &reset, R"pbdoc( 
    //     reset robot position

    //     Some other explanation about the add function. 
    // )pbdoc"); 

    m.def(
        "load_path_data", &load_path_data, R"pbdoc( 
        load path data from .txt file

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def(
        "read_angle_1", &read_angle_1, R"pbdoc( 
        read angle 1

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def(
        "read_angle_2", &read_angle_2, R"pbdoc( 
        read angle 2

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def(
        "read_angle_3", &read_angle_3, R"pbdoc( 
        read angle 3

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def(
        "set_position", &set_position, R"pbdoc( 
        set_position

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def(
        "rotate_to_target", &rotate_to_target, R"pbdoc( 
        rotate_to_target 

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def(
        "motor_3_stop", &motor_3_stop, R"pbdoc( 
        motor_3_stop

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def(
        "read_initial_angle_1", &read_initial_angle_1, R"pbdoc( 
        read_initial_angle_1 

        Some other explanation about the add function.  
    )pbdoc"); 

    m.def(
        "read_initial_angle_2", &read_initial_angle_2, R"pbdoc( 
        read_initial_angle_2

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def(
        "read_initial_angle_3", &read_initial_angle_3, R"pbdoc( 
        read_initial_angle_3

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def("get_demonstration", &get_demonstration, R"pbdoc(
        get_demonstration

        Some other explanation about the add function.
    )pbdoc"); 

    m.def("run_one_loop", &run_one_loop, R"pbdoc( 
        run_one_loop 

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def("move_to_target", &move_to_target, R"pbdoc(
        move_to_target

        Some other explanation about the add function.
    )pbdoc"); 

    m.def("move_to_target_point", &move_to_target_point, R"pbdoc(
        move_to_target_point

        Some other explanation about the add function.
    )pbdoc"); 
    
    // run_one_loop(double stiffness, double damping, double theta_1_initial, double theta_2_initial, int Num_waypoints)


#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev"; 
#endif
}