#include <iostream> 
#include <fstream> 
#include <string>
#include <vector>
#include <stdlib.h> 
#include <pybind11/pybind11.h> 
#include "gyems_can_functions.h" 
#include "renishaw_can_functions.hpp" 
using namespace std; 

#define PI 3.1415926

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)


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

int add(int i, int j) {
    return i + 2 * j; 
} 

int encode_motor_test() 
{ 

    controller_renishaw encoder("can2");  

    float encoder_arr[2];  

	encoder.read_ang_encoder(encoder_arr);   
  	double theta_1 = (double) encoder_arr[1]*PI/180.0;   
  	double theta_2 = (double) encoder_arr[0]*PI/180.0;   
    printf("Encoder 1 position: %f\n", theta_1);  
    printf("Encoder 2 position: %f\n", theta_2);  

    return 1;  
} 


double read_initial_angle_1()
{
    CANDevice can1((char *) "can1");  
    can1.begin();  

    Gcan motor_1(can1);   
    motor_1.begin();  
    
    double theta_1_initial = motor_1.read_sensor(2);   
    printf("Motor 1 initial position: %f\n", theta_1_initial);  

    return theta_1_initial; 
}

double read_initial_angle_2()
{
    CANDevice can0((char *) "can0");  
    can0.begin();  

    Gcan motor_2(can0);   
    motor_2.begin();  

    double theta_2_initial = motor_2.read_sensor(1);  
    printf("Motor 2 initial position: %f\n", theta_2_initial);  

    return theta_2_initial;  
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

int get_demonstration(double theta_1_initial, double theta_2_initial)
{
    ////////////////////////////////////////////////////////
    //// Initial Encoder and Motor CAN
    ////////////////////////////////////////////////////////
    // Gcan motor_1; 
    // Gcan motor_2; 
    // hardware_reset(&motor_1, &motor_2); 

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
    printf("Hardware set well demonstration start !!!!\n");  

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

    while(true) 
    {
        theta_1_t = motor_1.read_sensor(2) - theta_1_initial;  
        theta_2_t = -1 * (motor_2.read_sensor(1) + theta_1_t - theta_2_initial);  

        printf(" theta_1_t: %f\n", theta_1_t);  
        printf(" theta_2_t: %f\n", theta_2_t);  

        OutFileAngle << theta_1_t << "," << theta_2_t << "\n";  

        pos_1 = motor_1.set_torque(2, 0, &d_theta_1_t, &torque_1_t);   
        pos_2 = motor_2.set_torque(1, 0, &d_theta_2_t, &torque_2_t);   

        // OutFileTorque << torque_1_t << " " << torque_2_t << "\n";   

        // OutFileVel << d_theta_1_t << " " << d_theta_2_t << "\n";   

        // printf("d_theta_1_t: %f\n", d_theta_1_t);   
        // printf("d_theta_2_t: %f\n", d_theta_2_t);   
    }

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
        printf("theta 1: %f\n", theta_1_list[index_point]);  

        theta_2_list[index_point] = atof(angle_list[1].c_str());   
        printf("theta 2: %f\n", theta_2_list[index_point]);  

        index_point += 1;  
    }
    printf("Num_waypoints: %d\n", index_point);   
    input_angle_list.close();   
}


int run_one_loop(double stiffness, double damping, double theta_1_initial, double theta_2_initial, int Num_waypoints) 
{
    //////////////////////////////////////////////////////// 
    // give way points :::: 
    //////////////////////////////////////////////////////// 

    printf("Input stiffness :: %f\n", stiffness);  
    printf("Input damping :: %f\n", damping);  

    double K_p_1 = stiffness;   // 16
	double K_d_1 = damping;   // 0.8

    double K_p_2 = stiffness;   
	double K_d_2 = damping;   

    ////////////////////////////////////////////////////////
    // Initial offset angles ::: input 
    //////////////////////////////////////////////////////// 
    // double theta_1_initial = -0.294288;   
    // double theta_2_initial = 0.402938;   

    ////////////////////////////////////////////////////////
    // Initial hardware ::: can device
    ////////////////////////////////////////////////////////

    std::cout << "Initial Can0 and Can1 !!!" << std::endl;  
    CANDevice can0((char *) "can0");  
    can0.begin();  
    CANDevice can1((char *) "can1");  
    can1.begin(); 

    Gcan motor_1(can1);   
    Gcan motor_2(can0);   
    motor_1.begin();   
    motor_2.begin();   

    std::cout << "Enable Motors Done!!!" << std::endl;   

    ////////////////////////////////////////////////////////
    // Define file to store data
    ////////////////////////////////////////////////////////

    // string root_path = "home/zhimin/code/8_nus/robotic-teaching/control/motor_control"; 
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

    double torque_lower_bound = -2.5;   
    double torque_upper_bound = 2.5;   
    
    double ctl_ratio_1 = - 2000.0/32;   
    double ctl_ratio_2 = 2000.0/32;  

    double d_t = 0.001;   

    
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
    

    // for(int index=0; index<Num_waypoints; index=index+1) 
    // {
    //     theta_1_e = theta_1_list[index]; 
    //     theta_2_e = theta_2_list[index]; 

    //     cout << "index" << index << "\n";
    //     cout << "theta_1_e" << theta_1_e << "\n";
    //     cout << "theta_2_e" << theta_2_e << "\n";
    // }

    // int Num_waypoints = 19999; 
    double theta_1_list[Num_waypoints] = {0};    
    double theta_2_list[Num_waypoints] = {0};   

    ///////////////// load data ///////////////////////////
    ///////////////////////////////////////////////////////

    load_path_data(theta_1_list, theta_2_list);   

    printf("Theta_1_list :: %f\n", theta_1_list[0]);    
    printf("Theta_2_list :: %f\n", theta_2_list[0]);    

    ///////////////////////////////////////////////////////
    // avoid large motion at starting points
    ///////////////////////////////////////////////////////
    // for(int index=0; index<5; index=index+1) 
    // {
    //     pos_1 = motor_1.set_torque(2, 0.0, &d_theta_1_t, &torque_1_t); 
    //     pos_2 = motor_2.set_torque(1, 0.0, &d_theta_2_t, &torque_2_t); 
    // }

    // for(int index=0; index<Num_waypoints; index=index+1) 
    // {
    //     theta_1_e = theta_1_list[index];  
    //     theta_2_e = theta_2_list[index];  

    //     if(index==0) 
    //     {
    //         d_theta_1_e = 0.0; 
    //         d_theta_2_e = 0.0; 
    //     }
    //     else 
    //     {
    //         d_theta_1_e = (theta_1_list[index] - theta_1_list[index-1])/d_t; 
    //         d_theta_2_e = (theta_2_list[index] - theta_2_list[index-1])/d_t; 
    //     }
        
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

    //     // printf("input_torque_1_t: %f\n", torque_1); 
    //     // printf("input_torque_2_t: %f\n", torque_1); 

    //     pos_1 = motor_1.set_torque(2, torque_1, &d_theta_1_t, &torque_1_t); 
    //     pos_2 = motor_2.set_torque(1, torque_2, &d_theta_2_t, &torque_2_t); 


    //     ////////////////////////////////////////////////////////
    //     // Save Data
    //     ////////////////////////////////////////////////////////

    //     OutFileAngle << theta_1_t << "," << theta_2_t << "\n";  

    //     OutFileTorque << torque_1_o << "," << torque_2_o << "\n"; 

    //     OutFileVel << d_theta_1_t << "," << d_theta_2_t << "\n"; 
    // }
    
    // OutFileTorque.close();   
    // OutFileAngle.close();    
    // OutFileVel.close();   

    // motor_1.pack_stop_cmd(2);  
    // motor_2.pack_stop_cmd(1);  

    return 0;  
}


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

    m.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc"); 

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
        "read_initial_angle_1", &read_initial_angle_1, R"pbdoc( 
        read initial angle 1

        Some other explanation about the add function. 
    )pbdoc"); 

    m.def(
        "read_initial_angle_2", &read_initial_angle_2, R"pbdoc( 
        read initial angle 2

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

    // run_one_loop(double stiffness, double damping, double theta_1_initial, double theta_2_initial, int Num_waypoints)


#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}