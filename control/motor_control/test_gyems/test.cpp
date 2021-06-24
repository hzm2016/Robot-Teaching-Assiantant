#include "gyems_can_functions.h" 
using namespace std; 
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdlib.h>

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

int main()
{
    ////////////////////////////////////////////////////////
    // Initial hardware ::: can device
    ////////////////////////////////////////////////////////

    // std::cout << "Initial Can0 and Can1 !!!" << std::endl;  
    // CANDevice can0((char *) "can0");  
    // can0.begin(); 
    // CANDevice can1((char *) "can1");   
    // can1.begin(); 

    // Gcan motor_1(can0); 
    // Gcan motor_2(can1);   
    // motor_1.begin(); 
    // motor_2.begin(); 

    // std::cout << "Enable motor!!!" << std::endl; 
    
    ////////////////////////////////////////////////////////
    // Load path from txt file
    ////////////////////////////////////////////////////////
    int Num_waypoints = 0; 
    double d_t = 0.001; 

    double theta_1_list[Num_waypoints]; 
    double theta_2_list[Num_waypoints]; 

    ifstream input_angle_list; 
    input_angle_list.open("2_font_3_angle_list.txt"); 
    string s; 
    vector<string> angle_list; 

    while(getline(input_angle_list, s))
    {
        // cout << s << endl; 
        split(s, angle_list, ","); 
        theta_1_list[Num_waypoints] = atof(angle_list[0].c_str()); 
        printf("theta 1: %f\n", theta_1_list[Num_waypoints]); 
        theta_2_list[Num_waypoints] = atof(angle_list[1].c_str()); 
        printf("theta 1: %f\n", theta_2_list[Num_waypoints]); 

        Num_waypoints += 1; 
    }
    printf("Num_waypoints: %d\n", Num_waypoints); 
    input_angle_list.close(); 

    ////////////////////////////////////////////////////////
    // Define file to store data
    ////////////////////////////////////////////////////////

    string output_torque = "torque_list.txt"; 
    ofstream OutFileTorque(output_torque); 

    string output_angle_1 = "angle_1_list.txt"; 
    ofstream OutFileAngle1(output_angle_1); 

    string output_angle_2 = "angle_2_list.txt"; 
    ofstream OutFileAngle2(output_angle_2); 

    ////////////////////////////////////////////////////////
    // Initial Calibration
    ////////////////////////////////////////////////////////
    // double theta_1_initial = motor_1.read_sensor(1); 
    // printf(" Motor position 1: %f\n", theta_1_initial); 

    // double theta_2_initial = motor_2.read_sensor(2); 
    // printf(" Motor position 2: %f\n", theta_2_initial); 

    ////////////////////////////////////////////////////////
    // Impedance Parameters
    ////////////////////////////////////////////////////////
	double K_p = 2; 
	double K_d = 0.1; 
	double K_i = 0.0; 

    double torque_lower_bound = -1; 
    double torque_upper_bound = 1; 
    
    double ctl_ratio = 1; 
    double ctl_ratio = 1; 

    double theta_1_t = 0.0; 
    double theta_2_t = 0.0; 

    double d_theta_1_t = 0.0; 
    double d_theta_2_t = 0.0; 

    double d_theta_1_e = 0.0; 
    double d_theta_2_e = 0.0; 

    double torque_1 = 0.0; 
    double torque_2 = 0.0; 

    double torque_1_t = 0.0; 
    double torque_2_t = 0.0; 

    double pos_1 = 0.0; 
    double pos_2 = 0.0; 

    for(int index=0;index<Num_waypoints;index=index+1) 
    {
        double theta_1_e = theta_1_list[index]; 
        double theta_2_e = theta_2_list[index]; 

        if(index==0) 
        {
            d_theta_1_e = 0.0; 
            d_theta_2_e = 0.0; 
        }
        else 
        {
            d_theta_1_e = (theta_1_list[index] - theta_1_list[index-1])/d_t; 
            d_theta_2_e = (theta_2_list[index] - theta_2_list[index-1])/d_t; 
        }
        
        // theta_1_t = motor_1.read_sensor(1) - theta_1_initial; 
        // theta_2_t = motor_2.read_sensor(2) - theta_2_initial; 

        // d_theta_1_t = motor_1.read_sensor(1) - theta_1_initial; 
        // d_theta_2_t = motor_2.read_sensor(2) - theta_2_initial; 

        torque_1 = clip(-1 * K_p * (theta_1_e - theta_1_t) - K_d * (d_theta_1_e - d_theta_1_t), torque_lower_bound, torque_upper_bound); 
        torque_2 = clip(-1 * K_p * (theta_2_e - theta_2_t) - K_d * (d_theta_2_e - d_theta_2_t), torque_lower_bound, torque_upper_bound); 

        // pos_1 = motor_1.set_torque(1, torque_1, &d_theta_1_t, &torque_1_t); 
        // pos_2 = motor_2.set_torque(2, torque_2, &d_theta_2_t, &torque_2_t); 

        ////////////////////////////////////////////////////////
        // Save Data
        ////////////////////////////////////////////////////////

    }

    OutFileTorque.close(); 
    OutFileAngle1.close(); 
    OutFileAngle2.close(); 
}