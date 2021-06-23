#include "gyems_can_functions.h" 
using namespace std; 
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

int main()
{
    std::cout << "Initial Can0 and Can1 !!!" << std::endl;  
    CANDevice can0((char *) "can0");  
    can0.begin(); 
    CANDevice can1((char *) "can1");   
    can1.begin(); 

    Gcan motor_1(can0); 
    Gcan motor_2(can1);   
    motor_1.begin(); 
    motor_2.begin(); 

    std::cout << "Enable motor!!!" << std::endl; 

    // motor_1.read_multi_turn_angle(1);  
    // motor_1.unpack_multi_turn_angle(struct can_frame motor_1.rframe, &multi_turn_position); 
    // motor_1.readcan();  
    // motor_1.read_sensor(*position); 
    
    ////////////////////////////////////////////////////////
    // Load path from txt file
    ////////////////////////////////////////////////////////
    int Num_waypoints; 
    double d_t = 0.001; 

    double theta_1_list[Num_waypoints]; 
    double theta_2_list[Num_waypoints]; 

    ////////////////////////////////////////////////////////
    // Impedance Parameters
    ////////////////////////////////////////////////////////
	double K_p = 20; 
	double K_d = 0.1; 
	double K_i = 100; 

    double theta_1_initial = motor_1.read_sensor(1); 
    printf(" Motor position 1: %f\n", theta_1_initial); 

    double theta_2_initial = motor_2.read_sensor(2); 
    printf(" Motor position 2: %f\n", theta_2_initial); 
    
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
        
        theta_1_t = motor_1.read_sensor(1) - theta_1_initial; 
        theta_2_t = motor_2.read_sensor(2) - theta_2_initial; 

        double d_theta_1_t = motor_1.read_sensor(1) - theta_1_initial; 
        double d_theta_2_t = motor_2.read_sensor(2) - theta_2_initial; 

        torque_1 = -1 * K_p * (theta_1_e - theta_1_t) - K_d * (d_theta_1_e - d_theta_1_t); 
        torque_2 = -1 * K_p * (theta_2_e - theta_2_t) - K_d * (d_theta_2_e - d_theta_2_t); 

        double pos_1 = motor_1.set_torque(1, torque_1, &d_theta_1_t, &torque_1_t); 
        double pos_1 = motor_2.set_torque(2, torque_2, &d_theta_2_t, &torque_2_t); 

        ////////////////////////////////////////////////////////
        // Save Data
        ////////////////////////////////////////////////////////
        
    }

}