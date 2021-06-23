#include "gyems_can_functions.h" 
using namespace std; 
int main()
{
    std::cout << "Initial can new" << std::endl;  
    CANDevice can0((char *) "can0");  
    can0.begin(); 
    CANDevice can1((char *) "can1");   
    can1.begin(); 

    Gcan motor_1(can0); 
    Gcan motor_2(can1);   
    motor_1.begin(); 
    motor_2.begin(); 

    std::cout << "enable motor!" << std::endl; 
    std::cout << "begin finished new:::" << std::endl;  

    motor_1.read_multi_turn_angle(1);   
    motor_1.readcan();  
 
    motor_2.read_multi_turn_angle(2);   
    motor_2.readcan(); 

    // can_test.pack_torque_cmd(1, 0); 
    // can_test.pack_position_2_cmd(1, -36000, 360); 
    // can_test.read_PID(1); 

    // can_test.read_status_2_data(1); 
    // can_test.readcan(); 

    // can_test.write_acceleration2ram(1, 10);  
    // can_test.read_acceleration(1); 
    // std::cout<< "after !!!"<<std::endl;  

    // // can_test.read_single_turn_angle(1);  
    // can_test.readcan();
}