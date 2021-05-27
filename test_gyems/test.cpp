#include "gyems_can_functions.h" 
using namespace std; 
int main()
{
    std::cout << "initial can new" << std::endl;
    CANDevice can0((char *) "can0");
    can0.begin();
    std::cout << "begin finished :::" << std::endl;

    Gcan can_test(can0);
    std::cout << "enable motor!" << std::endl;
    can_test.begin(); 
    std::cout << "begin finished new:::" << std::endl; 

    // can_test.read_encoder(2); 
    // can_test.readcan(); 
     
    // can_test.read_single_turn_angle(1);
    // can_test.readcan();

    can_test.read_multi_turn_angle(1);
    can_test.readcan();

    // can_test.pack_torque_cmd(1, 0);

    // can_test.pack_position_2_cmd(1, -36000, 360);

    can_test.read_multi_turn_angle(1);
    can_test.readcan();
    // can_test.read_PID(1);

    // can_test.read_status_2_data(1);
    // can_test.readcan();

    // can_test.write_acceleration2ram(1, 10);
    // can_test.read_acceleration(1);
    // std::cout<< "after !!!"<<std::endl;

    // // can_test.read_single_turn_angle(1);
    // can_test.readcan();
}