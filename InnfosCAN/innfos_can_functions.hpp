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

#ifndef _CF_H_
#define _CF_H_

#define RTR 1

#define BIT_MASK_0 0xFF
#define BIT_MASK_1 0xFF00
#define BIT_MASK_2 0xFF0000
#define BIT_MASK_3 0xFF000000
#define BIT_MASK_4 0xFF00000000
#define BIT_MASK_5 0xFF0000000000
#define BIT_MASK_6 0xFF000000000000
#define BIT_MASK_7 0xFF00000000000000

using namespace std;

// define motor array index
#define NO_OF_MOTOR       1

#define TEST       5

#define MAX_SPEED 6000.0
/*
#define FRONT_LEFT_HIP        2
#define FRONT_LEFT_ABDUCTION  3
#define FRONT_RIGHT_KNEE      4
#define FRONT_RIGHT_HIP       5
#define FRONT_RIGHT_ABDUCTION 6
#define BACK_LEFT_KNEE        7
#define BACK_LEFT_HIP         8
#define BACK_LEFT_ABDUCTION   9
#define BACK_RIGHT_KNEE       10
#define BACK_RIGHT_HIP        11
#define BACK_RIGHT_ABDUCTION  12

#define CLAW_LEFT_1        13
#define CLAW_LEFT_2        14
#define CLAW_RIGHT_1       15
#define CLAW_RIGHT_2       16
*/

//define IQ value form converter required by instructions as according to Innfos wiki
//basically 2 to the power of (IQ_value)
#define IQ_24       16777216
#define IQ_8        256


//define size of can message according to Innfos wiki
#define SIZE_WRITE_1        2
#define SIZE_WRITE_2        3
#define SIZE_WRITE_3        5
#define SIZE_WRITE_4        1


#define CURRENT_MODE    0x01
#define SPEED_MODE      0x02
#define POSITION_MODE   0x03
#define POSITION_TRAPEZOIDAL_MODE   0x06
#define SPEED_TRAPEZOIDAL_MODE      0x07
#define HOMING_MODE     0x08


//define command ids
#define SET_VEL_SETPOINT        0x9
#define ENABLE_DISABLE_MOTOR        0x2A
#define CHANGE_MODE        0x7

/*#define ODRIVE_HEARTBEAT        1
#define ESTOP                   2
#define GET_MOTOR_ERROR         3
#define GET_ENCODER_ERROR       4
#define SET_AXIS_NODE_ID        6
#define SET_AXIS_REQUESTED_STATE  7
#define GET_ENCODER_ESTIMATE    9 
#define GET_ENCODER_COUNTS      10
#define MOVE_TO_POS             11
#define SET_POS_SETPOINT        12
#define SET_VEL_SETPOINT        13
#define SET_CUR_SETPOINT        14
#define SET_VEL_LIMIT           15
#define START_ANTI_COGGING      16
#define SET_TRAJ_VEL_LIMIT      17
#define SET_TRAJ_ACCEL_LIMIT    18
#define SET_TRAJ_A_PER_CSS      19
#define GET_IQ_VALUES           20
#define REBOOT_ODRIVE           22
#define GET_VBUS_VOLTAGE        23
#define SET_VEL_PI_GAIN         24
*/

/* struct for storing the motor data points */

struct odrive_motor
{

    float vel_setpoint; //0.01 factor

};

struct can_frame_odrive
{
        can_frame cframe;
        uint32_t node_id;
        uint32_t cmd_id;
       
};


class controller{

public:
    /*constructor*/
    controller(int writesocket_fd,int readsocket_f);
    controller(const char *can_iface_name);

    virtual ~controller() {};
    
    /* method to write tx_msg class member on the CAN BUS*/
    bool can_write();
    /*  method to read data from the CAN BUS and populate rx_data class member*/
    bool can_read();
   
    /* methods that set ODrive parameters*/
    void set_vel_setpoint(uint32_t node_id, float vel_setpoint);
    void enable_motor(uint32_t node_id);
    void disable_motor(uint32_t node_id);
    void change_mode(uint32_t node_id, uint8_t mode);

    
    
private:
    can_frame_odrive rx_msg;
    can_frame_odrive tx_msg;
    int read_socket;
    int write_socket;
   /*'legs' member variable, contains motor data for all the 12 motors*/
    odrive_motor motors[NO_OF_MOTOR+1];
    
    
};


#endif /* _CF_H_ */










