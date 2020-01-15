#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <net/if.h> 

#include <pthread.h>

#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <math.h>

#ifndef _INNFOS_CAN_H_
#define _INNFOS_CAN_H_

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
#define MAX_CURRENT 33.0


// Define IQ value form converter required by instructions as according to 
// Innfos wiki
// Basically 2 to the power of (IQ_value)
#define IQ_24       16777216
#define IQ_8        256


// Define size of can message according to Innfos wiki
#define SIZE_WRITE_1        2
#define SIZE_WRITE_2        3
#define SIZE_WRITE_3        5
#define SIZE_WRITE_4        1
#define SIZE_WRITE_5        6
#define SIZE_WRITE_RETURN        2

#define SIZE_READ           1
#define SIZE_READ_RETURN_1        2
#define SIZE_READ_RETURN_2        3
#define SIZE_READ_RETURN_3        5
#define SIZE_READ_RETURN_4        8
#define SIZE_READ_RETURN_5        5

// Define Modes
#define CURRENT_MODE    0x01
#define SPEED_MODE      0x02
#define POSITION_MODE   0x03
#define POSITION_TRAPEZOIDAL_MODE   0x06
#define SPEED_TRAPEZOIDAL_MODE      0x07
#define HOMING_MODE     0x08


// Define READ IDs
#define GET_HEARTBEAT        0x00
#define GET_MODE            0x55
#define GET_STATUS          0x2B

#define GET_ALARM              0xFF
#define GET_INVERTER_TEMP      0x94
#define GET_MOTOR_TEMP         0x94
#define GET_VBUS_VOLTAGE       0x45

#define GET_CUR        0x04
#define GET_VEL        0x05
#define GET_POS        0x06
#define GET_VEL_P_GAIN         0x17
#define GET_VEL_I_GAIN         0x18
#define GET_POS_P_GAIN         0x19
#define GET_POS_I_GAIN         0x1A

#define GET_ALL_3        0x94

#define GET_SERIAL_ID        0x02

// Define WRITE IDs
#define SET_CUR_SETPOINT        0x8
#define SET_VEL_SETPOINT        0x9
#define SET_POS_SETPOINT        0xA
#define ENABLE_DISABLE_MOTOR        0x2A
#define CHANGE_MODE        0x7

#define SET_VEL_P_GAIN         0x10
#define SET_VEL_I_GAIN         0x11
#define SET_POS_P_GAIN         0x12
#define SET_POS_I_GAIN         0x13

#define SET_CUR_FILTER        0x70
#define SET_VEL_FILTER        0x74
#define SET_POS_FILTER        0x78

#define SET_VEL_LIMIT_UPPER           0x31
#define SET_VEL_LIMIT_LOWER           0x30
#define SET_CUR_LIMIT_UPPER           0x2F
#define SET_CUR_LIMIT_LOWER           0x2E
#define SET_POS_LIMIT_UPPER           0x33
#define SET_POS_LIMIT_LOWER           0x32
#define SET_POS_TRAZ_VEL_LIMIT      0x1F
#define SET_POS_TRAZ_ACCEL_LIMIT    0x20
#define SET_POS_TRAZ_DECCEL_LIMIT    0x21
#define SET_VEL_TRAZ_VEL_LIMIT      0x25
#define SET_VEL_TRAZ_ACCEL_LIMIT    0x26
#define SET_VEL_TRAZ_DECCEL_LIMIT    0x27

#define CLEAR_ALARM    0xFE

// Define ALARM ids
#define OVERVOLTAGEERROR           0x0001
#define UNDERVOLTAGEERROR          0x0002
#define ABNORMALBLOCKING           0x0004
#define OVERHEATINGERROR           0x0008
#define READWRITEERROR             0x0010
#define MULTITURNCOUNTRROR         0x0020
#define INVERTERTEMPERATURESENSORERROR      0x0040
#define ABNORMALCOMMUNICATION               0x0080
#define MOTORTEMPERATURESENSORERROR         0x0100
#define STEPTOOBIG                          0x0200
#define DRVPROTECTIONERROR                  0x0400
#define DEVICEEXCEPTION            default

/* struct for storing the motor data points */

struct odrive_motor {
    float vel_setpoint; // 0.01 factor
};

struct can_frame_odrive {
    can_frame cframe;
    uint32_t node_id;
    uint32_t cmd_id;
};


class controller {
public:
    /*constructor*/
    controller(int writesocket_fd,int readsocket_f);
    controller(const char *can_iface_name);

    virtual ~controller() {};
    
    /* method to write tx_msg class member on the CAN BUS*/
    bool can_write();
    /*  method to read data from the CAN BUS and populate rx_data class member*/
    bool can_read();
   
    /* Setpoint Functions */
    void set_pos_setpoint(uint32_t node_id, float pos_setpoint);
    void set_vel_setpoint(uint32_t node_id, float vel_setpoint);
    void set_cur_setpoint(uint32_t node_id, float cur_setpoint);

    /* Motor enable/disable/mode functions */
    void enable_motor(uint32_t node_id);
    void disable_motor(uint32_t node_id);
    void change_mode(uint32_t node_id, uint8_t mode);

    /* Feedback Functions */
    
private:
    can_frame_odrive rx_msg;
    can_frame_odrive tx_msg;
    int read_socket;
    int write_socket;
   /*'legs' member variable, contains motor data for all the 12 motors*/
    odrive_motor motors[NO_OF_MOTOR+1];
};


#endif /* _CF_H_ */










