/* Created by Sumantra on 9th December 2019 */
/* Ver 1.1 Major bug fix by Chang Hong 10 Dec 2019 */

#include "innfos_can_functions.hpp"
#include "misc_functions.hpp"

/*CAN READ AND WRITE FUNCTIONS*/

controller::controller(int writesocket_fd, int readsocket_fd)
{
    /*initialize rx and tx can messages structs*/
    for (int i = 0; i < 7; ++i)
    {	
        this->tx_msg.cframe.data[i] = 0;
        this->rx_msg.cframe.data[i] = 0;
    }
    this->tx_msg.cframe.can_dlc = 8; 
    this->rx_msg.cframe.can_dlc = 8; 
    write_socket = writesocket_fd;
    read_socket = readsocket_fd;
    //printf("read socket: %d, write socket: %d\n",read_socket,write_socket );
}

controller::controller(const char *can_iface_name)
{
    // Initialize CAN Bus Sockets
    int s,s2;
    struct sockaddr_can addrSource, addr1, addr2;
    struct can_frame frame,frame2;
    struct ifreq ifr;
    struct ifreq ifr2;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket!");
    }

    strcpy(ifr.ifr_name, can_iface_name);
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr1.can_family  = AF_CAN;
    addr1.can_ifindex = ifr.ifr_ifindex;

    if(bind(s, (struct sockaddr *)&addr1, sizeof(addr1)) < 0) {
        perror("Error in socket bind");
    }
    
    /*initialize rx and tx can messages structs*/
    for (int i = 0; i < 7; ++i)
    {	
        this->tx_msg.cframe.data[i] = 0;
        this->rx_msg.cframe.data[i] = 0;
    }
    this->tx_msg.cframe.can_dlc = 8; 
    this->rx_msg.cframe.can_dlc = 8; 
    write_socket = s;
    read_socket = NULL;
    //printf("read socket: %d, write socket: %d\n",read_socket,write_socket );
}

bool controller::can_read()
{	
    struct can_frame sframe;	
    int nbytes;
    //printf("tread 2 socket: %d can_id: %x\n",read_socket,sframe.can_id);	
    nbytes = read(read_socket, &(rx_msg.cframe), sizeof(struct can_frame));
    sframe = rx_msg.cframe;
    //printf("tread 2 socket: %d can_id: %x\n",read_socket,sframe.can_id);	
    //printf("data read nbytes:%d %d %d %d %d %d %d %d %d. Yay!\n",nbytes,sframe.data[0], sframe.data[1], sframe.data[2], sframe.data[3],
    // sframe.data[4], sframe.data[5], sframe.data[6], sframe.data[7]); 
  
    /* paranoid check ... */
    if (nbytes < sizeof(struct can_frame)) {
            perror("read: incomplete CAN frame\n");
            return 1;
    }
    
}

bool controller::can_write()
{	
    int nbytes;
    printf("can_write %d:",write_socket);
    
    nbytes = write(this->write_socket, &(tx_msg.cframe), sizeof(struct can_frame));
    if (nbytes < 0) {
            perror("can raw socket write");
            return 1;
    }

    /* paranoid check ... */
    if (nbytes < sizeof(struct can_frame)) {
            perror("write: incomplete CAN frame\n");
            return 1;
    }

}

/*MSG HANDLER FUNCTIONS FOR PROCESSING AND ORGANIZING INCOMING MSGS*/


void controller::set_vel_setpoint(uint32_t node_id, float vel_setpoint)
{
    uint32_t send_velocity=int(round((vel_setpoint/MAX_SPEED)*IQ_24));

    this->tx_msg.node_id = node_id;
    this->tx_msg.cframe.can_dlc = SIZE_WRITE_3;

    this->tx_msg.cframe.data[0] = SET_VEL_SETPOINT;
    //most significant bit at the left ahnd side
    this->tx_msg.cframe.data[4] = (send_velocity & BIT_MASK_0) ;
    this->tx_msg.cframe.data[3] = (send_velocity & BIT_MASK_1) >> 8;
    this->tx_msg.cframe.data[2] = (send_velocity & BIT_MASK_2) >> 16;
    this->tx_msg.cframe.data[1] = (send_velocity & BIT_MASK_3) >> 24;

    can_write();
    printf("speed %f rpm set to %d: (%x)\n",vel_setpoint, send_velocity,send_velocity);
    printf("1: (%x) 2: (%x) 3: (%x) 4: (%x) \n",tx_msg.cframe.data[1] , tx_msg.cframe.data[2] ,tx_msg.cframe.data[3],tx_msg.cframe.data[4]  );

}

void controller::enable_motor(uint32_t node_id)
{

    this->tx_msg.node_id = node_id;
    this->tx_msg.cframe.can_dlc = SIZE_WRITE_1;

    this->tx_msg.cframe.data[0] = ENABLE_DISABLE_MOTOR;
    this->tx_msg.cframe.data[1] = 0x1;

    can_write();
    printf("enabling motor\n");
}

void controller::disable_motor(uint32_t node_id)
{

    this->tx_msg.node_id = node_id;
    this->tx_msg.cframe.can_dlc = SIZE_WRITE_1;

    this->tx_msg.cframe.data[0] = ENABLE_DISABLE_MOTOR;
    this->tx_msg.cframe.data[1] = 0x0;
    can_write();
    printf("disabling motor\n");
}
    
void controller::change_mode(uint32_t node_id, uint8_t mode)
{

    this->tx_msg.node_id = node_id;
    this->tx_msg.cframe.can_dlc = SIZE_WRITE_1;

    this->tx_msg.cframe.data[0] = CHANGE_MODE;
    this->tx_msg.cframe.data[1] = mode;
    can_write();
    printf("change mode to %d\n",mode);
}
    


    