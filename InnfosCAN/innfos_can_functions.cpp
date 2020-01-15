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
    int s;
    struct sockaddr_can addr1;
    //struct can_frame frame;
    struct ifreq ifr;

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
    read_socket = 0;
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
    if (nbytes < (int) sizeof(struct can_frame)) {
            perror("read: incomplete CAN frame\n");
            return 1;
    }

    return 0;    
}

bool controller::can_write()
{	
    int nbytes;

    #ifndef NDEBUG
    printf("can_write %d:",write_socket);
    #endif

    nbytes = write(this->write_socket, &(tx_msg.cframe), sizeof(struct can_frame));
    if (nbytes < 0) {
            perror("can raw socket write");
            return 1;
    }

    /* paranoid check ... */
    if (nbytes < (int) sizeof(struct can_frame)) {
            perror("write: incomplete CAN frame\n");
            return 1;
    }

    return 0;
}

/*MSG HANDLER FUNCTIONS FOR PROCESSING AND ORGANIZING INCOMING MSGS*/

void controller::set_pos_setpoint(uint32_t node_id, float pos_setpoint)
{
	//pos_setpoint between -128 to 127.999

	uint32_t send_position=int(round(pos_setpoint*IQ_24));
	
	this->tx_msg.node_id = node_id;
	this->tx_msg.cframe.can_dlc = SIZE_WRITE_3;

	this->tx_msg.cframe.data[0] = SET_POS_SETPOINT;
	//most significant bit at the left ahnd side
	this->tx_msg.cframe.data[4] = (send_position & BIT_MASK_0) ;
	this->tx_msg.cframe.data[3] = (send_position & BIT_MASK_1) >> 8;
	this->tx_msg.cframe.data[2] = (send_position & BIT_MASK_2) >> 16;
	this->tx_msg.cframe.data[1] = (send_position & BIT_MASK_3) >> 24;

	can_write();

    #ifndef NDEBUG
	printf("speed %f rpm set to %d: (%x)\n",pos_setpoint, send_position,send_position);
	#endif
}

void controller::set_vel_setpoint(uint32_t node_id, float vel_setpoint)
{
    //vel_setpoint between -6000 to 6000

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
    #ifndef NDEBUG
    printf("speed %f rpm set to %d: (%x)\n",vel_setpoint, send_velocity,send_velocity);
    #endif
}

void controller::set_cur_setpoint(uint32_t node_id, float cur_setpoint)
{
	//cur_setpoint between -33 to 33

	uint32_t send_current=int(round((cur_setpoint/MAX_CURRENT)*IQ_24));
	
	this->tx_msg.node_id = node_id;
	this->tx_msg.cframe.can_dlc = SIZE_WRITE_3;

	this->tx_msg.cframe.data[0] = SET_CUR_SETPOINT;
	//most significant bit at the left ahnd side
	this->tx_msg.cframe.data[4] = (send_current & BIT_MASK_0) ;
	this->tx_msg.cframe.data[3] = (send_current & BIT_MASK_1) >> 8;
	this->tx_msg.cframe.data[2] = (send_current & BIT_MASK_2) >> 16;
	this->tx_msg.cframe.data[1] = (send_current & BIT_MASK_3) >> 24;

	can_write();
    #ifndef NDEBUG
	printf("speed %f A set to %d: (%x)\n",cur_setpoint, send_current,send_current);
	#endif
}

void controller::enable_motor(uint32_t node_id)
{

    this->tx_msg.node_id = node_id;
    this->tx_msg.cframe.can_dlc = SIZE_WRITE_1;

    this->tx_msg.cframe.data[0] = ENABLE_DISABLE_MOTOR;
    this->tx_msg.cframe.data[1] = 0x1;

    can_write();
    #ifndef NDEBUG
    printf("enabling motor\n");
    #endif
}

void controller::disable_motor(uint32_t node_id)
{

    this->tx_msg.node_id = node_id;
    this->tx_msg.cframe.can_dlc = SIZE_WRITE_1;

    this->tx_msg.cframe.data[0] = ENABLE_DISABLE_MOTOR;
    this->tx_msg.cframe.data[1] = 0x0;
    can_write();
    #ifndef NDEBUG
    printf("disabling motor\n");
    #endif
}
    
void controller::change_mode(uint32_t node_id, uint8_t mode)
{

    this->tx_msg.node_id = node_id;
    this->tx_msg.cframe.can_dlc = SIZE_WRITE_1;

    this->tx_msg.cframe.data[0] = CHANGE_MODE;
    this->tx_msg.cframe.data[1] = mode;
    can_write();
    #ifndef NDEBUG
    printf("change mode to %d\n",mode);
    #endif
}
    


    