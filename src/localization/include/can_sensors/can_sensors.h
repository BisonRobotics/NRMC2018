#include <iostream>
//socketcan includes
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <stdint.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/bcm.h>

#include <sys/time.h>

class CanSensor {
	private:
		struct ifreq ifr;
		struct sockaddr_can addr;
		int s;
		int sbcm;
		int canID;

		struct timeval prevmsgtime;


		struct can_msg {
			struct bcm_msg_head msg_head;
			struct can_frame frame[1];
		} msg;

		int canSend(uint8_t *data, uint8_t len); //future feature

		

	public:
		int canRecieve(uint8_t *databuffer);
		CanSensor(int cID, char* interface);
		//int setRefreshRate(uint8_t Hz); //future feature
		//virtual int recieveData(); //future feature
};



