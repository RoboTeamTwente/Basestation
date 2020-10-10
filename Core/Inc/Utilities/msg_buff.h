
#ifndef __MSG_BUFF_STATUS_H
#define __MSG_BUFF_STATUS_H

#include <stdbool.h>
#include "BaseTypes.h"
#include "RobotCommand.h"

bool isTransmitting;


/*
 * create a struct that keeps track of all buffered messages and if they are new or not.
 *  when it is send isNew should be set to false.
 */
struct msgsBufferStatus {
	robotCommand command;
	bool isNewCommand;
	uint8_t feedback[PACKET_SIZE_ROBOT_FEEDBACK];
	bool isNewFeedback;
	uint8_t packetsSent;
	uint8_t packetsReceived;
};

struct msgsBufferStatus msgBuff[16];

uint8_t BS_DEBUG;


#endif // __MSG_BUFF_STATUS_H
