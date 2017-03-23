#ifndef _BUFFER_POOL_
#define _BUFFER_POOL_

#include "stdint.h"
#include "stdlib.h"
#include "model.h"

#define BUFFER_POOL_SIZE		10

/** 
  * @brief  CCBuffer status. FREE - the current node hasn`t receave any data yet, or receaving data has been reading.
	*					BUSY - the node contains usefull data, which has not reading yet.
  */
typedef enum _status_ {BUSY = 0x00, FREE = 0xFF} STATUS;

/** 
  * @brief  The main structure for motor`s control information
  */
typedef struct _car_control_buffer_ {
	
	Car payload;															/* The main structure for controling car information */
	
	struct _car_control_buffer_ *next;				/* The pointer to the next node CCBuffer */
	
	STATUS status;														/* Define the status of current node */
	
} CCBuffer;

CCBuffer *cc_buffer_init(CCBuffer **start_pos, uint8_t start_index);
void cc_buffer_dispose(CCBuffer **start_pos);

#endif
