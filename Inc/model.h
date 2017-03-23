#ifndef _MVC_MODEL_
#define _MVC_MODEL_

#include "stdint.h"

#define FLAG_DIRECTION 			0x01
#define FLAG_VALUE_TYPE			0x02
#define FLAG_MACHINE_STATE	0x04

typedef enum _direction_ {FORWARD = 0x00, BACKWARD = 0x01} Direction;
typedef enum _value_type_ {DIFFERENCE = 0x00, EXACT = 0x02} ValueType;
typedef enum _machines_state_ {STOPED = 0x00, MOVED = 0x04} State;

typedef struct _car_property_ {
	
	int8_t LeftWheelVelocity;
	
	int8_t RightWheelVelocity;
	
	uint8_t flags;
	
	uint16_t alligin;
	
	uint16_t rx_metadata;
	
} Car;

#endif
