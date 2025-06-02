#include <stdint.h>

#define E_STX 2
#define E_ETX 3

#define PACKED __attribute__((packed))

typedef union
{
	struct PACKED e_panid_protocol
	{
		uint8_t stx; // packet start idendifier
		uint8_t op;		// OP Code
		uint8_t len; // length
		uint16_t panid; // PAN ID
		uint8_t etx;	// packet end idendifier
	} e_Data;

	uint8_t buffer[6];
} E_PROTOCOL_PANID; //u_OP_SetPanID;