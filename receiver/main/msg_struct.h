#include <stdint.h>

#define E_STX 2
#define E_ETX 3

#define PACKED __attribute__((packed))

typedef union
{
	struct PACKED e_protocol_gps
	{
		uint8_t stx;		// packet start idendifier
		uint8_t op;			// OP Code (0x01: GPS Data)
		uint8_t len;		// length
		uint16_t id;		// ID
		uint8_t northSouth; // north south
		uint8_t eastWest;	// east west
		int32_t lat;		// latitude
		int32_t lon;		// longitude
		int32_t spd;		// speed
		int32_t voltage;	// voltage
		uint8_t etx;		// packet end idendifier
	} e_Data;

	uint8_t buffer[24];
} E_PROTOCOL_GPS;

typedef union
{
	struct PACKED e_protocol_remocon
	{
		uint8_t stx;  // packet start idendifier
		uint8_t op;	  // OP Code (0x02: Remote Control)
		uint8_t len;  // length
		uint16_t id;		// ID
		uint8_t mode; // north south
		uint8_t etx;  // packet end idendifier
	} e_Data;

	uint8_t buffer[5];
} E_PROTOCOL_REMOCON;

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