/*
 * Header files for typedefs
 *
*/
#ifndef TYPEDEFS_DOT_H
#define TYPEDEFS_DOT_H

#include <modbus/modbus.h>

typedef enum {false, true} bool;

#define MODBUS_SUCCESS  0

typedef struct process_table_struct
{
	uint16_t address;
	uint8_t  number_registers;
	int (*handler)(uint16_t, uint16_t);
}process_table_t;


typedef struct optargs_struct
{
    unsigned int port;                           // port number for modbus server to listen
    unsigned int HoldingRegisters;               // Read - Write registers
    unsigned int NumberHoldingRegisters;         // Number of read - write registers
}optargs_t;


typedef struct thread_params_struct
{
    modbus_t *ctx;
	modbus_mapping_t* mb_mapping;
	pthread_mutex_t mutex;
	char *terminate;
}thread_param_t;

typedef struct mbap_header_struct
{
	uint16_t transport_id;
	uint16_t protocol_id;
	uint16_t length;
	uint8_t  unit_id;
}__attribute__((packed))mbap_header_t;

typedef struct modbus_pdu_struct
{
    mbap_header_t mbap;
	uint8_t  fcode;
	uint8_t  data[];
}__attribute__((packed))modbus_pdu_t;

#endif
