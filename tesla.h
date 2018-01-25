/*
 * Copyright © kiwipower 2017
 *
 * Header file to simulate battery charge/discharge profile
 */
#ifndef BATTSIM_DOT_H
#define BATTSIM_DOT_H

#include <stdint.h>
#include <modbus/modbus.h>
#include "typedefs.h"

#define enableDebug                   1
#define dumpMemory                    2
#define firmwareVersion               101
#define directRealTimeout             1023
#define directRealHeartbeat           1022
#define statusFullChargeEnergy        205
#define statusNorminalEnergy          207
#define directPower                   1020
#define realMode                      1000
#define powerBlock                    1002


// proclet
int process_enableDebug (uint16_t, uint16_t );
int process_dumpMemory (uint16_t, uint16_t );
int process_firmwareVersion (uint16_t, uint16_t );
int process_directRealTimeout (uint16_t, uint16_t );
int process_directRealHeartbeat( uint16_t, uint16_t );
int process_statusFullChargeEnergy(uint16_t, uint16_t );
int process_statusNorminalEnergy (uint16_t, uint16_t );
int process_directPower( uint16_t, uint16_t  );
int process_realMode( uint16_t, uint16_t  );
int process_powerBlock( uint16_t, uint16_t  );

int  process_handler(uint16_t, uint16_t);
void process_query(modbus_pdu_t*);
void *handler( void *ptr );
#endif
