#include <stdio.h>
#include <stdlib.h>
#include <byteswap.h>
#include "tesla.h"
#include "typedefs.h"
#include <unistd.h>
#include <modbus/modbus.h>
#include <string.h>
#include <time.h>

#define BATTERY_POWER_RATING            230           // kW
#define TIME_CHARGE_FROM_0_TO_100       3000          // seconds
#define TIME_DISCHARGE_FROM_100_TO_0    2800          // seconds
#define HEARTBEAT_TIMEOUT_DEFAULT       60
#define STATE_OF_CHARGET_DEFAULT        50.0

// Private data
static modbus_t* ctx;
static modbus_mapping_t *mb_mapping;

static uint16_t heartbeatTimeout = HEARTBEAT_TIMEOUT_DEFAULT;
static uint16_t heartbeat = 0;

static int32_t StatusFullChargeEnergy = 0;

static uint16_t averagesoc_multiplier = 10;
static float battery_discharge_decrement = 0.0;
static float battery_charge_increment = 0.0;
static bool battery_charging = false;
static bool battery_discharging = false;

static const uint32_t sign_bit_mask             = 0x80000000;

static const float battery_charge_resolution    = 100.00 / (BATTERY_POWER_RATING * TIME_CHARGE_FROM_0_TO_100);     // % increase in charge per sec
static const float battery_discharge_resolution = 100.00 / (BATTERY_POWER_RATING * TIME_DISCHARGE_FROM_100_TO_0);  // % decrease in charge per sec
static const float battery_fully_charged        = 100.00;
static const float battery_fully_discharged     = 0.0;
static float state_of_charge = STATE_OF_CHARGET_DEFAULT;

//
// Lookup table for process functions
//
const process_table_t process_table[] =
{
    {directRealTimeout,     0,      process_directRealTimeout},  // 16 bits
    {directRealHeartbeat,   0,    process_directRealHeartbeat},  // 16 bits
    {statusFullChargeEnergy,1, process_statusFullChargeEnergy},  // 32 bits
    {statusNorminalEnergy,  1,   process_statusNorminalEnergy},
    {directPower,           1,            process_directPower},
	{realMode,              0,               process_realMode},
    { 0,                    0,                           NULL}
};

//
// Acks and dismisses alarms
//
int process_realMode (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    printf("%s\n", __PRETTY_FUNCTION__);
    return retval;
}
//
// Acks and dismisses alarms
//process_directRealTimeout
int process_directRealTimeout (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    int heartbeatTimeout_old = heartbeatTimeout;
    heartbeatTimeout = value;
    printf("%s old heartbeatTimeout = %d, new heartbeatTimeout = %d\n", __PRETTY_FUNCTION__, heartbeatTimeout_old, heartbeatTimeout );
    heartbeat = 0;
    return retval;
}

//
// Heartbeat signal. Expected to toggle heartbeat bit very PGM HB Period
//
int process_directRealHeartbeat (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    static uint16_t previous_value = 0;

    printf("%s - value:%04x \n", __PRETTY_FUNCTION__, value);

    if ( previous_value == value )
    {
        heartbeat = 0;
    }
    previous_value = ~value;
    return retval;
}


int process_statusFullChargeEnergy(uint16_t index, uint16_t value)
{
    uint16_t *address;
    uint16_t address_offset;
    int retval = MODBUS_SUCCESS; // need to figure out what this constant is

    address_offset = mb_mapping->start_registers + statusFullChargeEnergy;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
        address[index] = value;
        if ( index == 0 )                         // 32 bits word most significant work
        {
        	StatusFullChargeEnergy = value << 16;
        }
        else
        {
        	StatusFullChargeEnergy += value;     // 32 bits word least significant work
        }
    }
    printf("%s value = %d, StatusFullChargeEnergy %d\n", __PRETTY_FUNCTION__, value, StatusFullChargeEnergy );
    return retval;
}


int process_statusNorminalEnergy(uint16_t index, uint16_t value)
{
    uint16_t *address;
    uint16_t address_offset;
    int retval = MODBUS_SUCCESS;
    uint32_t temp;

    printf("%s \n", __PRETTY_FUNCTION__ );
    address_offset = mb_mapping->start_registers + statusNorminalEnergy;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
    	if ( index == 0 )
    	{
    		temp = (uint32_t) (state_of_charge/StatusFullChargeEnergy);
    		address[index] = (temp << 16);
    	}
    	else
    	{
    		address[index] += temp & 0x0000FFFF;
    	}
    }

    return retval;
}


//
// Total real power being delivered in kW: range(-32768  to 32767)
//
int process_directPower(uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    static uint32_t val;

    if ( index == 0 )
    {
    	val = value << 16;
    }
    else
    {
    	val  += value;                             // store set point value
		if ( val & sign_bit_mask )
		{
			val = ((~val) + 1);                     // get 2nd complement value
			printf("%s - battery charging val(-%d)\n", __PRETTY_FUNCTION__, val);
			battery_charging = true;
			battery_discharging = false;
			battery_charge_increment = ( val * battery_charge_resolution);  ;
		}
		else if (val > 0)
		{
			printf("%s - battery discharging val(%d)\n", __PRETTY_FUNCTION__, val);
			battery_discharging = true;
			battery_charging = false;
			battery_discharge_decrement = (val * battery_discharge_resolution);
		}
		else
		{
			printf("%s - not charging val(%d)\n", __PRETTY_FUNCTION__, val);
			battery_discharging = false;
			battery_charging = false;
		}
    }

    return retval;
}


int process_handler(uint16_t address, uint16_t data)
{
    int retval = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

    for ( const process_table_t *p = process_table; p->handler != 0; p++ )
    {
    	if ( ( address >= p->address ) && (address <= (p->number_registers + p->address)) ) {
            retval = p->handler(address - p->address, data);
            break;
        }
    }
    return retval;
}



/*
***************************************************************************************************************
 \fn      process_handler(uint8_t* pdata)
 \brief   processess all incoming commands

 Process all input commands. The Modbus function code 0x17 which is not standard seems to exhibit non standaard
 data structure seen not belows.

 \note

      MODBUS_FC_READ_HOLDING_REGISTERS
      MODBUS_FC_WRITE_SINGLE_REGISTER - has the following data format
      ------------------------------------------------
      | TID | PID | LEN | UID | FC | [W|R]S | [W|R]Q |
      ------------------------------------------------
      0     1     3     5     7    8        11       13

      MODBUS_FC_WRITE_MULTIPLE_REGISTERS - has the following data format operation
      -------------------------------------------------------
      | TID | PID | LEN | UID | FC | WS | WQ | WC | WR x nn |
      -------------------------------------------------------
      0     1     3     5     7    8    11   13   14

      MODBUS_FC_WRITE_AND_READ_REGISTERS - has the following data format
      -----------------------------------------------------------------
      | TID | PID | LEN | UID | FC | RS | RQ | WS | WQ | WC | WR x nn |
      -----------------------------------------------------------------
      0     1     3     5     7    8    11   13   15   17   18

      TID = Transaction Id, PID = Protocol Id, LEN = Total length of message, UID = unit Id
      FC = Function Code, RS = Read start address, RQ = Read quantity, WS = Write start address,
      WQ = Write quantity, WC = Write count, WR = Write Register. nn => WQ x 2 bytes
**************************************************************************************************************
*/

void process_query(modbus_pdu_t* mb)
{
    int i,j,retval = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
	uint16_t address,value,count;
    int len = __bswap_16(mb->mbap.length) - 2; // len - fc - unit_id
    uint8_t fc;

    //printf("mbap: txtn_id(%d), proto_id(%d), len(%d), unit_id(%d), fc(%d)\n",
    //		__bswap_16(mb->mbap.transport_id), __bswap_16(mb->mbap.protocol_id),
	//		__bswap_16(mb->mbap.length), mb->mbap.unit_id, mb->fcode);

    for ( i = 0; i < len; i++ )
    {
        fc = mb->fcode;
        switch ( fc )
        {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
        	address = (mb->data[i++] * 256) + mb->data[i++]; // address
        	value   = (mb->data[i++] * 256) + mb->data[i++]; // data
        	retval  = process_handler(address, value);
        	break;

        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
        	address = (mb->data[i++] * 256) + mb->data[i++]; // address
        	count = (mb->data[i++] * 256) + mb->data[i++];   // register count
        	i++;                                             // skip over byte count
        	for ( j = 0; j < count; j++ )
        	{
        	    value   = (mb->data[i++] * 256) + mb->data[i++]; // data
        	    retval  = process_handler(address + j, value);
        	}
        	break;

        case MODBUS_FC_WRITE_AND_READ_REGISTERS:
        	address = (mb->data[i++] * 256) + mb->data[i++]; // address
        	value   = (mb->data[i++] * 256) + mb->data[i++]; // data
        	retval  = process_handler(address, value);
        	address = (mb->data[i++] * 256) + mb->data[i++]; // address
        	count = (mb->data[i++] * 256) + mb->data[i++];   // register count
        	i++;                                             // skip over byte count
        	for ( j = 0; j < count; j++ )
        	{
        	    value   = (mb->data[i++] * 256) + mb->data[i++]; // data
        	    retval  = process_handler(address + j, value);
        	}
        	break;

        default:
        	printf ("default - %d\n", retval);
            retval = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
			break;
        }
    }
    if ( retval == MODBUS_SUCCESS)
    	modbus_reply(ctx, (uint8_t*)mb, sizeof(mbap_header_t) + sizeof(fc) + len, mb_mapping); // subtract function code
    else
       modbus_reply_exception(ctx, (uint8_t*)mb, retval);
}

void update_json_file(float soc, const char* status)
{
    FILE *fp;
    const char* filename = "soc.json";

    fp = fopen ( filename, "w");
    if ( fp )
    {
        //rewind(fp);
        fprintf(fp,
                "{" \
                    "\"stateOfCharge\": {"  \
                        "\"charge\":\"%f\", " \
                        "\"status\":\"%s\", " \
                        "\"time\":\"%08ld\" " \
                    "}" \
                "}\n", soc, status, time(NULL));
        //fputs(s, fp);
        fclose(fp);
    }
}

//
// Thread handler
//
void *handler( void *ptr )
{
    char *terminate;
    char status[12] = "idle";
    printf("entering thread handler\n");

    thread_param_t* param = (thread_param_t*) ptr;
    ctx = param->ctx;
    mb_mapping = param->mb_mapping;
    terminate = param->terminate;
    free(param);

    while ( *terminate == false )
    {
        sleep(1);
        printf("heartbeat timer %d seconds, heartbeatTimeout period %d\n", heartbeat, heartbeatTimeout );
        if ( heartbeat > heartbeatTimeout )
        {
            printf("heartbeat not received in heartbeatTimeout configured period = %d\n", heartbeatTimeout);
            heartbeat = 0;
        }

		if (battery_charging)
		{
			strcpy(status,"charging");
			if ( (state_of_charge + battery_charge_increment) <= battery_fully_charged )
			{
				state_of_charge += battery_charge_increment;
			}
			else
			{
				state_of_charge = battery_fully_charged;
				battery_charging = false;
			}
		}
		else if (battery_discharging)
		{
			strcpy(status,"discharging");
			if ( (state_of_charge - battery_discharge_decrement) >= battery_fully_discharged )
			{
				state_of_charge -= battery_discharge_decrement;
			}
			else
			{
				state_of_charge = battery_fully_discharged;
				battery_discharging = false;
			}
		}
		else
		{
			strcpy(status,"idle");
		}
        update_json_file(state_of_charge, (const char*)status);
        heartbeat++;
    }
    printf("exiting thread handler\n");
}

