/*
 * Copyright © 2008-2014 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <getopt.h>
# include <sys/socket.h>
#include "tesla.h"
#include <pthread.h>

/* For MinGW */
#ifndef MSG_NOSIGNAL
# define MSG_NOSIGNAL 0
#endif

#include "main.h"
#include "typedefs.h"

#define MODBUS_DEFAULT_PORT 1502

enum {
    TCP,
    TCP_PI,
    RTU
};

enum OPTARGS_ERROR
{
    OPTARG_SUCCESS = 0,
    OPTARG_BAD_NUMBER_OUTPUT_DISCRETE_COILS = -100,
    OPTARG_BAD_NUMBER_INPUT_DISCRETE_COILS,
    OPTARG_BAD_NUMBER_INPUT_REGISTER,
    OPTARG_BAD_NUMBER_OUTPUT_REGISTER,
};

const char* error_name(int error_id)
{
    switch ( error_id )
    {
    case OPTARG_SUCCESS:
        return "Success";

    case OPTARG_BAD_NUMBER_OUTPUT_DISCRETE_COILS:
        return "Bad number of output discrete coils given";

    case OPTARG_BAD_NUMBER_INPUT_DISCRETE_COILS:
        return "Bad number of input discrete coils given";

    case OPTARG_BAD_NUMBER_INPUT_REGISTER:
        return "Bad number of input registers given";

    case OPTARG_BAD_NUMBER_OUTPUT_REGISTER:
        return "Bad number of output registers given";

    default:
         /* intenitonally left blank" */
        break;
    }
    return NULL;
}

int usage(int argc, char** argv)
{
    printf("Usage:\n");
    printf("%s [option <value>] [option <value:number-bits>] [option <value:number_registers>] ...\n", argv[0]);
    printf("\nOptions:\n");
    printf(" -p | --port\t\t\t\t # Set Modbus port to listen on for incoming requests (Default 502)\n");
    printf(" -o | --DiscreteOutputCoils\t\t # Discrete output coils this is usually 1 bit Read/Write (Default %d:%d)\n",UT_BITS_ADDRESS,UT_BITS_NB);
    printf(" -i | --DiscreteInputContacts\t\t # Discrete input contacts this is 1 bit Read only bit (Default %d:%d)\n",UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB);
    printf(" -r | --AnalogInputRegisters\t\t # These are 16-bits Read Only registers (Default %d:%d)\n",UT_INPUT_REGISTERS_ADDRESS, UT_INPUT_REGISTERS_NB);
    printf(" -w | --AnalogOutputHoldingRegisters\t # These are 16 bits Read Write registers (Default %d:%d)\n",UT_REGISTERS_ADDRESS, UT_REGISTERS_NB);
    printf(" -h | --help\t\t\t\t # Print this help menu\n");
    printf("\nExamples:\n");
    printf("%s -p 1502 | --port 1502\t\t\t\t     # Change the listen port to 1502\n", argv[0]);
    printf("%s -w 14012:63 | --AnalogOutputHoldingRegisters 14012:63 # Create 63 read write register starting from address 14012.\n", argv[0]);
    exit(1);
}

int get_optarguments(int argc, char*argv[], optargs_t* args)
{
    int c;
    int retval = OPTARG_SUCCESS;

    while (1)
    {
        int this_option_optind = optind ? optind : 1;
        int option_index = 0;
        static struct option long_options[] =
        {
            {"port",                         required_argument, 0, 'p' },
            {"DiscreteOutputCoils",          required_argument, 0, 'o' },
            {"DiscreteInputContacts",        required_argument, 0, 'i' },
            {"AnalogInputRegisters",         required_argument, 0, 'r' },
            {"AnalogOutputHoldingRegisters", required_argument, 0, 'w' },
            {"help",                         required_argument, 0, 'h' },
            {0, 0, 0, 0 }
        };

        c = getopt_long(argc, argv, "p:o:i:r:w:h?", long_options, &option_index);
        if ((c == -1) || retval < 0)
            break;

        switch (c)
        {
            case 'p':
                args->port = atoi(optarg);
                break;

            case 'o':
                if (strchr(optarg,':') == NULL)
                {
                    retval =  OPTARG_BAD_NUMBER_OUTPUT_DISCRETE_COILS;
                    continue;
                }
                args -> DiscreteOutputCoils = atoi(optarg);
                args -> NumberDiscreteOutputCoils = atoi(strchr(optarg,':')+1);
                break;

            case 'i':
                if (strchr(optarg,':') == NULL)
                {
                    retval = OPTARG_BAD_NUMBER_INPUT_DISCRETE_COILS;
                    continue;
                }
                args -> DiscreteInputContacts = atoi(optarg);
                args -> NumberDiscreteInputContacts = atoi(strchr(optarg,':')+1);
                break;

            case 'r':
                if (strchr(optarg,':') == NULL)
                {
                    retval =  OPTARG_BAD_NUMBER_INPUT_REGISTER;
                    continue;
                }
                args -> AnalogInputRegisters = atoi(optarg);
                args -> NumberAnalogInputRegisters = atoi(strchr(optarg,':')+1);
                break;

            case 'w':
                if (strchr(optarg,':') == NULL)
                {
                    retval =  OPTARG_BAD_NUMBER_OUTPUT_REGISTER;
                    continue;
                }
                args -> AnalogOutputHoldingRegisters = atoi(optarg);
                args -> NumberAnalogOutputHoldingRegisters = atoi(strchr(optarg,':')+1);
                break;

            case 'h':
                usage(argc, argv);
                break;

            case '?':
                break;

            default:
                printf("?? getopt returned character code 0%o ??\n", c);
        }
    }
    return retval;
}


pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char*argv[])
{
	uint8_t *query;
    extern void *handler( void *ptr );
    modbus_t *ctx;
    int s = -1;
    int rc;
    pthread_t thread1;
    char terminate;
    modbus_mapping_t *mb_mapping;
    thread_param_t* thread_param;
    bool initialised = FALSE;
    bool done = FALSE;
    uint32_t tv_sec = 60;
    uint32_t tv_usec = 0;
    optargs_t args =
    {
        MODBUS_DEFAULT_PORT,
        UT_BITS_ADDRESS,UT_BITS_NB,
        UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB,
        UT_INPUT_REGISTERS_ADDRESS, UT_INPUT_REGISTERS_NB,
        UT_REGISTERS_ADDRESS, UT_REGISTERS_NB
    };


    setvbuf(stdout, NULL, _IONBF, 0);                          // disable stdout buffering
    rc = get_optarguments(argc, argv, &args);
    if ( rc < OPTARG_SUCCESS )
    {
        usage(argc, argv);
    }
    printf("mb2http arguments...\n");
    printf("port:%d DiscreteOutputCoils:%d(%d) DiscreteInputContacts:%d(%d) " \
           "AnalogInputRegisters:%d(%d) AnalogOutputHoldingRegisters:%d(%d)\n",
                    args.port,
                    args.DiscreteOutputCoils,          args.NumberDiscreteOutputCoils,
                    args.DiscreteInputContacts,        args.NumberDiscreteInputContacts,
                    args.AnalogInputRegisters,         args.NumberAnalogInputRegisters,
                    args.AnalogOutputHoldingRegisters, args.NumberAnalogOutputHoldingRegisters );

    ctx = modbus_new_tcp(NULL, args.port);
    modbus_set_response_timeout(ctx, tv_sec,tv_usec);
    modbus_set_debug(ctx, TRUE);

	mb_mapping = modbus_mapping_new_start_address(
		args.DiscreteOutputCoils, args.NumberDiscreteOutputCoils,
		args.DiscreteInputContacts, args.NumberDiscreteInputContacts,
		args.AnalogOutputHoldingRegisters, args.NumberAnalogOutputHoldingRegisters,
		args.AnalogInputRegisters, args.NumberAnalogInputRegisters);

	if (mb_mapping == NULL)
	{
		printf("Failed to allocate the mapping: %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		return -1;
	}
	thread_param = malloc(sizeof (thread_param_t));
	terminate = FALSE;
	thread_param -> ctx = ctx;
	thread_param -> mb_mapping = mb_mapping;
	thread_param -> mutex = mutex1;
	thread_param -> terminate = &terminate;
	pthread_create( &thread1, NULL, handler, thread_param);

    query = malloc(MODBUS_TCP_MAX_ADU_LENGTH);

    for (;;)
    {
        s = modbus_tcp_listen(ctx, 1);
        modbus_tcp_accept(ctx, &s);
        done = FALSE;
        while (!done)
        {
        	rc = modbus_receive(ctx, query);
        	switch (rc)
        	{
        	case -1:
        		close(s); // close the socket
        		done = TRUE;
        		break;

        	default:
       		    process_query((modbus_pdu_t*)query);
        		continue;
        	}
        }
    } // for (;;)
    modbus_close(ctx);
    modbus_free(ctx);
    free(query);
    modbus_mapping_free(mb_mapping);     // out of the loop to maintain register values

    return 0;
}


