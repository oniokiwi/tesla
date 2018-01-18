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

#include "main.h"
#include "typedefs.h"

static char query[MODBUS_TCP_MAX_ADU_LENGTH];

#define MODBUS_DEFAULT_PORT 1502


static void usage(const char *app_name)
{
    printf("Usage:\n");
    printf("%s [option <value>] ...\n", app_name);
    printf("\nOptions:\n");
    printf(" -p \t\t # Set Modbus port to listen on for incoming requests (Default 502)\n");
    printf(" -w \t\t # These are 16 bits Read Write registers (Default %d)\n", UT_REGISTERS_NB);
    printf(" -? \t\t # Print this help menu\n");
    printf("\nExamples:\n");
    printf("%s -p 1502  \t # Change the listen port to 1502\n", app_name);
    printf("%s -w %d \t # Create %d holding register starting at zero.\n", app_name, UT_REGISTERS_NB,UT_REGISTERS_NB);
    exit(1);
}

int main(int argc, char*argv[])
{
    extern void *handler( void *ptr );
    modbus_t *ctx;
    int rc, opt, s = -1, port = MODBUS_DEFAULT_PORT;
    uint16_t holding_register = UT_REGISTERS_NB;
    pthread_t thread1;
    char terminate;
    modbus_mapping_t *mb_mapping;
    thread_param_t* thread_param;
    bool done = FALSE;
    bool initialised = FALSE;

    setvbuf(stdout, NULL, _IONBF, 0);                          // disable stdout buffering

    while ((opt = getopt(argc, argv, "p:w:")) != -1)
    {
        switch (opt) {
        case 'p':
            port = atoi(optarg);
            break;

        case 'w':
            holding_register = atoi(optarg);
            break;

        default:
            usage(*argv);
        }
    }
    printf("Tesla battery simulator...\n");
    printf("port:%d HoldingRegisters:%d\n", port, holding_register );

    for (;;)
    {
        ctx = modbus_new_tcp(NULL, port);
        if ( initialised == FALSE )
        {
            mb_mapping = modbus_mapping_new_start_address(
               0, 0,
               0, 0,
               0, holding_register,
               0, 0);

            if (mb_mapping == NULL)
            {
                printf("Failed to allocate the mapping: %s\n", modbus_strerror(errno));
                modbus_free(ctx);
                return -1;
            }
            initialised = TRUE;
        }
        thread_param = malloc(sizeof (thread_param_t));
        terminate = FALSE;
        thread_param -> ctx = ctx;
        thread_param -> mb_mapping = mb_mapping;
        thread_param -> terminate = &terminate;
        pthread_create( &thread1, NULL, handler, thread_param);

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
                modbus_close(ctx);
                modbus_free(ctx);
                terminate = true;
                pthread_join( thread1, NULL);
                done = TRUE;
                break;

            case 0:
                // No data received
                break;

            default:
                process_query((modbus_pdu_t*)query);
                continue;
            }
        }
    } // for (;;)
    modbus_mapping_free(mb_mapping);     // out of the loop to maintain register values

    return 0;
}


