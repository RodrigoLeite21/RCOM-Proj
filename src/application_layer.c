// application_layer.c

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connection;
    memset(&connection, 0, sizeof(LinkLayer));

    strncpy(connection.serialPort, serialPort, sizeof(connection.serialPort) - 1);
    connection.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    connection.baudRate = baudRate;
    connection.nRetransmissions = nTries;
    connection.timeout = timeout;

    printf("\n--- Opening link ---\n");

    if (llopen(connection) < 0) {
        fprintf(stderr, "Error: llopen failed\n");
        return;
    }

    printf("Link opened successfully.\n");


    printf("--- Closing link ---\n");
    if (llclose() == 0)
        printf("Link closed successfully.\n");
    else
        printf("Error: llclose failed\n");
}