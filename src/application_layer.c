// application_layer.c

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DATA_PACKET 1
#define START_PACKET 2
#define END_PACKET 3


static int build_control_packet(unsigned char *packet, int controlField, const char *filename, long fileSize)
{
    int index = 0;

    packet[index++] = controlField; // C field

    // Type = 0, Length = sizeof(fileSize), Value = fileSize
    packet[index++] = 0; // T1
    packet[index++] = sizeof(long);
    memcpy(&packet[index], &fileSize, sizeof(long));
    index += sizeof(long);

    // Type = 1, Length = strlen(filename), Value = filename
    packet[index++] = 1;
    unsigned char filenameLength = strlen(filename);
    packet[index++] = filenameLength;
    memcpy(&packet[index], filename, filenameLength);
    index += filenameLength;

    return index;
}

static long getFileSize(FILE *file)
{
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    rewind(file);
    return size;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort, serialPort);
    linkLayer.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;

    printf("\n--- Opening link ---\n");
    if (llopen(linkLayer) == -1)
    {
        fprintf(stderr, "Error: llopen failed\n");
        return;
    }

    if (linkLayer.role == LlTx)
    {
        FILE *file = fopen(filename, "rb");
        if (!file)
        {
            perror("Error opening file");
            llclose(1);
            return;
        }

        long fileSize = getFileSize(file);
        unsigned char packet[512];

        int packetSize = build_control_packet(packet, START_PACKET, filename, fileSize);
        llwrite(packet, packetSize);

        unsigned char dataBuffer[256];
        int seq = 0;
        int bytesRead;

        while ((bytesRead = fread(dataBuffer, 1, sizeof(dataBuffer), file)) > 0)
        {
            unsigned char dataPacket[260];
            int index = 0;

            dataPacket[index++] = DATA_PACKET;
            dataPacket[index++] = seq % 256;
            dataPacket[index++] = bytesRead / 256;
            dataPacket[index++] = bytesRead % 256;

            memcpy(&dataPacket[index], dataBuffer, bytesRead);
            index += bytesRead;

            if (llwrite(dataPacket, index) == -1)
            {
                fprintf(stderr, "Error: Failed to send data packet\n");
                break;
            }

            seq++;
        }

        packetSize = build_control_packet(packet, END_PACKET, filename, fileSize);
        llwrite(packet, packetSize);

        fclose(file);
        printf("File '%s' sent successfully (%ld bytes)\n", filename, fileSize);
    }
    else
    {
        FILE *file = fopen(filename, "wb");
        if (!file)
        {
            perror("Error creating file");
            llclose(0);
            return;
        }

        unsigned char packet[512];
        int packetSize;
        int receiving = 1;

        while (receiving)
        {
            packetSize = llread(packet);

            if (packetSize == -1)
                continue;

            unsigned char control = packet[0];

            if (control == START_PACKET)
            {
                printf("Start packet received.\n");
            }
            else if (control == DATA_PACKET)
            {
                int dataSize = packet[2] * 256 + packet[3];
                fwrite(&packet[4], 1, dataSize, file);
            }
            else if (control == END_PACKET)
            {
                printf("End packet received.\n");
                receiving = 0;
            }
        }

        fclose(file);
        printf("File '%s' received successfully.\n", filename);
    }

    printf("--- Closing link ---\n");
    if (llclose(linkLayer.role) == -1)
        fprintf(stderr, "Error: llclose failed\n");
    else
        printf("Link closed successfully.\n");
}