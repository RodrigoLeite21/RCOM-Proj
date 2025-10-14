// application_layer.c

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <arpa/inet.h>

#define DATA_PACKET 1
#define START_PACKET 2
#define END_PACKET 3

#define MAX_FILENAME_LEN 255

static int build_control_packet(unsigned char *packet, int controlField, const char *filename, uint32_t fileSize)
{
    int index = 0;

    packet[index++] = (unsigned char)controlField;

    packet[index++] = 0;
    packet[index++] = 4; 
    uint32_t be_size = htonl(fileSize);
    memcpy(&packet[index], &be_size, 4);
    index += 4;

    packet[index++] = 1;
    uint8_t filenameLength = (uint8_t)strlen(filename);
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
    strncpy(linkLayer.serialPort, serialPort, sizeof(linkLayer.serialPort)-1);
    linkLayer.serialPort[sizeof(linkLayer.serialPort)-1] = '\0';
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
            llclose();
            return;
        }

        long rawSize = getFileSize(file);
        if (rawSize < 0) {
            perror("ftell");
            fclose(file);
            llclose();
            return;
        }
        if (rawSize > UINT32_MAX) {
            fprintf(stderr, "File too large (>4GB).\n");
            fclose(file);
            llclose();
            return;
        }
        uint32_t fileSize = (uint32_t)rawSize;

        if (strlen(filename) > MAX_FILENAME_LEN) {
            fprintf(stderr, "Filename too long (max %d chars).\n", MAX_FILENAME_LEN);
            fclose(file);
            llclose();
            return;
        }

        unsigned char packet[1024];

        int packetSize = build_control_packet(packet, START_PACKET, filename, fileSize);
        if (llwrite(packet, packetSize) < 0) {
            fprintf(stderr, "Error: Failed to send START packet\n");
            fclose(file);
            llclose();
            return;
        }

        unsigned char dataBuffer[512];
        int seq = 0;
        size_t bytesRead;

        while ((bytesRead = fread(dataBuffer, 1, sizeof(dataBuffer), file)) > 0)
        {
            unsigned char dataPacket[4 + sizeof(dataBuffer)];
            int index = 0;

            dataPacket[index++] = DATA_PACKET;
            dataPacket[index++] = (uint8_t)(seq % 256);
            dataPacket[index++] = (uint8_t)((bytesRead >> 8) & 0xFF);
            dataPacket[index++] = (uint8_t)(bytesRead & 0xFF);

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
        printf("File '%s' sent successfully (%u bytes)\n", filename, fileSize);
    }
    else
    {
        FILE *file = fopen(filename, "wb");
        if (!file)
        {
            perror("Error creating file");
            llclose();
            return;
        }

        unsigned char packet[1024];
        int packetSize;
        int receiving = 1;

        while (receiving)
        {
            packetSize = llread(packet);

            if (packetSize == -1)
                continue;

            if (packetSize == 0)
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
    if (llclose() == -1)
        fprintf(stderr, "Error: llclose failed\n");
    else
        printf("Link closed successfully.\n");
}