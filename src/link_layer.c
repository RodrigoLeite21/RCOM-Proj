// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define F  0x7E
#define A  0x03
#define C_SET 0x03
#define C_UA  0x07
#define BCC(a, c) ((a) ^ (c))

#define BUF_SIZE 256

// State Machine
enum states {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOPPED,
};

// Global Variables
volatile int STOP = 0;
int alarmEnabled = 0;
int alarmCount = 0;

unsigned char setFrame[5] = {F, A, C_SET, BCC(A, C_SET), F};
unsigned char uaFrame[5]  = {F, A, C_UA,  BCC(A, C_UA),  F};

void alarmHandler(int signal)
{
    alarmEnabled = 0;
    alarmCount++;
    printf("Alarm #%d: received\n", alarmCount);
    writeBytesSerialPort(setFrame, 5);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    STOP = 0;
    alarmEnabled = 0;
    alarmCount = 0;

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", connectionParameters.serialPort);

    if (connectionParameters.role == LlTx)
    {
        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;

        if (sigaction(SIGALRM, &act, NULL) == -1)
        {
            perror("sigaction");
            return -1;
        }

        writeBytesSerialPort(setFrame, 5);
        printf("SET frame sent\n");

        enum states state = START;
        unsigned char byte, a = 0, c = 0;

        while (STOP == 0 && alarmCount < connectionParameters.nRetransmissions)
        {
            if (!alarmEnabled)
            {
                alarm(connectionParameters.timeout);
                alarmEnabled = 1;
            }

            int res = readByteSerialPort(&byte);
            if (res > 0)
            {
                switch (state)
                {
                    case START:
                        if (byte == F) state = FLAG_RCV;
                        break;

                    case FLAG_RCV:
                        if (byte == A) { state = A_RCV; a = byte; }
                        else if (byte != F) state = START;
                        break;

                    case A_RCV:
                        if (byte == C_UA) { state = C_RCV; c = byte; }
                        else if (byte == F) state = FLAG_RCV;
                        else state = START;
                        break;

                    case C_RCV:
                        if (byte == (a ^ c)) state = BCC_OK;
                        else if (byte == F) state = FLAG_RCV;
                        else state = START;
                        break;

                    case BCC_OK:
                        if (byte == F)
                        {
                            state = STOPPED;
                            STOP = 1;
                            alarm(0);
                            printf("UA received, connection established\n");
                        }
                        else state = START;
                        break;
                    case STOPPED:
                        break;
                }
            }
        }

        if (!STOP)
        {
            printf("Connection failed after %d tries\n", alarmCount);
            return -1;
        }
    }
    else if (connectionParameters.role == LlRx)
    {
        enum states state = START;
        unsigned char byte, a = 0, c = 0;

        while (STOP == 0)
        {
            if (readByteSerialPort(&byte) > 0)
            {
                switch (state)
                {
                    case START:
                        if (byte == F) state = FLAG_RCV;
                        break;

                    case FLAG_RCV:
                        if (byte == A) { state = A_RCV; a = byte; }
                        else if (byte != F) state = START;
                        break;

                    case A_RCV:
                        if (byte == C_SET) { state = C_RCV; c = byte; }
                        else if (byte == F) state = FLAG_RCV;
                        else state = START;
                        break;

                    case C_RCV:
                        if (byte == (a ^ c)) state = BCC_OK;
                        else if (byte == F) state = FLAG_RCV;
                        else state = START;
                        break;

                    case BCC_OK:
                        if (byte == F)
                        {
                            state = STOPPED;
                            STOP = 1;
                            printf("SET received -> sending UA\n");
                        }
                        else state = START;
                        break;
                    
                    case STOPPED:
                        break;
                }
            }
        }

        writeBytesSerialPort(uaFrame, 5);
        printf("UA sent\n");
    }
    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    printf("Closing connection...\n");
    closeSerialPort();
    printf("Serial port closed\n");
    return 0;
}
