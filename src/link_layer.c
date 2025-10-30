// Link Layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <stdint.h>

#define _POSIX_SOURCE 1

// Basic frame constants
#define FLAG 0x7E
#define A_TX 0x03
#define A_RX 0x01

#define C_SET 0x03
#define C_UA  0x07
#define C_DISC 0x0B

// I-frame control (Ns bit in bit 7)
#define C_I_NS0 0x00
#define C_I_NS1 0x80

// RR / REJ control values
#define C_RR0  0x05
#define C_RR1  0x85
#define C_REJ0 0x01
#define C_REJ1 0x81

#define ESC 0x7D
#define ESC_XOR 0x20

#ifndef MAX_PAYLOAD_SIZE
#define MAX_PAYLOAD_SIZE 1024
#endif
#define MAX_FRAME_SIZE (2*MAX_PAYLOAD_SIZE + 64)

#define FLUSH_LIMIT 10000

// Globals
volatile int STOP = 0;
static volatile sig_atomic_t alarm_fired = 0;

static int g_role = 0;
static int g_timeout = 0;
static int g_nretrans = 0;
static uint8_t g_tx_ns = 0;      
static uint8_t g_rx_expected = 0;
static int g_duplicate_count = 0;

// Alarm handler used for retransmissions
static void alarm_handler(int signo) {
    (void)signo;
    alarm_fired = 1;
}

// Compute BCC1 = A ^ C
static uint8_t bcc1(uint8_t A, uint8_t C) {
    return (uint8_t)(A ^ C);
}

// Compute BCC2 
static uint8_t bcc2(const uint8_t *buf, int len) {
    uint8_t x = 0x00;
    for (int i = 0; i < len; ++i) x ^= buf[i];
    return x;
}

static int stuff(const unsigned char *in, int inlen, unsigned char *out, int outcap) {
    int p = 0;
    for (int i = 0; i < inlen; ++i) {
        unsigned char c = in[i];
        if (c == FLAG || c == ESC) {
            if (p + 2 > outcap) return -1;
            out[p++] = ESC;
            out[p++] = c ^ ESC_XOR;
        } else {
            if (p + 1 > outcap) return -1;
            out[p++] = c;
        }
    }
    return p;
}

static int destuff(const unsigned char *in, int inlen, unsigned char *out, int outcap) {
    int p = 0;
    for (int i = 0; i < inlen; ++i) {
        unsigned char c = in[i];
        if (c == ESC) {
            if (i + 1 >= inlen) return -1;
            unsigned char r = in[++i] ^ ESC_XOR;
            if (p + 1 > outcap) return -1;
            out[p++] = r;
        } else {
            if (p + 1 > outcap) return -1;
            out[p++] = c;
        }
    }
    return p;
}

// Write a supervision frame: FLAG A C BCC FLAG
static int send_su(uint8_t A_field, uint8_t C_field) {
    unsigned char f[5];
    f[0] = FLAG; f[1] = A_field; f[2] = C_field; f[3] = bcc1(A_field, C_field); f[4] = FLAG;
    int w = writeBytesSerialPort(f, 5);
    return (w == 5) ? 0 : -1;
}

// Read a supervision frame (blocking)
static int read_su(uint8_t expectedA, uint8_t *Cout) {
    enum { ST_START, ST_FLAG, ST_A, ST_C, ST_BCC } st = ST_START;
    unsigned char b;
    unsigned char A = 0, C = 0, B = 0;
    while (1) {
        int r = readByteSerialPort(&b);
        if (r < 0) return -1;
        if (r == 0) continue;
        switch (st) {
            case ST_START:
                if (b == FLAG) st = ST_FLAG;
                break;
            case ST_FLAG:
                if (b == expectedA) { A = b; st = ST_A; }
                else if (b != FLAG) st = ST_START;
                break;
            case ST_A:
                C = b; st = ST_C; break;
            case ST_C:
                B = b; st = ST_BCC; break;
            case ST_BCC:
                if (b == FLAG) {
                    if (B == (uint8_t)(A ^ C)) {
                        if (Cout) *Cout = C;
                        return 0;
                    } else {
                        return -1;
                    }
                } else {
                    st = ST_START;
                }
                break;
        }
    }
    return -1;
}

static void flush_until_flag() {
    unsigned char b;
    int flushCount = 0;
    do {
        int r = readByteSerialPort(&b);
        if (r <= 0) break;
        flushCount++;
    } while (b != FLAG && flushCount < FLUSH_LIMIT);
}

// Read an I-frame:
static int read_iframe(uint8_t expectedA, uint8_t *Cout, unsigned char *out, int outcap) {
    unsigned char b;
    while (1) {
        int r = readByteSerialPort(&b);
        if (r < 0) return -1;
        if (r == 0) continue;
        if (b == FLAG) break;
    }

    unsigned char body[MAX_FRAME_SIZE];
    int blen = 0;
    while (1) {
        int r = readByteSerialPort(&b);
        if (r < 0) { flush_until_flag(); return -1; }
        if (r == 0) continue;
        if (b == FLAG) break;
        if (blen >= (int)sizeof(body)) { flush_until_flag(); return -1; }
        body[blen++] = b;
    }

    if (blen < 4) { flush_until_flag(); return -1; }
    unsigned char A = body[0];
    unsigned char C = body[1];
    unsigned char B1 = body[2];
    if (A != expectedA) { flush_until_flag(); return -1; }
    if (B1 != (unsigned char)(A ^ C)) { flush_until_flag(); return -1; }

    int stuffed_len = blen - 3;
    if (stuffed_len < 1) {
        if (Cout) *Cout = C;
        return 0;
    }
    unsigned char destuffed[MAX_FRAME_SIZE];
    int dlen = destuff(body + 3, stuffed_len, destuffed, sizeof(destuffed));
    if (dlen < 1) { flush_until_flag(); return -1; }
    int payload_len = dlen - 1;
    unsigned char recv_bcc2 = destuffed[payload_len];
    unsigned char calc_bcc2 = bcc2(destuffed, payload_len);
    if (calc_bcc2 != recv_bcc2) { flush_until_flag(); return -1; }
    if (payload_len > outcap) { flush_until_flag(); return -1; }
    if (payload_len > 0) memcpy(out, destuffed, payload_len);
    if (Cout) *Cout = C;
    return payload_len;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    STOP = 0;
    alarm_fired = 0;

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened:\n", connectionParameters.serialPort);

    g_role = connectionParameters.role;
    g_timeout = connectionParameters.timeout;
    g_nretrans = connectionParameters.nRetransmissions;
    g_tx_ns = 0;
    g_rx_expected = 0;
    g_duplicate_count = 0;

    struct sigaction act = {0};
    act.sa_handler = alarm_handler;
    sigemptyset(&act.sa_mask);
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        perror("sigaction");
        closeSerialPort();
        return -1;
    }

    if (g_role == LlTx) {
        printf("Sending SET...\n");
        unsigned char set[5] = {FLAG, A_TX, C_SET, bcc1(A_TX, C_SET), FLAG};

        int tries = 0;
        while (tries < g_nretrans) {
            alarm_fired = 0;
            if (writeBytesSerialPort(set, 5) != 5) {
                closeSerialPort(); return -1;
            }
            alarm(g_timeout);

            unsigned char rC = 0;
            int res = read_su(A_TX, &rC);
            if (res == 0) {
                if (rC == C_UA) {
                    alarm(0);
                    alarm_fired = 0;
                    printf("UA received.\nLink opened successfully.\n\n");
                    g_tx_ns = 0;
                    g_rx_expected = 0;
                    return 0;
                } else {
                    continue;
                }
            } else {
                if (alarm_fired) {
                    tries++;
                    alarm(0);
                    alarm_fired = 0;
                    printf("Timeout, retransmitting SET (try %d)...\n", tries);
                    continue;
                }
                tries++;
                continue;
            }
        }
        closeSerialPort();
        return -1;
    } else {
        while (1) {
            unsigned char rC = 0;
            int res = read_su(A_TX, &rC);
            if (res < 0) continue;
            if (rC == C_SET) {
                printf("SET received.\nSending UA...\n");
                if (send_su(A_TX, C_UA) < 0) {
                    closeSerialPort(); return -1;
                }
                printf("Link opened successfully.\n\n");
                g_tx_ns = 0;
                g_rx_expected = 0;
                return 0;
            }
        }
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    if (!buf || bufSize < 0 || bufSize > MAX_PAYLOAD_SIZE) return -1;

    unsigned char frame[MAX_FRAME_SIZE];
    int pos = 0;
    unsigned char C = (g_tx_ns == 0) ? C_I_NS0 : C_I_NS1;
    frame[pos++] = FLAG;
    frame[pos++] = A_TX;
    frame[pos++] = C;
    frame[pos++] = bcc1(A_TX, C);

    unsigned char payload_with_bcc[MAX_PAYLOAD_SIZE + 1];
    memcpy(payload_with_bcc, buf, bufSize);
    payload_with_bcc[bufSize] = bcc2(buf, bufSize);
    int pwlen = bufSize + 1;

    unsigned char stuffed[MAX_FRAME_SIZE];
    int stuffed_len = stuff(payload_with_bcc, pwlen, stuffed, sizeof(stuffed));
    if (stuffed_len < 0) return -1;
    if (pos + stuffed_len >= (int)sizeof(frame) - 2) return -1;
    memcpy(frame + pos, stuffed, stuffed_len);
    pos += stuffed_len;

    frame[pos++] = FLAG;

    struct sigaction act = {0};
    act.sa_handler = alarm_handler;
    sigemptyset(&act.sa_mask);
    if (sigaction(SIGALRM, &act, NULL) == -1) return -1;

    int attempts = 0;

    while (attempts < g_nretrans) {
        alarm_fired = 0;
        if (writeBytesSerialPort(frame, pos) != pos) return -1;
        alarm(g_timeout);

        unsigned char rc = 0;
        int r = read_su(A_RX, &rc);
        if (r == 0) {
            if (rc == C_RR0 || rc == C_RR1) {
                unsigned char nr = (rc == C_RR1) ? 1 : 0;
                if (nr == (uint8_t)(g_tx_ns ^ 1)) {
                    alarm(0);
                    alarm_fired = 0;
                    g_tx_ns ^= 1;
                    return bufSize;
                } else {
                    continue;
                }
            } else if (rc == C_REJ0 || rc == C_REJ1) {
                attempts++;
                alarm(0);
                alarm_fired = 0;
                printf("Received REJ -> retransmitting (attempt %d)\n", attempts);
                continue;
            } else if (rc == C_DISC) {
                alarm(0);
                alarm_fired = 0;
                return -1;
            } else {
                continue;
            }
        } else {
            if (alarm_fired) {
                attempts++;
                alarm(0);
                alarm_fired = 0;
                printf("Timeout while waiting for RR/REJ -> retransmit (attempt %d)\n", attempts);
                continue;
            }
            attempts++;
            continue;
        }
    }

    alarm(0); alarm_fired = 0;
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    if (!packet) return -1;
    int rejCount = 0;
    while (1) {
        unsigned char C = 0;
        unsigned char local[MAX_PAYLOAD_SIZE + 16];
        int n = read_iframe(A_TX, &C, local, MAX_PAYLOAD_SIZE);
        if (n < 0) {
            unsigned char rej = (g_rx_expected == 0) ? C_REJ0 : C_REJ1;
            send_su(A_RX, rej);
            printf("REJ sent (expected seq: %d)\n", g_rx_expected);
            rejCount++;
            if (rejCount > 10) {
                fprintf(stderr, "Too many REJ sent, aborting connection...\n");
                return -1;
            }
            continue;
        }
        rejCount = 0;

        unsigned char ns = (C & 0x80) ? 1 : 0;

        if (ns == g_rx_expected) {
            unsigned char rr = (g_rx_expected ^ 1) ? C_RR1 : C_RR0;
            if (send_su(A_RX, rr) < 0) return -1;
            g_rx_expected ^= 1;
            if (n > 0) memcpy(packet, local, n);
            printf("Frame accepted (seq: %d, size: %d bytes)\n", ns, n);
            return n;
        } else {
            g_duplicate_count++;
            printf("Duplicated Frame Detected! Received seq:%d but expected seq:%d\n...", 
                   g_duplicate_count, ns, g_rx_expected);
            printf("Discarding duplicate and resending RR%d.\n", g_rx_expected);
            
            unsigned char rr = (g_rx_expected) ? C_RR1 : C_RR0;
            if (send_su(A_RX, rr) < 0) return -1;
            continue;
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose() {
    struct sigaction act = {0};
    act.sa_handler = alarm_handler;
    sigemptyset(&act.sa_mask);
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        closeSerialPort();
        return -1;
    }

    if (g_role == LlTx) {
        unsigned char disc[5] = {FLAG, A_TX, C_DISC, bcc1(A_TX, C_DISC), FLAG};
        int attempts = 0;
        printf("Sending DISC...\n");
        while (attempts < g_nretrans) {
            alarm_fired = 0;
            if (writeBytesSerialPort(disc, 5) != 5) { 
                fprintf(stderr, "Failed to send DISC\n");
                closeSerialPort(); 
                return -1; 
            }
            alarm(g_timeout);

            unsigned char rc = 0;
            int r = read_su(A_RX, &rc);
            if (r == 0 && rc == C_DISC) {
                printf("DISC received.\n");
                unsigned char ua[5] = {FLAG, A_RX, C_UA, bcc1(A_RX, C_UA), FLAG};
                if (writeBytesSerialPort(ua, 5) != 5) { 
                    fprintf(stderr, "Failed to send UA\n");
                    closeSerialPort(); 
                    return -1; 
                }
                printf("Sending UA...\n\n");
                closeSerialPort();
                printf("Serial port closed.\n");
                return 0;
            }
            if (alarm_fired) {
                attempts++;
                alarm(0);
                alarm_fired = 0;
                printf("Timeout waiting for DISC, retrying (%d)...\n", attempts);
                continue;
            }
            if (r < 0) {
                attempts++;
                printf("Error waiting for DISC, retrying (%d)...\n", attempts);
                continue;
            }
        }
        fprintf(stderr, "Max DISC retries reached; closing anyway\n");
        closeSerialPort();
        printf("Serial port closed.\n");
        return -1;
    } else {
        while (1) {
            unsigned char rc = 0;
            int r = read_su(A_TX, &rc);
            if (r < 0) continue;
            if (rc == C_DISC) {
                printf("DISC received.\n");
                break;
            }
        }
        unsigned char disc_rx[5] = {FLAG, A_RX, C_DISC, bcc1(A_RX, C_DISC), FLAG};
        if (writeBytesSerialPort(disc_rx, 5) != 5) { 
            fprintf(stderr, "Failed to send DISC\n");
            closeSerialPort(); 
            return -1; 
        }
        printf("Sending DISC...\n");

        while (1) {
            unsigned char rc = 0;
            int r = read_su(A_RX, &rc);
            if (r < 0) continue;
            if (rc == C_UA) {
                printf("UA received.\n");
                break;
            }
        }
        closeSerialPort();
        printf("Serial port closed.\n");
        return 0;
    }
}