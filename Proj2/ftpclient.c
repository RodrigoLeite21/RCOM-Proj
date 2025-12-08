#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define CONTROL_PORT 21
#define MAX_BUF 1024

//------------------------------------------------------
// Read a single FTP reply line (one line only)
//------------------------------------------------------
int read_reply(int sock, char *buffer) {
    int n = 0;
    char c;

    while (read(sock, &c, 1) == 1) {
        if (n < MAX_BUF - 1)
            buffer[n++] = c;
        if (c == '\n')
            break;
    }
    buffer[n] = '\0';

    if (n >= 3 &&
        buffer[0] >= '0' && buffer[0] <= '9' &&
        buffer[1] >= '0' && buffer[1] <= '9' &&
        buffer[2] >= '0' && buffer[2] <= '9') {
        return atoi(buffer);
    }
    return 0;
}

//------------------------------------------------------
// Read full FTP reply (handles multiline replies: 220-, 230- ...)
// Returns numeric code and concat all lines into buffer
//------------------------------------------------------
int read_full_reply(int sock, char *buffer) {
    char line[MAX_BUF];
    int code = 0;
    int first = 1;
    char expected[4] = {0};

    buffer[0] = '\0';

    while (1) {
        int n = 0;
        char c;

        while (read(sock, &c, 1) == 1) {
            if (n < MAX_BUF - 1)
                line[n++] = c;
            if (c == '\n')
                break;
        }
        line[n] = '\0';

        if (n == 0)
            break;

        if (strlen(buffer) + strlen(line) < MAX_BUF - 1)
            strcat(buffer, line);

        if (n >= 3 &&
            line[0] >= '0' && line[0] <= '9' &&
            line[1] >= '0' && line[1] <= '9' &&
            line[2] >= '0' && line[2] <= '9') {

            if (first) {
                strncpy(expected, line, 3);
                expected[3] = '\0';
                code = atoi(expected);
                first = 0;

                if (line[3] != '-')
                    break;
            } else {
                if (strncmp(line, expected, 3) == 0 && line[3] == ' ')
                    break;
            }
        }
    }

    return code;
}

//------------------------------------------------------
// Send FTP command + CRLF
//------------------------------------------------------
void send_cmd(int sock, const char *cmd) {
    char buf[512];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    write(sock, buf, strlen(buf));
}

//------------------------------------------------------
// Parse ftp://[user:pass@]host/path
//------------------------------------------------------
void parse_url(char *url, char *user, char *pass, char *host, char *path) {
    if (strncmp(url, "ftp://", 6) != 0) {
        printf("URL must start with ftp://\n");
        exit(1);
    }

    url += 6;

    char *at = strchr(url, '@');
    char *slash = strchr(url, '/');

    if (at && at < slash) {
        sscanf(url, "%[^:]:%[^@]@%[^/]/%s", user, pass, host, path);
    } else {
        strcpy(user, "anonymous");
        strcpy(pass, "anonymous");
        sscanf(url, "%[^/]/%s", host, path);
    }
}

//------------------------------------------------------
// Extract filename from path
//------------------------------------------------------
char *filename_from_path(char *path) {
    char *p = strrchr(path, '/');
    return p ? p + 1 : path;
}

//------------------------------------------------------
// Connect using skeleton code logic
//------------------------------------------------------
int connect_socket(char *ip, int port) {
    int sockfd;
    struct sockaddr_in addr;

    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket()");
        exit(-1);
    }

    if (connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("connect()");
        exit(-1);
    }

    return sockfd;
}

//------------------------------------------------------
// MAIN
//------------------------------------------------------
int main(int argc, char *argv[]) {

    if (argc != 2) {
        printf("Usage: %s ftp://[user:pass@]host/path\n", argv[0]);
        return -1;
    }

    char user[64], pass[64], host[128], path[256];
    parse_url(argv[1], user, pass, host, path);

    printf("User: %s\nPass: %s\nHost: %s\nPath: %s\n\n",
           user, pass, host, path);

    //--------------------------------------------------
    // 1 — Resolve hostname
    //--------------------------------------------------
    struct hostent *h;
    if ((h = gethostbyname(host)) == NULL) {
        herror("gethostbyname()");
        return -1;
    }

    char *ip = inet_ntoa(*((struct in_addr *) h->h_addr));
    printf("Resolved IP: %s\n", ip);

    //--------------------------------------------------
    // 2 — Connect to FTP control socket (TCP 21)
    //--------------------------------------------------
    int ctrl = connect_socket(ip, CONTROL_PORT);

    char reply[MAX_BUF];

    //--------------------------------------------------
    // 2.1 — Initial server greeting (220 / 220- ... 220 )
    //--------------------------------------------------
    int code = read_full_reply(ctrl, reply);
    printf("S: %s", reply);

    //--------------------------------------------------
    // 3 — Login (USER/PASS, incluindo multiline 230-)
    //--------------------------------------------------
    char cmd[512];

    snprintf(cmd, sizeof(cmd), "USER %s", user);
    send_cmd(ctrl, cmd);
    code = read_full_reply(ctrl, reply);
    printf("S: %s", reply);

    if (code == 331) {
        snprintf(cmd, sizeof(cmd), "PASS %s", pass);
        send_cmd(ctrl, cmd);
        code = read_full_reply(ctrl, reply);
        printf("S: %s", reply);
    }

    if (code != 230) {
        fprintf(stderr, "Login failed with code %d\n", code);
        close(ctrl);
        return -1;
    }

    //--------------------------------------------------
    // 4 — Enter passive mode (single-line 227)
    //--------------------------------------------------
    send_cmd(ctrl, "PASV");
    code = read_reply(ctrl, reply);
    printf("S: %s", reply);

    if (code != 227) {
        fprintf(stderr, "Expected 227 PASV reply, got %d: %s\n", code, reply);
        close(ctrl);
        return -1;
    }

    int h1,h2,h3,h4,p1,p2;
    char *p = strchr(reply, '(');
    if (!p) {
        fprintf(stderr, "Error: no '(' in PASV reply: %s\n", reply);
        close(ctrl);
        return -1;
    }
    p++;

    if (sscanf(p, "%d,%d,%d,%d,%d,%d%*[^0-9]",
               &h1,&h2,&h3,&h4,&p1,&p2) != 6) {
        fprintf(stderr, "Error parsing PASV reply: %s\n", reply);
        close(ctrl);
        return -1;
    }

    char data_ip[64];
    snprintf(data_ip, sizeof(data_ip), "%d.%d.%d.%d", h1,h2,h3,h4);
    int data_port = p1 * 256 + p2;

    printf("Data connection: %s:%d\n", data_ip, data_port);

    //--------------------------------------------------
    // 5 — Open data connection
    //--------------------------------------------------
    int data_sock = connect_socket(data_ip, data_port);

    //--------------------------------------------------
    // 6 — RETR file
    //--------------------------------------------------
    snprintf(cmd, sizeof(cmd), "RETR %s", path);
    send_cmd(ctrl, cmd);

    code = read_reply(ctrl, reply);
    printf("S: %s", reply);
    if (code != 150 && code != 125) {
        fprintf(stderr, "RETR failed with code %d\n", code);
        close(data_sock);
        close(ctrl);
        return -1;
    }

    //--------------------------------------------------
    // 7 — Receive file
    //--------------------------------------------------
    char *filename = filename_from_path(path);
    FILE *f = fopen(filename, "wb");
    if (!f) {
        perror("fopen()");
        close(data_sock);
        close(ctrl);
        return -1;
    }

    int n;
    char buffer[MAX_BUF];

    while ((n = read(data_sock, buffer, MAX_BUF)) > 0) {
        fwrite(buffer, 1, n, f);
    }

    fclose(f);
    close(data_sock);

    printf("File saved: %s\n", filename);

    //--------------------------------------------------
    // 8 — Final transfer reply
    //--------------------------------------------------
    code = read_reply(ctrl, reply);
    printf("S: %s", reply);

    //--------------------------------------------------
    // 9 — QUIT
    //--------------------------------------------------
    send_cmd(ctrl, "QUIT");
    code = read_reply(ctrl, reply);
    printf("S: %s", reply);

    close(ctrl);
    return 0;
}

