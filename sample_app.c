#include <pthread.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <arpa/inet.h>

/* definitions */
#define NEVENTS 2
#define TARGET_SERVER_IP "192.168.100.2"
#define TARGET_SERVER_PORT 7151
#define SERVER_IP INADDR_ANY
#define SERVER_PORT 7150
#define STDIN_BUF_SIZE 256
#define BUF_SIZE 0x10000

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define d(s, ...)                                                                 \
    do                                                                            \
    {                                                                             \
        pthread_mutex_lock(&mutex);                                               \
        printf("%s(%d) %s " s "\n", __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        pthread_mutex_unlock(&mutex);                                             \
    } while (0)

#define ERRRET(c, s, ...)                                                                      \
    do                                                                                         \
    {                                                                                          \
        if (c)                                                                                 \
        {                                                                                      \
            fprintf(stderr, "%s(%d) %s " s "\n", __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
            goto error_return;                                                                 \
        }                                                                                      \
    } while (0)

#ifndef min
#define min(a, b) (a) > (b) ? (b) : (a)
#endif

enum SERVER_STATUS
{
    ST_SRV_NONE,
    ST_SRV_RUNNING,
    ST_SRV_FAILED,
};

static void *server_thread(void *param);
static void init_sock_addr(int *sock, struct sockaddr_in *addr, uint32_t ip, uint16_t port);
static int set_epoll_event(int epfd, int sock, struct epoll_event *ev);
static char *b2s(uint8_t *data, size_t len);
static void send_tcp_to_freertos(uint8_t *data, size_t len);

/* variables */
pthread_t th;
enum SERVER_STATUS server_status = ST_SRV_NONE;
static char stdin_buf[STDIN_BUF_SIZE];
static __thread char rx_buf[BUF_SIZE];
static char print_buf[BUF_SIZE];
static pthread_mutex_t mutex;

int main(void)
{
    int ret;
    char *p;

    pthread_mutex_init(&mutex, NULL);
    ret = pthread_create(&th, NULL, server_thread, NULL);
    if (ret)
    {
        d("pthread_create failed.");
        return 0;
    }

    while (server_status != ST_SRV_FAILED)
    {
        if (fgets(stdin_buf, STDIN_BUF_SIZE, stdin))
        {
            p = strchr(stdin_buf, '\n');
            if (!p)
                continue;

            *p = '\0';
            send_tcp_to_freertos(p, strlen(p));
        }
    }
    return 0;
}

static void init_sock_addr(int *sock, struct sockaddr_in *addr, uint32_t ip, uint16_t port)
{
    *sock = socket(AF_INET, SOCK_STREAM, 0);
    memset(addr, 0, sizeof(struct sockaddr_in));
    addr->sin_family = AF_INET;
    addr->sin_addr.s_addr = ip;
    addr->sin_port = htons(port);
}

static int set_epoll_event(int epfd, int sock, struct epoll_event *ev)
{

    memset(ev, 0, sizeof(struct epoll_event));
    ev->events = EPOLLIN;
    ev->data.fd = sock;
    return epoll_ctl(epfd, EPOLL_CTL_ADD, sock, ev);
}

static inline char b2c(uint8_t b)
{
    return (0x20 <= b && b <= 0x7e) ? b : '.';
}

static char *b2s(uint8_t *data, size_t org_len)
{
    int i;
    char *p;
    char str[0x11] = {0};
    size_t len = min(sizeof(print_buf) / 5, org_len);

    p = (char *)print_buf;
    for (i = 0; i < len; i++)
    {
        p += sprintf(p, "%02x ", data[i]);
        str[i % 16] = b2c(data[i]);

        if ((i % 16) == 15)
        {
            p += sprintf(p, "%s\n", str);
        }
    }

    if (i % 16)
    {
        str[i] = '\0';
        memset(p, ' ', (16 - (i % 16)) * 3);
        p += (16 - (i % 16)) * 3;
        p += sprintf(p, "%s\n", str);
    }
    return print_buf;
}

static void *server_thread(void *param)
{
    int sock, csock = -1;
    struct sockaddr_in addr, caddr;
    socklen_t csize;
    int epfd;
    struct epoll_event ev[NEVENTS], ev_ret[NEVENTS];
    char buf[2048];
    int i;
    int nfds;
    int n;
    bool found;
    (void)param;

    init_sock_addr(&sock, &addr, SERVER_IP, SERVER_PORT);

    n = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    ERRRET(n != 0, "bind failed.%d", n);

    n = listen(sock, NEVENTS);
    ERRRET(n != 0, "listen failed.");

    epfd = epoll_create(NEVENTS);
    ERRRET(epfd < 0, "epoll_create failed.");

    n = set_epoll_event(epfd, sock, &ev[0]);
    ERRRET(n != 0, "epoll_ctl failed.");

    server_status = ST_SRV_RUNNING;
    while (1)
    {
        nfds = epoll_wait(epfd, ev_ret, NEVENTS, -1);
        ERRRET(nfds <= 0, "epoll_wait");

        found = false;
        for (i = 0; i < nfds; i++)
        {
            if (ev_ret[i].data.fd == sock)
            {
                csize = sizeof(caddr);
                csock = accept(sock, (struct sockaddr *)&caddr, &csize);
                ev[i].events = EPOLLIN;
                ev[i].data.fd = csock;
                epoll_ctl(epfd, EPOLL_CTL_ADD, csock, &ev[i]);
                d("accept from %x", caddr.sin_addr.s_addr);
            }
            else
            {
                n = read(ev_ret[i].data.fd, rx_buf, BUF_SIZE);
                if (n == 0)
                {
                    epoll_ctl(epfd, EPOLL_CTL_DEL, ev_ret[i].data.fd, NULL);
                    close(ev_ret[i].data.fd);
                    d("close");
                }
                else
                {
                    d("received from FreeRTOS: %s", b2s(rx_buf, n));
                    send(ev_ret[i].data.fd, rx_buf, n, 0);
                }
            }
        }
    }

error_return:
    server_status = ST_SRV_FAILED;
    if (sock >= 0)
        close(sock);
    if (csock >= 0)
        close(csock);
    return NULL;
}

static void send_tcp_to_freertos(uint8_t *data, size_t len)
{
    int sock, ret;
    struct sockaddr_in addr;

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    ERRRET(sock < 0, "socket create failed.%d", sock);

    addr.sin_addr.s_addr = inet_addr(TARGET_SERVER_IP);
    addr.sin_family = PF_INET;
    addr.sin_port = htons(TARGET_SERVER_PORT);

    ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    ERRRET(ret, "connect failed.%d", ret);

    ret = send(sock, data, len, 0);
    ERRRET(ret < 0, "send failed.%d", ret);

    ret = recv(sock, rx_buf, BUF_SIZE, 0);
    ERRRET(ret < 0, "recv failed.%d", ret);

    d("send to freertos");

error_return:
    if (sock >= 0)
        close(sock);
    return;
}