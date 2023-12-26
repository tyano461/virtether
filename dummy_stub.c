#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/queue.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/fcntl.h>
#include "ve_common.h"

#define NEVENTS 2
#define SERVER_ADDR INADDR_ANY
#define PBUF_SIZE 2000

#ifndef min
#define min(a, b) ((a) > (b)) ? (b) : (a)
#endif

#define d2c(c) (c < 0xa) ? '0' + c : (c <= 0xf) ? 'a' + (c - 0xa) \
                                                : '.'
#define d(s, ...)                                                                 \
    do                                                                            \
    {                                                                             \
        printf("%s(%d) %s " s "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)

#define ERRRET(c, s, ...)                                                                      \
    do                                                                                         \
    {                                                                                          \
        if (c)                                                                                 \
        {                                                                                      \
            fprintf(stderr, "%s(%d) %s " s "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
            goto error_return;                                                                 \
        }                                                                                      \
    } while (0)

typedef struct str_txlist
{
    TAILQ_ENTRY(str_txlist)
    entry;
    size_t len;
    uint8_t data[0];
} txlist_t;

/* functions */
void *recv_main(void *param);
char *b2s(uint8_t *data, size_t len);
void receive_data_to_ipstack(uint8_t *data, size_t len);
static void send_to_qemu(uint8_t *data, size_t len);

/* variables */
pthread_mutex_t mutex;
pthread_cond_t cond;
TAILQ_HEAD(tqhead, str_txlist)
qmhead;

uint8_t send_buf[0x200];
size_t send_size;

/* functions */
int main(void)
{
    pthread_t receiver;
    txlist_t *queue;

    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&cond, NULL);

    TAILQ_INIT(&qmhead);

    pthread_create(&receiver, NULL, recv_main, NULL);

    while (1)
    {
        send_size = 0;
        pthread_mutex_lock(&mutex);
        queue = TAILQ_FIRST(&qmhead);
        d("from qemu q:%p", queue);
        if (queue)
        {
            TAILQ_REMOVE(&qmhead, queue, entry);
            d("%s", b2s(queue->data, queue->len));
            if (queue->len)
            {
                memcpy(send_buf, queue->data, queue->len);
                send_size = queue->len;
            }
            free(queue);
        }
        else
        {
            pthread_cond_wait(&cond, &mutex);
        }
        pthread_mutex_unlock(&mutex);
        if (send_size)
        {
            receive_data_to_ipstack(send_buf, send_size);
        }
    }

    return 0;
}

void init_sock_addr(int *sock, struct sockaddr_in *addr, uint32_t ip, uint16_t port)
{
    *sock = socket(AF_INET, SOCK_DGRAM, 0);
    addr->sin_family = AF_INET;
    addr->sin_addr.s_addr = ip;
    addr->sin_port = htons(port);
}

int set_epoll_event(int epfd, int sock, struct epoll_event *ev)
{

    memset(ev, 0, sizeof(struct epoll_event));
    ev->events = EPOLLIN;
    ev->data.fd = sock;
    return epoll_ctl(epfd, EPOLL_CTL_ADD, sock, ev);
}

void *recv_main(void *param)
{
    int sock1, sock2;
    struct sockaddr_in addr1, addr2;
    int epfd;
    struct epoll_event ev[NEVENTS], ev_ret[NEVENTS];
    char buf[2048];
    int i;
    int nfds;
    int n;
    txlist_t *queue;
    bool found;
    (void)param;

    init_sock_addr(&sock1, &addr1, SERVER_ADDR, SERVER_PORT_FROM_QEMU);
    init_sock_addr(&sock2, &addr2, SERVER_ADDR, SERVER_PORT_FROM_DRIVER);

    n = bind(sock1, (struct sockaddr *)&addr1, sizeof(addr1));
    ERRRET(n != 0, "bind");

    n = bind(sock2, (struct sockaddr *)&addr2, sizeof(addr2));
    ERRRET(n != 0, "bind");

    epfd = epoll_create(NEVENTS);
    ERRRET(epfd < 0, "epoll_create failed.");

    n = set_epoll_event(epfd, sock1, &ev[0]);
    ERRRET(n != 0, "epoll_ctl failed.");

    n = set_epoll_event(epfd, sock2, &ev[1]);
    ERRRET(n != 0, "epoll_ctl failed.");

    while (1)
    {
        nfds = epoll_wait(epfd, ev_ret, NEVENTS, -1);
        ERRRET(nfds <= 0, "epoll_wait");

        found = false;
        for (i = 0; i < nfds; i++)
        {
            if (ev_ret[i].data.fd == sock1)
            {
                found = true;
                n = recv(sock1, buf, sizeof(buf), 0);
                if (n)
                {
                    queue = malloc(n + sizeof(txlist_t));
                    queue->len = n;
                    memcpy(queue->data, buf, n);
                    pthread_mutex_lock(&mutex);
                    TAILQ_INSERT_TAIL(&qmhead, queue, entry);
                    pthread_cond_signal(&cond);
                    pthread_mutex_unlock(&mutex);
                    d("cond signal");
                }
            }
            else if (ev_ret[i].data.fd == sock2)
            {
                found = true;
                n = recv(sock2, buf, sizeof(buf), 0);
                send_to_qemu(buf, n);
            }
        }

        if (!found)
        {
            d("unknown receive fd:%d sock:%d/%d", ev_ret[i].data.fd, sock1, sock2);
        }
    }

error_return:
    if (sock1 >= 0)
        close(sock1);
    return NULL;
}

char print_buf[PBUF_SIZE];
char *b2s(uint8_t *data, size_t orgsize)
{
    char *p;
    int i;
    char u, l;
    int len = min(orgsize, (PBUF_SIZE / 4));

    p = print_buf;
    for (i = 0; i < len; i++)
    {
        u = (data[i] & 0xf0) >> 4;
        l = (data[i] & 0x0f) >> 0;
        *p++ = d2c(u);
        *p++ = d2c(l);
        *p++ = ' ';
        if (i % 16 == 15)
        {
            *(p - 1) = '\n';
        }
    }
    if (*(p - 1) != '\n')
    {
        *p++ = '\n';
    }
    *p++ = '\0';
    return print_buf;
}

void receive_data_to_ipstack(uint8_t *data, size_t len)
{
    int fd;
    int written;
    const char *devname = "/dev/veth_cdev";

    fd = open(devname, O_WRONLY);
    ERRRET(fd < 0, "open failed.");

    written = write(fd, data, len);
    ERRRET(written <= 0, "write failed.");

    d("write:%d", written);
error_return:
    if (fd >= 0)
        close(fd);
}

static void send_to_qemu(uint8_t *data, size_t len)
{
    int sock;
    struct sockaddr_in addr;
    int ret;

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    ERRRET(sock < 0, "socket create failed.");

    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT_TO_QEMU);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    ret = sendto(sock, data, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    d("sendto(%ld): %d data:\n%s", len, ret, b2s(data, len));

error_return:
    if (sock >= 0)
        close(sock);
}