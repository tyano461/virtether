/*
 * This file is provided under a dual BSD/GPLv2 license. When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copy
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * * Neither the name of Intel Corporation nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Intel PCIe NTB Network Linux driver
 *
 * Contact Information:
 * Jon Mason <jon....@intel.com>
 */
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/module.h>
#include <linux/if_arp.h>
#include <linux/net.h>
#include <linux/in.h>
#include <linux/socket.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define DRIVER_VERSION "0.1"

#define d(s, ...)                                                                                 \
    do                                                                                            \
    {                                                                                             \
        int i;                                                                                    \
        char *p = print_buf;                                                                      \
        sprintf(print_buf, "%s(%d) %s " s "\n", __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        for (i = 0; print_buf[i]; i++)                                                            \
        {                                                                                         \
            if (print_buf[i] == '\n')                                                             \
            {                                                                                     \
                print_buf[i] = '\0';                                                              \
                printk("%s\n", p);                                                                \
                p = &print_buf[i + 1];                                                            \
            }                                                                                     \
        }                                                                                         \
    } while (0)

#define BUF_SIZE 400
#define d2c(d) (d < 0xa) ? d + '0' : ((d < 0x10) ? d - 0xa + 'a' : '.')

#ifndef min
#define min(a, b) ((a) > (b)) ? (b) : (a)
#endif

MODULE_DESCRIPTION(KBUILD_MODNAME);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

char *b2s(const void *data, int maxlen);

char print_buf[1000];
struct veth_netdev
{
    struct net_device *ndev;
};

typedef struct str_macaddr
{
    uint8_t addr[6];
} macaddr_t;

typedef struct str_ipaddr
{
    uint8_t addr[4];
} ipaddr_t;

typedef struct str_arp_data
{
    macaddr_t src_mac;
    ipaddr_t src_ip;
    macaddr_t tgt_mac;
    ipaddr_t tgt_ip;
} arp_data_t;

#define veth_TX_TIMEOUT_MS 1000
#define veth_RXQ_SIZE 1

void send_udp(uint8_t *data, size_t len);

static struct net_device *netdev;

uint8_t opposit_mac[6];
static netdev_tx_t veth_netdev_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    struct ethhdr *eth;
    struct arphdr *arp;

    eth = (struct ethhdr *)skb->data;
    // ipv6 drop
    if (eth->h_proto == 0xdd86)
        return NETDEV_TX_OK;

    // ARP
    if (eth->h_proto == 0x0608)
    {
        arp = (struct arphdr *)&eth[1];
        if (arp->ar_op == 0x0100)
        {
            // request

            send_udp(skb->data, skb->len);

        }
    }

    d("l:%x d:\n%s", skb->len, b2s(skb->data, skb->len));

    // enqueue

    ndev->stats.tx_packets++;
    ndev->stats.tx_bytes += skb->len;

    return NETDEV_TX_OK;
#if 0
err:
ndev->stats.tx_dropped++;
ndev->stats.tx_errors++;
netif_stop_queue(ndev);
returnNETDEV_TX_BUSY;
#endif
}

static int veth_netdev_open(struct net_device *ndev)
{
    struct sk_buff *skb;
    int rc, i;

    d("");

    /* Add some empty rx bufs */
    for (i = 0; i < veth_RXQ_SIZE; i++)
    {
        d("alloc size:%d", ndev->mtu + ETH_HLEN);
        skb = netdev_alloc_skb(ndev, ndev->mtu + ETH_HLEN);
        if (!skb)
        {
            d("netdev_alloc_skb failed.");
            rc = -ENOMEM;
            goto err;
        }
        // rx enqueue
    }
    d("carrier off");
    rc = 0;
    netif_carrier_off(ndev);
    ndev->state = IFF_UP;
    ndev->operstate = IF_OPER_UP;
err:
    if (skb)
        kfree(skb);
    return rc;
}

static int veth_netdev_close(struct net_device *ndev)
{
    d("");

    // queue remove
    // link down

    return 0;
}

static int veth_netdev_change_mtu(struct net_device *ndev, int new_mtu)
{
    int rc;
    d("");

    if (!netif_running(ndev))
    {
        rc = -1;
        ndev->mtu = new_mtu;
        goto error_return;
    }

    // link down
    // queue remove
    // link up
    rc = 0;
error_return:

    return rc;
}

static void veth_netdev_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
    d("");
    if (netif_running(dev))
        netif_wake_queue(dev);
}

static const struct net_device_ops veth_netdev_ops = {
    .ndo_open = veth_netdev_open,
    .ndo_stop = veth_netdev_close,
    .ndo_start_xmit = veth_netdev_start_xmit,
    .ndo_change_mtu = veth_netdev_change_mtu,
    .ndo_tx_timeout = veth_netdev_tx_timeout,
    .ndo_set_mac_address = eth_mac_addr,
};

static void veth_get_drvinfo(__attribute__((unused)) struct net_device *dev,
                            struct ethtool_drvinfo *info)
{
    d("");
    strlcpy(info->driver, KBUILD_MODNAME, sizeof(info->driver));
    strlcpy(info->version, DRIVER_VERSION, sizeof(info->version));
}

static const char veth_nic_stats[][ETH_GSTRING_LEN] = {
    "rx_packets",
    "rx_bytes",
    "rx_errors",
    "rx_dropped",
    "rx_length_errors",
    "rx_frame_errors",
    "rx_fifo_errors",
    "tx_packets",
    "tx_bytes",
    "tx_errors",
    "tx_dropped",
};

static int veth_get_stats_count(__attribute__((unused)) struct net_device *dev)
{
    d("");
    return ARRAY_SIZE(veth_nic_stats);
}

static int veth_get_sset_count(struct net_device *dev, int sset)
{
    d("");
    switch (sset)
    {
    case ETH_SS_STATS:
        return veth_get_stats_count(dev);
    default:
        return -EOPNOTSUPP;
    }
}

static void veth_get_strings(struct net_device *dev, u32 sset, u8 *data)
{
    d("");
    switch (sset)
    {
    case ETH_SS_STATS:
        memcpy(data, *veth_nic_stats, sizeof(veth_nic_stats));
    }
}

static void
veth_get_ethtool_stats(struct net_device *dev,
                      __attribute__((unused)) struct ethtool_stats *stats,
                      u64 *data)
{
    int i = 0;

    d("");
    data[i++] = dev->stats.rx_packets;
    data[i++] = dev->stats.rx_bytes;
    data[i++] = dev->stats.rx_errors;
    data[i++] = dev->stats.rx_dropped;
    data[i++] = dev->stats.rx_length_errors;
    data[i++] = dev->stats.rx_frame_errors;
    data[i++] = dev->stats.rx_fifo_errors;
    data[i++] = dev->stats.tx_packets;
    data[i++] = dev->stats.tx_bytes;
    data[i++] = dev->stats.tx_errors;
    data[i++] = dev->stats.tx_dropped;
}

static const struct ethtool_ops veth_ethtool_ops = {
    .get_drvinfo = veth_get_drvinfo,
    .get_sset_count = veth_get_sset_count,
    .get_strings = veth_get_strings,
    .get_ethtool_stats = veth_get_ethtool_stats,
    .get_link = ethtool_op_get_link,
};

static int __init veth_netdev_init_module(void)
{
    int rc = 0;
    struct veth_netdev *dev;

    d("%s", KBUILD_MODNAME);

    netdev = alloc_etherdev(sizeof(struct veth_netdev));
    if (!netdev)
        return -ENOMEM;

    dev = netdev_priv(netdev);
    dev->ndev = netdev;
    netdev->features = NETIF_F_HIGHDMA;

    netdev->hw_features = netdev->features;
    netdev->watchdog_timeo = msecs_to_jiffies(veth_TX_TIMEOUT_MS);

    random_ether_addr(netdev->perm_addr);
    memcpy(netdev->dev_addr, netdev->perm_addr, netdev->addr_len);
    d("mac addr:%s", b2s(netdev->dev_addr, netdev->addr_len));

    netdev->netdev_ops = &veth_netdev_ops;
    netdev->ethtool_ops = &veth_ethtool_ops;

    /* dummy opposit */
    random_ether_addr(opposit_mac);

    // create tx queue

    netdev->mtu = 1500 - ETH_HLEN;
    rc = register_netdev(netdev);

    d("%s: %s created rc:%d", KBUILD_MODNAME, netdev->name, rc);
    return 0;

#if 0
err:
    free_netdev(netdev);
#endif
    return rc;
}
module_init(veth_netdev_init_module);

static void __exit veth_netdev_exit_module(void)
{
    // struct veth_netdev *dev = netdev_priv(netdev);

    unregister_netdev(netdev);
    free_netdev(netdev);

    d("%s: Driver removed", KBUILD_MODNAME);
}
module_exit(veth_netdev_exit_module);

static char buf[BUF_SIZE];
char *b2s(const void *data, int maxlen)
{
    int i;
    int len = min(maxlen, (BUF_SIZE - 1) / 4);
    uint8_t *o = (uint8_t *)data;
    char *p = buf;
    char u, l;
    for (i = 0; i < len; i++)
    {
        u = ((*o) & 0xf0) >> 4;
        l = ((*o) & 0x0f) >> 0;
        *p++ = d2c(u);
        *p++ = d2c(l);
        if ((i % 16) == 15)
        {
            *p++ = '\n';
        }
        else
        {
            *p++ = ' ';
        }
        o++;
    }
    if ((*(p - 1)) != '\n')
        *p++ = '\n';
    *p++ = '\0';
    return buf;
}

void send_udp(uint8_t *data, size_t len)
{
    struct socket *sock;
    struct sockaddr_in sin;
    struct msghdr msg;
    struct iovec iov;
    int ret;
    ret = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &sock);
    if (ret < 0)
    {
        d("sock_create error");
        goto error_return;
    }

    sin.sin_family = AF_INET;
    sin.sin_port = htons(7144);
    sin.sin_addr.s_addr = 0x0100007f;

    ret = sock->ops->connect(sock, (struct sockaddr *)&sin, sizeof(struct sockaddr), 0);
    if (ret < 0)
    {
        d("connect error");
        goto error_return;
    }

    msg.msg_flags = 0;
    msg.msg_name = &sin;
    msg.msg_namelen = sizeof(struct sockaddr_in);
    msg.msg_control = NULL;
    msg.msg_controllen = 0;

    iov.iov_base = data;
    iov.iov_len = len;

    iov_iter_init(&msg.msg_iter, READ, &iov, 1, len);

    ret = sock_sendmsg(sock, &msg);
    if (ret < 0)
    {
        d("send error");
    }

error_return:
    return;
}