#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/inetdevice.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <net/dst.h>
#include <net/xfrm.h>

#define MODULE_NAME "vth"
#define DRIVER_NAME "virtether"

enum
{
  VETH_INFO_UNSPEC,
  VETH_INFO_MAC,
  VETH_INFO_PEER,
  VETH_INFO_PEER_MAC,

  __VETH_INFO_MAX
#define VETH_INFO_MAX (__VETH_INFO_MAX - 1)
};

#define d(s, ...)                                                             \
  do                                                                          \
  {                                                                           \
    printk("%s(%d) %s " s "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
  } while (0)

MODULE_LICENSE("GPL");

/* definitions */

/* functions */
static int dummy_func(struct net_device *dev);
static int start_xmit(struct sk_buff *skb, struct net_device *dev);
static int ndo_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);
static void ndo_tx_timeout(struct net_device *dev, unsigned int txqueue);
static void veth_setup(struct net_device *dev);
static int veth_newlink(struct net *src_net, struct net_device *dev, struct nlattr *tb[], struct nlattr *data[], struct netlink_ext_ack *extack);
static void veth_dellink(struct net_device *dev, struct list_head *head);

/* variables */
struct net_device_ops dev_ops = {
    .ndo_init = dummy_func,
    .ndo_open = dummy_func,
    .ndo_stop = dummy_func,
    .ndo_start_xmit = start_xmit,
    .ndo_do_ioctl = ndo_do_ioctl,
    .ndo_tx_timeout = ndo_tx_timeout,
};

static const struct nla_policy veth_policy[VETH_INFO_MAX + 1] = {
    [VETH_INFO_MAC] = {.type = NLA_BINARY, .len = ETH_ALEN},
    [VETH_INFO_PEER] = {.type = NLA_STRING},
    [VETH_INFO_PEER_MAC] = {.type = NLA_BINARY, .len = ETH_ALEN},
};

static struct rtnl_link_ops link_ops = {
    .kind = DRIVER_NAME,
    .priv_size = 0,
    .setup = veth_setup,
    .newlink = veth_newlink,
    .dellink = veth_dellink,
    .policy = veth_policy,
    .maxtype = 1,
};


/* functions */
static int dummy_func(struct net_device *dev)
{
  d("");
  return 0;
}

static int start_xmit(struct sk_buff *skb, struct net_device *dev)
{
  d("");
  return 0;
}

static int ndo_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
  d("");
  return 0;
}
static void ndo_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
  d("");
}

static void veth_setup(struct net_device *dev)
{
  d("IN");
  ether_setup(dev);

  dev->netdev_ops = &dev_ops;
  dev->ethtool_ops = NULL;
  dev->features |= NETIF_F_LLTX;
  netif_carrier_off(dev);
  d("OUT");
}
static int veth_newlink(struct net *src_net, struct net_device *dev, struct nlattr *tb[], struct nlattr *data[], struct netlink_ext_ack *extack)
{
  d("");
  return 0;
}
static void veth_dellink(struct net_device *dev, struct list_head *head)
{
  d("");
}

#define IMPL_BASE_FUNC(s)                \
  static int s##_init(void)              \
  {                                      \
    int ret;                             \
    ret = rtnl_link_register(&link_ops); \
    d("rtnl_link_register:%d", ret);     \
    return ret;                          \
  }                                      \
  static void s##_exit(void)             \
  {                                      \
    d("");                               \
    rtnl_link_unregister(&link_ops);     \
  }                                      \
  module_init(s##_init);                 \
  module_exit(s##_exit)

IMPL_BASE_FUNC(MODULE_NAME);