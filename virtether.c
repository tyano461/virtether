#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/inetdevice.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>

#define MODULE_NAME "vth"
#define DRIVER_NAME "virtether"

#define d(s, ...)                                                            \
  do                                                                         \
  {                                                                          \
    printk("%s(%d) %s" s "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
  } while (0)

MODULE_LICENSE("GPLv2");

static int dummy_func(struct net_device *dev);
static int start_xmit(struct sk_buff *skb, struct net_device *dev);
static int ndo_do_ioctl(struct net_device *dev,
                        struct ifreq *ifr, int cmd);
static void ndo_tx_timeout(struct net_device *dev,
                           unsigned int txqueue);
struct net_device_ops driver_netdevops = {
    .ndo_init = dummy_func,
    .ndo_open = dummy_func,
    .ndo_stop = dummy_func,
    .ndo_start_xmit = start_xmit,
    .ndo_do_ioctl = ndo_do_ioctl,
    .ndo_tx_timeout = ndo_tx_timeout,
};

static int dummy_func(struct net_device *dev)
{
  return 0;
}

static int start_xmit(struct sk_buff *skb, struct net_device *dev)
{
  return 0;
}

static int ndo_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
  return 0;
}
static void ndo_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
}

#define IMPL_BASE_FUNC(s)    \
  static int s##_init(void)  \
  {                          \
    d("");                   \
    return 0;                \
  }                          \
  static void s##_exit(void) \
  {                          \
    d("");                   \
  }                          \
  module_init(s##_init);     \
  module_exit(s##_exit)

IMPL_BASE_FUNC(MODULE_NAME);