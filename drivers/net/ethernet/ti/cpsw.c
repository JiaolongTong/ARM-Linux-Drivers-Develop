/*
 * Texas Instruments Ethernet Switch Driver
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/if_vlan.h>
#include <linux/net_switch_config.h>

#include <linux/cpsw.h>
#include <plat/dmtimer.h>
#include "cpsw_ale.h"
#include "davinci_cpdma.h"


#define CPSW_DEBUG	(NETIF_MSG_HW		| NETIF_MSG_WOL		| \
			 NETIF_MSG_DRV		| NETIF_MSG_LINK	| \
			 NETIF_MSG_IFUP		| NETIF_MSG_INTR	| \
			 NETIF_MSG_PROBE	| NETIF_MSG_TIMER	| \
			 NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	| \
			 NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	| \
			 NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	| \
			 NETIF_MSG_RX_STATUS)

#define msg(level, type, format, ...)				\
do {								\
	if (netif_msg_##type(priv) && net_ratelimit())		\
		dev_##level(priv->dev, format, ## __VA_ARGS__);	\
} while (0)

#define CPDMA_RXTHRESH		0x0c0
#define CPDMA_RXFREE		0x0e0
#define CPDMA_TXHDP_VER1	0x100
#define CPDMA_TXHDP_VER2	0x200
#define CPDMA_RXHDP_VER1	0x120
#define CPDMA_RXHDP_VER2	0x220
#define CPDMA_TXCP_VER1		0x140
#define CPDMA_TXCP_VER2		0x240
#define CPDMA_RXCP_VER1		0x160
#define CPDMA_RXCP_VER2		0x260

#define CPSW_POLL_WEIGHT	64
#define CPSW_MIN_PACKET_SIZE	60
#define CPSW_MAX_PACKET_SIZE	(1500 + 14 + 4 + 4)
#define CPSW_PHY_SPEED		1000

#define CPSW_PRIMAP(shift, priority)	(priority << (shift * 4))

#define slave(priv, idx)	((priv)->slaves + idx)
#define switchcmd(__cmd__)	((__cmd__)->cmd_data.switchcmd)
#define portcmd(__cmd__)	((__cmd__)->cmd_data.portcmd)
#define priocmd(__cmd__)	((__cmd__)->cmd_data.priocmd)

/* CPSW control module masks */
#define CPSW_INTPACEEN		(0x3 << 16)
#define CPSW_INTPRESCALE_MASK	(0x7FF << 0)
#define CPSW_CMINTMAX_CNT	63
#define CPSW_CMINTMIN_CNT	2
#define CPSW_CMINTMAX_INTVL	(1000 / CPSW_CMINTMIN_CNT)
#define CPSW_CMINTMIN_INTVL	((1000 / CPSW_CMINTMAX_CNT) + 1)

#define cpsw_enable_irq(priv)	\
	do {			\
		u32 i;		\
		for (i = 0; i < priv->num_irqs; i++) \
			enable_irq(priv->irqs_table[i]); \
	} while (0);
#define cpsw_disable_irq(priv)	\
	do {			\
		u32 i;		\
		for (i = 0; i < priv->num_irqs; i++) \
			disable_irq_nosync(priv->irqs_table[i]); \
	} while (0);

#define ALE_ALL_PORTS		0x7

#define CPSW_CPDMA_EOI_REG	0x894
#define CPSW_TIMER_MASK		0xA0908
#define CPSW_TIMER_CAP_REG	0xFD0
#define CPSW_RX_TIMER_REQ	5
#define CPSW_TX_TIMER_REQ	6

#ifdef CONFIG_TI_CPSW_DUAL_EMAC

/* Enable VLAN aware mode to add VLAN for induvudual interface */
#define CPSW_VLAN_AWARE_MODE

#define cpsw_get_slave_ndev(priv, __slave_no__)		\
	(priv->slaves[__slave_no__].ndev)
#define cpsw_get_slave_priv(priv, __slave_no__)		\
	netdev_priv(priv->slaves[__slave_no__].ndev)
#define for_each_slave(priv, func, arg...)			\
	do {							\
		(func)((priv)->slaves + priv->emac_port, ##arg);\
	} while (0)
#define cpsw_dual_emac_source_port_detect(status, priv, ndev, skb)	\
	do {								\
		if (CPDMA_RX_SOURCE_PORT(status) == 1) {		\
			ndev = cpsw_get_slave_ndev(priv, 0);		\
			priv = netdev_priv(ndev);			\
			skb->dev = ndev;				\
		} else if (CPDMA_RX_SOURCE_PORT(status) == 2) {		\
			ndev = cpsw_get_slave_ndev(priv, 1);		\
			priv = netdev_priv(ndev);			\
			skb->dev = ndev;				\
		}							\
	} while (0)
#define cpsw_add_switch_mode_bcast_ale_entries(priv, slave_port)
#define cpsw_update_slave_open_state(priv, state)		\
	priv->slaves[priv->emac_port].open_stat = state;
#define cpsw_slave_phy_index(priv)	((priv)->emac_port)

#else	/* CONFIG_TI_CPSW_DUAL_EMAC */

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#define VLAN_SUPPORT
#define CPSW_VLAN_AWARE_MODE
#endif

#define cpsw_get_slave_ndev(priv, __slave_no__)		NULL
#define cpsw_get_slave_priv(priv, __slave_no__)		NULL
#define for_each_slave(priv, func, arg...)			\
	do {							\
		int idx;					\
		for (idx = 0; idx < (priv)->data.slaves; idx++)	\
			(func)((priv)->slaves + idx, ##arg);	\
	} while (0)
#define cpsw_dual_emac_source_port_detect(status, priv, ndev, skb)
#define cpsw_add_switch_mode_bcast_ale_entries(priv, slave_port)	\
	cpsw_ale_add_mcast(priv->ale, priv->ndev->broadcast,		\
			   1 << slave_port, 0, 0)
#define cpsw_update_slave_open_state(priv, state)
#define cpsw_slave_phy_index(priv)	((priv)->data.ethtool_slave)

#endif	/* CONFIG_TI_CPSW_DUAL_EMAC */

#ifdef CPSW_VLAN_AWARE_MODE
#define CPSW_VLAN_AWARE		BIT(1)
#define CPSW_ALE_VLAN_AWARE	1
#else
#define CPSW_VLAN_AWARE		0x0
#define CPSW_ALE_VLAN_AWARE	0
#endif

struct omap_dm_timer *dmtimer_rx;
struct omap_dm_timer *dmtimer_tx;

extern u32 omap_ctrl_readl(u16 offset);
extern void omap_ctrl_writel(u32 val, u16 offset);

static int debug_level;
module_param(debug_level, int, 0);
MODULE_PARM_DESC(debug_level, "cpsw debug level (NETIF_MSG bits)");

static int ale_ageout = 10;
module_param(ale_ageout, int, 0);
MODULE_PARM_DESC(ale_ageout, "cpsw ale ageout interval (seconds)");

static int rx_packet_max = CPSW_MAX_PACKET_SIZE;
module_param(rx_packet_max, int, 0);
MODULE_PARM_DESC(rx_packet_max, "maximum receive packet size (bytes)");

struct cpsw_ss_regs {
	u32	id_ver;
	u32	soft_reset;
	u32	control;
	u32	int_control;
	u32	rx_thresh_en;
	u32	rx_en;
	u32	tx_en;
	u32	misc_en;
	u32	mem_allign1[8];
	u32	rx_thresh_stat;
	u32	rx_stat;
	u32	tx_stat;
	u32	misc_stat;
	u32	mem_allign2[8];
	u32	rx_imax;
	u32	tx_imax;
};

struct cpsw_regs {
	u32	id_ver;
	u32	control;
	u32	soft_reset;
	u32	stat_port_en;
	u32	ptype;
	u32	soft_idle;
	u32	thru_rate;
	u32	gap_thresh;
	u32	tx_start_wds;
	u32	flow_control;
};

struct cpsw_slave_regs {
	u32	max_blks;
	u32	blk_cnt;
	u32	flow_thresh;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	ts_seq_mtype;
#ifdef CONFIG_ARCH_TI814X
	u32	ts_ctl;
	u32	ts_seq_ltype;
	u32	ts_vlan;
#endif
	u32	sa_lo;
	u32	sa_hi;
};

struct cpsw_host_regs {
	u32	max_blks;
	u32	blk_cnt;
	u32	flow_thresh;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	cpdma_tx_pri_map;
	u32	cpdma_rx_chan_map;
};

struct cpsw_sliver_regs {
	u32	id_ver;
	u32	mac_control;
	u32	mac_status;
	u32	soft_reset;
	u32	rx_maxlen;
	u32	__reserved_0;
	u32	rx_pause;
	u32	tx_pause;
	u32	__reserved_1;
	u32	rx_pri_map;
};

struct cpsw_hw_stats {
	u32	rxgoodframes;
	u32	rxbroadcastframes;
	u32	rxmulticastframes;
	u32	rxpauseframes;
	u32	rxcrcerrors;
	u32	rxaligncodeerrors;
	u32	rxoversizedframes;
	u32	rxjabberframes;
	u32	rxundersizedframes;
	u32	rxfragments;
	u32	__pad_0[2];
	u32	rxoctets;
	u32	txgoodframes;
	u32	txbroadcastframes;
	u32	txmulticastframes;
	u32	txpauseframes;
	u32	txdeferredframes;
	u32	txcollisionframes;
	u32	txsinglecollframes;
	u32	txmultcollframes;
	u32	txexcessivecollisions;
	u32	txlatecollisions;
	u32	txunderrun;
	u32	txcarriersenseerrors;
	u32	txoctets;
	u32	octetframes64;
	u32	octetframes65t127;
	u32	octetframes128t255;
	u32	octetframes256t511;
	u32	octetframes512t1023;
	u32	octetframes1024tup;
	u32	netoctets;
	u32	rxsofoverruns;
	u32	rxmofoverruns;
	u32	rxdmaoverruns;
};

struct cpsw_slave {
	struct cpsw_slave_regs __iomem	*regs;
	struct cpsw_sliver_regs __iomem	*sliver;
	int				slave_num;
	u32				mac_control;
	struct cpsw_slave_data		*data;
	struct phy_device		*phy;
	u32				port_vlan;
	struct net_device		*ndev;
	u32				open_stat;
};

struct cpsw_priv {
	spinlock_t			lock;
	struct platform_device		*pdev;
	struct net_device		*ndev;
	struct resource			*cpsw_res;
	struct resource			*cpsw_ss_res;
	struct napi_struct		napi;
#define napi_to_priv(napi)	container_of(napi, struct cpsw_priv, napi)
	struct device			*dev;
	struct cpsw_platform_data	data;
	struct cpsw_regs __iomem	*regs;
	struct cpsw_ss_regs __iomem	*ss_regs;
	struct cpsw_hw_stats __iomem	*hw_stats;
	struct cpsw_host_regs __iomem	*host_port_regs;
	u32				msg_enable;
	u32				coal_intvl;
	u32				bus_freq_mhz;
	struct net_device_stats		stats;
	int				rx_packet_max;
	int				host_port;
	struct clk			*clk;
	u8				mac_addr[ETH_ALEN];
	struct cpsw_slave		*slaves;
	struct cpdma_ctlr		*dma;
	struct cpdma_chan		*txch, *rxch;
	struct cpsw_ale			*ale;
	u32				emac_port;
	u8				port_state[3];
	/* snapshot of IRQ numbers */
	u32 irqs_table[4];
	u32 num_irqs;
};

#ifdef CONFIG_TI_CPSW_DUAL_EMAC

static inline void cpsw_p0_fifo_type_select(struct cpsw_priv *priv)
{
	u32 reg;
	reg = readl(&priv->host_port_regs->flow_thresh);
	reg &= ~(0x3 << 16);
	reg |= 0x1 << 16;
	writel(reg, &priv->host_port_regs->flow_thresh);
}

static inline void cpsw_add_dual_emac_mode_default_ale_entries(
		struct cpsw_priv *priv, struct cpsw_slave *slave,
		u32 slave_port)
{
	writel(slave->port_vlan, &slave->regs->port_vlan);
	cpsw_ale_add_vlan(priv->ale, slave->port_vlan,
		1 << slave_port | 1 << priv->host_port, 0,
		1 << slave_port | 1 << priv->host_port,
		1 << slave_port | 1 << priv->host_port);
	cpsw_ale_vlan_add_mcast(priv->ale,
		priv->ndev->broadcast,
		1 << slave_port | 1 << priv->host_port,
		slave->port_vlan, 0, 0);
	cpsw_ale_add_ucast(priv->ale, priv->mac_addr,
		priv->host_port, 0);
	cpsw_ale_add_vlan(priv->ale, 0,
		ALE_ALL_PORTS << priv->host_port,
		ALE_ALL_PORTS << priv->host_port, priv->host_port, 0);
}

static int cpsw_common_res_usage_state(struct cpsw_priv *priv)
{
	u32 i;
	u32 usage_count = 0;

	for (i = 0; i < priv->data.slaves; i++)
		if (priv->slaves[i].open_stat)
			usage_count++;

	return usage_count;
}

static inline int cpsw_tx_packet_submit(struct net_device *ndev,
			struct cpsw_priv *priv, struct sk_buff *skb)
{
	if (ndev == cpsw_get_slave_ndev(priv, 0))
		return cpdma_chan_submit(priv->txch, skb, skb->data,
				  skb->len, 1, GFP_KERNEL);
	else
		return cpdma_chan_submit(priv->txch, skb, skb->data,
				  skb->len, 2, GFP_KERNEL);
}

#define cpsw_add_switch_mode_default_ale_entries(priv)

#else	/* CONFIG_TI_CPSW_DUAL_EMAC */

#define cpsw_p0_fifo_type_select(priv)
#define cpsw_add_dual_emac_mode_default_ale_entries(priv, slave, slave_port)
#define cpsw_update_slave_open_state(priv, state)
#define cpsw_common_res_usage_state(priv)	0
#define cpsw_tx_packet_submit(ndev, priv, skb)		\
	cpdma_chan_submit(priv->txch, skb, skb->data,	\
			  skb->len, 0, GFP_KERNEL)

static inline void cpsw_add_switch_mode_default_ale_entries(
			struct cpsw_priv *priv)
{
	cpsw_ale_add_ucast(priv->ale, priv->mac_addr,
			   priv->host_port, 0);
	cpsw_ale_add_mcast(priv->ale, priv->ndev->broadcast,
			   1 << priv->host_port, 0, 0);
}

#endif	/* CONFIG_TI_CPSW_DUAL_EMAC */

static int cpsw_set_coalesce(struct net_device *ndev,
			struct ethtool_coalesce *coal);

static void cpsw_intr_enable(struct cpsw_priv *priv)
{
	__raw_writel(0xFF, &priv->ss_regs->tx_en);
	__raw_writel(0xFF, &priv->ss_regs->rx_en);

	cpdma_ctlr_int_ctrl(priv->dma, true);
	return;
}

static void cpsw_intr_disable(struct cpsw_priv *priv)
{
	__raw_writel(0, &priv->ss_regs->tx_en);
	__raw_writel(0, &priv->ss_regs->rx_en);

	cpdma_ctlr_int_ctrl(priv->dma, false);
	return;
}

void cpsw_tx_handler(void *token, int len, int status)
{
	struct sk_buff		*skb = token;
	struct net_device	*ndev = skb->dev;
	struct cpsw_priv	*priv = netdev_priv(ndev);

	if (unlikely(netif_queue_stopped(ndev)))
		netif_start_queue(ndev);
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += len;
	dev_kfree_skb_any(skb);
}

void cpsw_rx_handler(void *token, int len, int status)
{
	struct sk_buff		*skb = token;
	struct net_device	*ndev = skb->dev;
	struct cpsw_priv	*priv = netdev_priv(ndev);
	int			ret = 0;

	cpsw_dual_emac_source_port_detect(status, priv, ndev, skb);

	if (unlikely(!netif_running(ndev)) ||
			unlikely(!netif_carrier_ok(ndev))) {
		dev_kfree_skb_any(skb);
		return;
	}

	if (likely(status >= 0)) {
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, ndev);
		netif_receive_skb(skb);
		priv->stats.rx_bytes += len;
		priv->stats.rx_packets++;
		skb = NULL;
	}


	if (unlikely(!netif_running(ndev))) {
		if (skb)
			dev_kfree_skb_any(skb);
		return;
	}

	if (likely(!skb)) {
		skb = netdev_alloc_skb_ip_align(ndev, priv->rx_packet_max);
		if (WARN_ON(!skb))
			return;

		ret = cpdma_chan_submit(priv->rxch, skb, skb->data,
				skb_tailroom(skb), 0, GFP_KERNEL);
	}

	WARN_ON(ret < 0);

}

static void set_cpsw_dmtimer_clear(void)
{
	omap_dm_timer_write_status(dmtimer_rx, OMAP_TIMER_INT_CAPTURE);
	omap_dm_timer_write_status(dmtimer_tx, OMAP_TIMER_INT_CAPTURE);

	return;
}

static irqreturn_t cpsw_interrupt(int irq, void *dev_id)
{
	struct cpsw_priv *priv = dev_id;

	if (likely(netif_running(priv->ndev))) {
		cpsw_intr_disable(priv);
		cpsw_disable_irq(priv);
		napi_schedule(&priv->napi);
	} else {
		priv = cpsw_get_slave_priv(priv, 1);
		if (likely(priv) && likely(netif_running(priv->ndev))) {
			cpsw_intr_disable(priv);
			cpsw_disable_irq(priv);
			napi_schedule(&priv->napi);
		}
	}

	return IRQ_HANDLED;
}

static int cpsw_poll(struct napi_struct *napi, int budget)
{
	struct cpsw_priv	*priv = napi_to_priv(napi);
	int			num_tx, num_rx;

	num_tx = cpdma_chan_process(priv->txch, 128);
	num_rx = cpdma_chan_process(priv->rxch, budget);

	if (num_rx || num_tx)
		msg(dbg, intr, "poll %d rx, %d tx pkts\n", num_rx, num_tx);

	if (num_rx < budget) {
		napi_complete(napi);
		cpdma_ctlr_eoi(priv->dma);
		set_cpsw_dmtimer_clear();
		cpsw_intr_enable(priv);
		cpsw_enable_irq(priv);
	}

	return num_rx;
}

static inline void soft_reset(const char *module, void __iomem *reg)
{
	unsigned long timeout = jiffies + HZ;

	__raw_writel(1, reg);
	do {
		cpu_relax();
	} while ((__raw_readl(reg) & 1) && time_after(timeout, jiffies));

	WARN(__raw_readl(reg) & 1, "failed to soft-reset %s\n", module);
}

#define mac_hi(mac)	(((mac)[0] << 0) | ((mac)[1] << 8) |	\
			 ((mac)[2] << 16) | ((mac)[3] << 24))
#define mac_lo(mac)	(((mac)[4] << 0) | ((mac)[5] << 8))

static void cpsw_set_slave_mac(struct cpsw_slave *slave,
			       struct cpsw_priv *priv)
{
	__raw_writel(mac_hi(priv->mac_addr), &slave->regs->sa_hi);
	__raw_writel(mac_lo(priv->mac_addr), &slave->regs->sa_lo);
}

static inline u32 cpsw_get_slave_port(struct cpsw_priv *priv, u32 slave_num)
{
	if (priv->host_port == 0)
		return slave_num + 1;
	else
		return slave_num;
}

static void _cpsw_adjust_link(struct cpsw_slave *slave,
			      struct cpsw_priv *priv, bool *link)
{
	struct phy_device	*phy = slave->phy;
	u32			mac_control = 0;
	u32			slave_port;

	if (!phy)
		return;

	slave_port = cpsw_get_slave_port(priv, slave->slave_num);

	if (phy->link) {
		/* enable forwarding */
		cpsw_ale_control_set(priv->ale, slave_port,
			ALE_PORT_STATE, priv->port_state[slave_port]);

		mac_control = priv->data.mac_control;
		if (phy->speed == 10)
			mac_control |= BIT(18); /* In Band mode */
		if (phy->speed == 1000) {
			mac_control |= BIT(7);	/* Enable gigabit mode */
		}
		if (phy->speed == 100)
			mac_control |= BIT(15);
		if (phy->duplex)
			mac_control |= BIT(0);	/* FULLDUPLEXEN	*/
		if (phy->interface == PHY_INTERFACE_MODE_RGMII) /* RGMII */
			mac_control |= (BIT(15)|BIT(16));
		*link = true;
	} else {
		cpsw_ale_control_set(priv->ale, slave_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);
		mac_control = 0;
	}

	if (mac_control != slave->mac_control) {
		phy_print_status(phy);
		__raw_writel(mac_control, &slave->sliver->mac_control);
	}

	slave->mac_control = mac_control;
}

static void cpsw_adjust_link(struct net_device *ndev)
{
	struct cpsw_priv	*priv = netdev_priv(ndev);
	bool			link = false;

	for_each_slave(priv, _cpsw_adjust_link, priv, &link);

	if (link) {
		netif_carrier_on(ndev);
		if (netif_running(ndev))
			netif_wake_queue(ndev);
	} else {
		netif_carrier_off(ndev);
		netif_stop_queue(ndev);
	}
}

static inline int __show_stat(char *buf, int maxlen, const char* name, u32 val)
{
	static char *leader = "........................................";

	if (!val)
		return 0;
	else
		return snprintf(buf, maxlen, "%s %s %10d\n", name,
				leader + strlen(name), val);
}

static ssize_t cpsw_hw_stats_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct net_device	*ndev = to_net_dev(dev);
	struct cpsw_priv	*priv = netdev_priv(ndev);
	int			len = 0;
	struct cpdma_chan_stats	dma_stats;

#define show_stat(x) do {						\
	len += __show_stat(buf + len, SZ_4K - len, #x,			\
			   __raw_readl(&priv->hw_stats->x));		\
} while (0)

#define show_dma_stat(x) do {						\
	len += __show_stat(buf + len, SZ_4K - len, #x, dma_stats.x);	\
} while (0)

	len += snprintf(buf + len, SZ_4K - len, "CPSW Statistics:\n");
	show_stat(rxgoodframes);	show_stat(rxbroadcastframes);
	show_stat(rxmulticastframes);	show_stat(rxpauseframes);
	show_stat(rxcrcerrors);		show_stat(rxaligncodeerrors);
	show_stat(rxoversizedframes);	show_stat(rxjabberframes);
	show_stat(rxundersizedframes);	show_stat(rxfragments);
	show_stat(rxoctets);		show_stat(txgoodframes);
	show_stat(txbroadcastframes);	show_stat(txmulticastframes);
	show_stat(txpauseframes);	show_stat(txdeferredframes);
	show_stat(txcollisionframes);	show_stat(txsinglecollframes);
	show_stat(txmultcollframes);	show_stat(txexcessivecollisions);
	show_stat(txlatecollisions);	show_stat(txunderrun);
	show_stat(txcarriersenseerrors); show_stat(txoctets);
	show_stat(octetframes64);	show_stat(octetframes65t127);
	show_stat(octetframes128t255);	show_stat(octetframes256t511);
	show_stat(octetframes512t1023);	show_stat(octetframes1024tup);
	show_stat(netoctets);		show_stat(rxsofoverruns);
	show_stat(rxmofoverruns);	show_stat(rxdmaoverruns);

	cpdma_chan_get_stats(priv->rxch, &dma_stats);
	len += snprintf(buf + len, SZ_4K - len, "\nRX DMA Statistics:\n");
	show_dma_stat(head_enqueue);	show_dma_stat(tail_enqueue);
	show_dma_stat(pad_enqueue);	show_dma_stat(misqueued);
	show_dma_stat(desc_alloc_fail);	show_dma_stat(pad_alloc_fail);
	show_dma_stat(runt_receive_buff); show_dma_stat(runt_transmit_buff);
	show_dma_stat(empty_dequeue);	show_dma_stat(busy_dequeue);
	show_dma_stat(good_dequeue);	show_dma_stat(teardown_dequeue);

	cpdma_chan_get_stats(priv->txch, &dma_stats);
	len += snprintf(buf + len, SZ_4K - len, "\nTX DMA Statistics:\n");
	show_dma_stat(head_enqueue);	show_dma_stat(tail_enqueue);
	show_dma_stat(pad_enqueue);	show_dma_stat(misqueued);
	show_dma_stat(desc_alloc_fail);	show_dma_stat(pad_alloc_fail);
	show_dma_stat(runt_receive_buff); show_dma_stat(runt_transmit_buff);
	show_dma_stat(empty_dequeue);	show_dma_stat(busy_dequeue);
	show_dma_stat(good_dequeue);	show_dma_stat(teardown_dequeue);

	return len;
}

DEVICE_ATTR(hw_stats, S_IRUGO, cpsw_hw_stats_show, NULL);

#define PHY_CONFIG_REG	22
static void cpsw_set_phy_config(struct cpsw_priv *priv, struct phy_device *phy)
{
	struct cpsw_platform_data *pdata = priv->pdev->dev.platform_data;
	struct mii_bus *miibus;
	int phy_addr = 0;
	u16 val = 0;
	u16 tmp = 0;

	if (!phy)
		return;

	miibus = phy->bus;

	if (!miibus)
		return;

	phy_addr = phy->addr;

	/* Disable 1 Gig mode support if it is not supported */
	if (!pdata->gigabit_en)
		phy->supported &= ~(SUPPORTED_1000baseT_Half |
					SUPPORTED_1000baseT_Full);

	/* Following lines enable gigbit advertisement capability even in case
	 * the advertisement is not enabled by default
	 */
	val = miibus->read(miibus, phy_addr, MII_BMCR);
	val |= (BMCR_SPEED100 | BMCR_ANENABLE | BMCR_FULLDPLX);
	miibus->write(miibus, phy_addr, MII_BMCR, val);
	tmp = miibus->read(miibus, phy_addr, MII_BMCR);

	/* Enable gigabit support only if the speed is 1000Mbps */
	if (phy->speed == CPSW_PHY_SPEED) {
		tmp = miibus->read(miibus, phy_addr, MII_BMSR);
		if (tmp & 0x1) {
			val = miibus->read(miibus, phy_addr, MII_CTRL1000);
			val |= BIT(9);
			miibus->write(miibus, phy_addr, MII_CTRL1000, val);
			tmp = miibus->read(miibus, phy_addr, MII_CTRL1000);
		}
	}

	val = miibus->read(miibus, phy_addr, MII_ADVERTISE);
	val |= (ADVERTISE_10HALF | ADVERTISE_10FULL | \
		ADVERTISE_100HALF | ADVERTISE_100FULL);
	miibus->write(miibus, phy_addr, MII_ADVERTISE, val);
	tmp = miibus->read(miibus, phy_addr, MII_ADVERTISE);

	/* TODO : This check is required. This should be
	 * moved to a board init section as its specific
	 * to a phy.*/
	if (phy->phy_id == 0x0282F014) {
		/* This enables TX_CLK-ing in case of 10/100MBps operation */
		val = miibus->read(miibus, phy_addr, PHY_CONFIG_REG);
		val |= BIT(5);
		miibus->write(miibus, phy_addr, PHY_CONFIG_REG, val);
		tmp = miibus->read(miibus, phy_addr, PHY_CONFIG_REG);
	}

	return;
}

static void cpsw_slave_open(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	char name[32];
	u32 slave_port;

	sprintf(name, "slave-%d", slave->slave_num);

	soft_reset(name, &slave->sliver->soft_reset);

	/* setup priority mapping */
	__raw_writel(0x76543210, &slave->sliver->rx_pri_map);
	__raw_writel(0x33221100, &slave->regs->tx_pri_map);

	/* setup max packet size, and mac address */
	__raw_writel(priv->rx_packet_max, &slave->sliver->rx_maxlen);
	cpsw_set_slave_mac(slave, priv);

	slave->mac_control = 0;	/* no link yet */

	slave_port = cpsw_get_slave_port(priv, slave->slave_num);

	cpsw_add_dual_emac_mode_default_ale_entries(priv, slave, slave_port);
	cpsw_add_switch_mode_bcast_ale_entries(priv, slave_port);
	priv->port_state[slave_port] = ALE_PORT_STATE_FORWARD;

	slave->phy = phy_connect(priv->ndev, slave->data->phy_id,
				 &cpsw_adjust_link, 0, slave->data->phy_if);
	if (IS_ERR(slave->phy)) {
		msg(err, ifup, "phy %s not found on slave %d\n",
		    slave->data->phy_id, slave->slave_num);
		slave->phy = NULL;
	} else {
		dev_info(priv->dev, "CPSW phy found : id is : 0x%x\n",
			slave->phy->phy_id);
		cpsw_set_phy_config(priv, slave->phy);
		phy_start(slave->phy);
	}
}

#if defined(VLAN_SUPPORT) && !defined(CONFIG_TI_CPSW_DUAL_EMAC)
static inline void cpsw_add_default_vlan(struct cpsw_priv *priv)
{
	writel(priv->data.default_vlan, &priv->host_port_regs->port_vlan);
	writel(priv->data.default_vlan, &priv->slaves[0].regs->port_vlan);
	writel(priv->data.default_vlan, &priv->slaves[1].regs->port_vlan);
	cpsw_ale_add_vlan(priv->ale, priv->data.default_vlan,
			ALE_ALL_PORTS << priv->host_port,
			ALE_ALL_PORTS << priv->host_port,
			ALE_ALL_PORTS << priv->host_port, 0);
}
#else
#define cpsw_add_default_vlan(priv)
#endif

static void cpsw_init_host_port(struct cpsw_priv *priv)
{
	u32 control_reg;

	/* soft reset the controller and initialize ale */
	soft_reset("cpsw", &priv->regs->soft_reset);
	cpsw_ale_start(priv->ale);

	/* switch to vlan aware/unaware mode */
	cpsw_ale_control_set(priv->ale, priv->host_port, ALE_VLAN_AWARE,
			     CPSW_ALE_VLAN_AWARE);
	control_reg = readl(&priv->regs->control);
	control_reg |= CPSW_VLAN_AWARE;
	writel(control_reg, &priv->regs->control);
	cpsw_p0_fifo_type_select(priv);

	/* setup host port priority mapping */
	__raw_writel(0x76543210, &priv->host_port_regs->cpdma_tx_pri_map);
	__raw_writel(0, &priv->host_port_regs->cpdma_rx_chan_map);

	cpsw_ale_control_set(priv->ale, priv->host_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_add_switch_mode_default_ale_entries(priv);
}

static int cpsw_ndo_open(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	int i, ret;
	u32 reg;

	if (!cpsw_common_res_usage_state(priv))
		cpsw_intr_disable(priv);
	netif_carrier_off(ndev);

	if (priv->data.phy_control)
		(*priv->data.phy_control)(true);

	reg = __raw_readl(&priv->regs->id_ver);

	msg(info, ifup, "initializing cpsw version %d.%d (%d)\n",
	    (reg >> 8 & 0x7), reg & 0xff, (reg >> 11) & 0x1f);

	/* initialize host and slave ports */
	if (!cpsw_common_res_usage_state(priv))
		cpsw_init_host_port(priv);
	for_each_slave(priv, cpsw_slave_open, priv);

	/* Add default VLAN */
	cpsw_add_default_vlan(priv);

	if (!cpsw_common_res_usage_state(priv)) {
		ret = device_create_file(&ndev->dev, &dev_attr_hw_stats);
		if (ret < 0) {
			dev_err(priv->dev, "unable to add device attr\n");
			return ret;
		}

		/* setup tx dma to fixed prio and zero offset */
		cpdma_control_set(priv->dma, CPDMA_TX_PRIO_FIXED, 1);
		cpdma_control_set(priv->dma, CPDMA_RX_BUFFER_OFFSET, 0);

		/* disable priority elevation and enable statistics */
		/* on all ports */
		writel(0, &priv->regs->ptype);
		writel(0x7, &priv->regs->stat_port_en);

		if (WARN_ON(!priv->data.rx_descs))
			priv->data.rx_descs = 128;

		for (i = 0; i < priv->data.rx_descs; i++) {
			struct sk_buff *skb;

			ret = -ENOMEM;
			skb = netdev_alloc_skb_ip_align(priv->ndev,
							priv->rx_packet_max);
			if (!skb)
				break;
			ret = cpdma_chan_submit(priv->rxch, skb, skb->data,
					skb_tailroom(skb), 0, GFP_KERNEL);
			if (WARN_ON(ret < 0)) {
				dev_kfree_skb_any(skb);
				break;
			}
		}
		/*
		 * continue even if we didn't manage to submit all
		 * receive descs
		 */
		msg(info, ifup, "submitted %d rx descriptors\n", i);
	}

	/* Enable Interrupt pacing if configured */
	if (priv->coal_intvl != 0) {
		struct ethtool_coalesce coal;

		coal.rx_coalesce_usecs = (priv->coal_intvl << 4);
		cpsw_set_coalesce(ndev, &coal);
	}

	/* Enable Timer for capturing cpsw rx interrupts */
	omap_dm_timer_set_int_enable(dmtimer_rx, OMAP_TIMER_INT_CAPTURE);
	omap_dm_timer_set_capture(dmtimer_rx, 1, 0, 0);
	omap_dm_timer_enable(dmtimer_rx);

	/* Enable Timer for capturing cpsw tx interrupts */
	omap_dm_timer_set_int_enable(dmtimer_tx, OMAP_TIMER_INT_CAPTURE);
	omap_dm_timer_set_capture(dmtimer_tx, 1, 0, 0);
	omap_dm_timer_enable(dmtimer_tx);

	cpdma_ctlr_start(priv->dma);
	cpsw_intr_enable(priv);
	napi_enable(&priv->napi);
	cpdma_ctlr_eoi(priv->dma);

	cpsw_update_slave_open_state(priv, true)

	return 0;
}

static void cpsw_slave_stop(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	if (!slave->phy)
		return;
	phy_stop(slave->phy);
	phy_disconnect(slave->phy);
	slave->phy = NULL;
}

static int cpsw_ndo_stop(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	msg(info, ifdown, "shutting down cpsw device\n");

	netif_stop_queue(priv->ndev);
	napi_disable(&priv->napi);
	netif_carrier_off(priv->ndev);

	if (cpsw_common_res_usage_state(priv) <= 1) {
		cpsw_intr_disable(priv);
		cpdma_ctlr_int_ctrl(priv->dma, false);

		omap_dm_timer_set_int_enable(dmtimer_rx, 0);
		omap_dm_timer_set_int_enable(dmtimer_tx, 0);

		cpdma_ctlr_stop(priv->dma);
		cpsw_ale_stop(priv->ale);
	}

	device_remove_file(&ndev->dev, &dev_attr_hw_stats);
	for_each_slave(priv, cpsw_slave_stop, priv);
	if (priv->data.phy_control)
		(*priv->data.phy_control)(false);

	cpsw_update_slave_open_state(priv, false)

	return 0;
}

static netdev_tx_t cpsw_ndo_start_xmit(struct sk_buff *skb,
				       struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	int ret;

	ndev->trans_start = jiffies;

	ret = skb_padto(skb, CPSW_MIN_PACKET_SIZE);
	if (unlikely(ret < 0)) {
		msg(err, tx_err, "packet pad failed");
		goto fail;
	}

	ret = cpsw_tx_packet_submit(ndev, priv, skb);
	if (unlikely(ret != 0)) {
		msg(err, tx_err, "desc submit failed");
		goto fail;
	}

	return NETDEV_TX_OK;
fail:
	priv->stats.tx_dropped++;
	netif_stop_queue(ndev);
	return NETDEV_TX_BUSY;
}

#ifndef CONFIG_TI_CPSW_DUAL_EMAC
/*
 *  cpsw_set_priority_mapping
 *    This function configures the packet priority handling on each port.
 */
static int cpsw_set_priority_mapping(struct cpsw_priv *priv, int port,
	int rxpriority, int txpriority, int switchpriority)
{
	if (port == 0) {
		writel(CPSW_PRIMAP(rxpriority, txpriority),
			&priv->host_port_regs->cpdma_tx_pri_map);
		writel(CPSW_PRIMAP(txpriority, switchpriority),
			&priv->host_port_regs->cpdma_tx_pri_map);
	} else {
		/* Configure sliver priority mapping registers */
		writel(CPSW_PRIMAP(rxpriority, txpriority),
			&slave(priv, port-1)->sliver->rx_pri_map);
		writel(CPSW_PRIMAP(txpriority, switchpriority),
			&slave(priv, port-1)->sliver->rx_pri_map);
	}
	return 0;
}

/*
 * cpsw_rate_limit_tx
 * Limits the number of Tx packets on a port for a 100ms time period.
 * Rate limit can be done either on Tx or Rx for all the ports due to
 * hardware limitation
 */
static int cpsw_rate_limit_tx(struct cpsw_ale *ale, u32 enable, u32 port,
				u32 addr_type, u32 limit)
{
	u32 type;

	if (!enable) {
		cpsw_ale_control_set(ale, port, ALE_RATE_LIMIT, 0);
		return 0;
	}

	/* ALE prescale is set for 100 ms time */
	if (addr_type == ADDR_TYPE_BROADCAST)
		type = ALE_PORT_BCAST_LIMIT;
	else if (addr_type == ADDR_TYPE_MULTICAST)
		type = ALE_PORT_MCAST_LIMIT;
	else
		return -EFAULT;

	cpsw_ale_control_set(ale, port, type, limit);
	/* Enable the bit for rate limiting transmission */
	cpsw_ale_control_set(ale, port, ALE_RATE_LIMIT_TX, 1);
	cpsw_ale_control_set(ale, port, ALE_RATE_LIMIT, 1);

	return 0;
}

/*
 * cpsw_rate_limit_rx
 * Limits the number of Rx packets on a port for a 100ms time period.
 * Rate limit can be done either on Tx or Rx for all the ports due to
 * hardware limitation
 */
static int cpsw_rate_limit_rx(struct cpsw_ale *ale, u32 enable, u32 port,
				u32 addr_type, u32 limit)
{
	u32 type;

	if (!enable) {
		cpsw_ale_control_set(ale, port, ALE_RATE_LIMIT, 0);
		return 0;
	}

	/* ALE prescale is set for 100 ms time */
	if (addr_type == ADDR_TYPE_BROADCAST)
		type = ALE_PORT_BCAST_LIMIT;
	else if (addr_type == ADDR_TYPE_MULTICAST)
		type = ALE_PORT_MCAST_LIMIT;
	else
		return -EFAULT;

	cpsw_ale_control_set(ale, port, type, limit);
	/* Disable the bit for rate limiting reception */
	cpsw_ale_control_set(ale, port, ALE_RATE_LIMIT_TX, 0);
	cpsw_ale_control_set(ale, port, ALE_RATE_LIMIT, 1);

	return 0;
}

int cpsw_rate_limit(struct cpsw_ale *ale, u32 enable, u32 direction, u32 port,
			u32 packet_type, u32 limit)
{

	if (port == 0) {
		/*
		 * For host port transmit/receive terminlogy is inverse
		 * of cpgmac port
		 */
		if (direction)
			cpsw_rate_limit_rx(ale, enable, port,
					packet_type, limit);
		else
			cpsw_rate_limit_tx(ale, enable, port,
					packet_type, limit);
	} else {
		if (direction)
			cpsw_rate_limit_tx(ale, enable, port,
					packet_type, limit);
		else
			cpsw_rate_limit_rx(ale, enable, port,
					packet_type, limit);
	}
	return 0;
}

static int cpsw_config_dump(struct cpsw_priv *priv, u8 *buf, u32 size)
{
	int vlan_aware = 0;
	int out_len = 0;
	u32 port_state;
	char *port_state_str[] = {"disabled", "blocked", "learn", "forward"};

	out_len += snprintf(buf, size, "Switch Configuarion\n");

	vlan_aware = readl(&priv->regs->control) & CPSW_VLAN_AWARE;

	out_len += snprintf(buf + out_len, size - out_len, "VLAN Aware Mode"
			"                              :  %s\n",
			(vlan_aware ? "Yes" : "No"));
	out_len += snprintf(buf + out_len, size - out_len, "ALE VLAN Aware "
			"Mode                          :  %s\n",
			(cpsw_ale_control_get(priv->ale, priv->host_port,
				ALE_VLAN_AWARE) ? "Yes" : "No"));
	out_len += snprintf(buf + out_len, size - out_len, "Unknown VLAN "
			"Members                         :  %u\n",
			cpsw_ale_control_get(priv->ale, 0,
				ALE_PORT_UNKNOWN_VLAN_MEMBER));
	out_len += snprintf(buf + out_len, size - out_len, "Unknown Multicast"
			" Flood Mask                 :  %u\n",
			cpsw_ale_control_get(priv->ale, 0,
				ALE_PORT_UNKNOWN_MCAST_FLOOD));
	out_len += snprintf(buf + out_len, size - out_len, "Unknown Registered"
			" Multicast Flood Mask      :  %u\n",
			cpsw_ale_control_get(priv->ale, 0,
				ALE_PORT_UNKNOWN_REG_MCAST_FLOOD));
	out_len += snprintf(buf + out_len, size - out_len, "Unknown VLAN Force"
			" Untagged Egress           :  %u\n",
			cpsw_ale_control_get(priv->ale, 0,
				ALE_PORT_UNTAGGED_EGRESS));

	out_len += snprintf(buf + out_len, size - out_len,
			"\nPort Configuration\n");
	out_len += snprintf(buf + out_len, size - out_len,
			"\t%-8s %-8s %-8s %s\n", "PORT", "PVID", "PRIORITY",
			"STATE");
	out_len += snprintf(buf + out_len, size - out_len,
			"\t---------------------------------\n");

	if (vlan_aware) {
		int port_vlan;

		port_state = cpsw_ale_control_get(priv->ale, 0, ALE_PORT_STATE);
		port_vlan = readl(&priv->host_port_regs->port_vlan);
		out_len += snprintf(buf + out_len, size - out_len,
			"\t%-8u %-8u %-8u %s\n", 0,
			port_vlan & 0xfff, (port_vlan > 13) & 0x7,
			port_state_str[port_state]);

		port_state = cpsw_ale_control_get(priv->ale, 1, ALE_PORT_STATE);
		port_vlan = readl(&priv->slaves[0].regs->port_vlan);
		out_len += snprintf(buf + out_len, size - out_len,
			"\t%-8u %-8u %-8u %s\n", 1,
			port_vlan & 0xfff, (port_vlan > 13) & 0x7,
			port_state_str[port_state]);

		port_state = cpsw_ale_control_get(priv->ale, 2, ALE_PORT_STATE);
		port_vlan = readl(&priv->slaves[1].regs->port_vlan);
		out_len += snprintf(buf + out_len, size - out_len,
			"\t%-8u %-8u %-8u %s\n", 2,
			port_vlan & 0xfff, (port_vlan > 13) & 0x7,
			port_state_str[port_state]);
	} else {
		port_state = cpsw_ale_control_get(priv->ale, 0, ALE_PORT_STATE);
		out_len += snprintf(buf + out_len, size - out_len,
			"\t%-8u %-8s %-8s %s\n", 0,
			"-", "-", port_state_str[port_state]);

		port_state = cpsw_ale_control_get(priv->ale, 1, ALE_PORT_STATE);
		out_len += snprintf(buf + out_len, size - out_len,
			"\t%-8u %-8s %-8s %s\n", 1,
			"-", "-", port_state_str[port_state]);

		port_state = cpsw_ale_control_get(priv->ale, 2, ALE_PORT_STATE);
		out_len += snprintf(buf + out_len, size - out_len,
			"\t%-8u %-8s %-8s %s\n", 2,
			"-", "-", port_state_str[port_state]);
	}

	return out_len;
}

static int cpsw_set_port_state(struct cpsw_priv *priv, int port,
			int port_state)
{
	int ret = -EFAULT;
	switch (port_state) {
	case PORT_STATE_DISABLED:
		priv->port_state[port] = ALE_PORT_STATE_DISABLE;
		break;
	case PORT_STATE_BLOCKED:
		priv->port_state[port] = ALE_PORT_STATE_BLOCK;
		break;
	case PORT_STATE_LEARN:
		priv->port_state[port] = ALE_PORT_STATE_LEARN;
		break;
	case PORT_STATE_FORWARD:
		priv->port_state[port] = ALE_PORT_STATE_FORWARD;
		break;
	}
	ret = cpsw_ale_control_set(priv->ale, port, ALE_PORT_STATE,
			priv->port_state[port]);

	return ret;
}

static int cpsw_switch_config_ioctl(struct net_device *ndev,
		struct ifreq *ifrq, int cmd)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	struct net_switch_config *switch_config;
	int ret = -EFAULT;

	/*
	* Only SIOCSWITCHCONFIG is used as cmd argument and hence, there is no
	* switch statement required.
	* Function calls are based on switch_config.cmd
	*/
	if (cmd != SIOCSWITCHCONFIG)
		return ret;

	switch_config = kzalloc(sizeof(struct net_switch_config), GFP_KERNEL);
	if (copy_from_user(switch_config, (ifrq->ifr_data),
			sizeof(struct net_switch_config)))
		return ret;

	switch (switch_config->cmd) {
	case CONFIG_SWITCH_ADD_MULTICAST:
		if ((switchcmd(switch_config).untag_port <= 3) &&
				(switchcmd(switch_config).vid <= 4095) &&
				(switchcmd(switch_config).mem_port > 0) &&
				(switchcmd(switch_config).mem_port <= 7) &&
				is_multicast_ether_addr(
				switchcmd(switch_config).addr)) {
			if (switchcmd(switch_config).vid == 0)
				ret = cpsw_ale_add_mcast(priv->ale,
					switchcmd(switch_config).addr,
					switchcmd(switch_config).mem_port
						<< priv->host_port,
					switchcmd(switch_config).flag,
					switchcmd(switch_config).untag_port);
			else
				ret = cpsw_ale_vlan_add_mcast(priv->ale,
					switchcmd(switch_config).addr,
					switchcmd(switch_config).mem_port
						<< priv->host_port,
					switchcmd(switch_config).vid,
					switchcmd(switch_config).flag,
					switchcmd(switch_config).untag_port);
		} else {
			dev_err(priv->dev, "Invalid Switch config arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_ADD_UNICAST:
		if ((switchcmd(switch_config).vid <= 4095) &&
				(switchcmd(switch_config).mem_port <= 2) &&
				is_valid_ether_addr(
					switchcmd(switch_config).addr)) {
			if (switchcmd(switch_config).vid == 0)
				ret = cpsw_ale_add_ucast(priv->ale,
					switchcmd(switch_config).addr,
					priv->host_port, 0);
			else
				ret = cpsw_ale_vlan_add_ucast(priv->ale,
					switchcmd(switch_config).addr,
					priv->host_port, 0,
					switchcmd(switch_config).vid);
		} else {
			dev_err(priv->dev, "Invalid Switch config arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_ADD_OUI:
		if (!is_zero_ether_addr(switchcmd(switch_config).addr)) {
			ret = cpsw_ale_add_oui(priv->ale,
				switchcmd(switch_config).addr);
		} else {
			dev_err(priv->dev, "Invalid Switch config arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_FIND_ADDR:
		if ((switchcmd(switch_config).vid <= 4095) &&
			!is_zero_ether_addr(switchcmd(switch_config).addr)) {
			ret = cpsw_ale_match_addr(priv->ale,
				switchcmd(switch_config).addr,
				switchcmd(switch_config).vid);
			if (ret >= 0) {
				switch_config->ret_type = ret;
				ret = copy_to_user(ifrq->ifr_data,
					switch_config,
					sizeof(struct net_switch_config)) ?
					-EFAULT : 0;
			}
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_DEL_MULTICAST:
		if ((switchcmd(switch_config).vid <= 4095) &&
				is_multicast_ether_addr(
				switchcmd(switch_config).addr)) {
			if (switchcmd(switch_config).vid == 0)
				ret = cpsw_ale_del_mcast(priv->ale,
					switchcmd(switch_config).addr,
					switchcmd(switch_config).mem_port
						<< priv->host_port);
			else
				ret = cpsw_ale_vlan_del_mcast(priv->ale,
					switchcmd(switch_config).addr,
					switchcmd(switch_config).mem_port
						<< priv->host_port,
					switchcmd(switch_config).vid);
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_DEL_UNICAST:
		if ((switchcmd(switch_config).vid <= 4095) &&
			is_valid_ether_addr(switchcmd(switch_config).addr)) {
			if (switchcmd(switch_config).vid == 0) {
				if (!memcmp(switchcmd(switch_config).addr,
						priv->mac_addr, 6)) {
					ret = -EPERM;
					break;
				}
				ret = cpsw_ale_del_ucast(priv->ale,
					switchcmd(switch_config).addr, 0);
			} else
				ret = cpsw_ale_del_ucast(priv->ale,
					switchcmd(switch_config).addr,
					switchcmd(switch_config).vid);
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_ADD_VLAN:
		if ((switchcmd(switch_config).vid <= 4095) &&
				(switchcmd(switch_config).mem_port > 0) &&
				(switchcmd(switch_config).mem_port <= 7)) {
			ret = cpsw_ale_add_vlan(priv->ale,
				switchcmd(switch_config).vid,
				switchcmd(switch_config).mem_port
					<< priv->host_port,
				switchcmd(switch_config).untag_port,
				switchcmd(switch_config).reg_multi,
				switchcmd(switch_config).unreg_multi);
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_FIND_VLAN:
		if (switchcmd(switch_config).vid <= 4095) {
			switch_config->ret_type = cpsw_ale_match_vlan(priv->ale,
				switchcmd(switch_config).vid);
			ret = copy_to_user(ifrq->ifr_data, switch_config,
				sizeof(struct net_switch_config)) ?
				-EFAULT : 0;
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_DEL_VLAN:
		if ((switchcmd(switch_config).vid <= 4095) &&
				(switchcmd(switch_config).mem_port <= 7)) {
			ret = cpsw_ale_del_vlan(priv->ale,
				switchcmd(switch_config).vid,
				switchcmd(switch_config).mem_port
				<< priv->host_port);
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_SET_PORT_VLAN_CONFIG:
	{
		int port_vlan = switchcmd(switch_config).vid |
				switchcmd(switch_config).prio_port << 13;

		if ((switchcmd(switch_config).vid <= 4095) &&
				(switchcmd(switch_config).port <= 2) &&
				(switchcmd(switch_config).prio_port <= 7)) {
			if (switchcmd(switch_config).CFI_port)
				port_vlan |= (1 << 12);

			if (switchcmd(switch_config).port == 0)
				writel(port_vlan,
					&priv->host_port_regs->port_vlan);
			else if (switchcmd(switch_config).port == 1)
				writel(port_vlan,
					&(priv->slaves[0].regs->port_vlan));
			else
				writel(port_vlan,
					&(priv->slaves[1].regs->port_vlan));
			ret = 0;
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;
	}

	case CONFIG_SWITCH_TIMEOUT:
		ret = cpsw_ale_set_ageout(priv->ale,
			switchcmd(switch_config).ale_timeout);
		break;

	case CONFIG_SWITCH_DUMP:
		ret = cpsw_ale_dump(priv->ale,
			switchcmd(switch_config).aledump,
			switch_config->cmd_data.buf, 4095);
		if (ret)
			ret = copy_to_user(ifrq->ifr_data, switch_config,
				sizeof(struct net_switch_config)) ?
				-EFAULT : 0;
		break;

	case CONFIG_SWITCH_SET_FLOW_CONTROL:
		if (portcmd(switch_config).port <= 7) {
			writel(portcmd(switch_config).port,
				&priv->regs->flow_control);
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_SET_PRIORITY_MAPPING:
		if ((priocmd(switch_config).port <= 2) &&
			(priocmd(switch_config).prio_rx <= 7) &&
			(priocmd(switch_config).prio_tx <= 7) &&
			(priocmd(switch_config).prio_switch <= 3)) {
			ret = cpsw_set_priority_mapping(priv,
				priocmd(switch_config).port,
				priocmd(switch_config).prio_rx,
				priocmd(switch_config).prio_tx,
				priocmd(switch_config).prio_switch);
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_PORT_STATISTICS_ENABLE:
		if (portcmd(switch_config).port <= 7) {
			writel(portcmd(switch_config).port,
				&priv->regs->stat_port_en);
			ret = 0;
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_CONFIG_DUMP:
		cpsw_config_dump(priv, switch_config->cmd_data.buf, 4096);
		ret = copy_to_user(ifrq->ifr_data, switch_config,
			sizeof(struct net_switch_config)) ?
			-EFAULT : 0;
		break;

	case CONFIG_SWITCH_RATELIMIT:
		if ((portcmd(switch_config).port <= 2) &&
				(portcmd(switch_config).enable == 0))
			ret = cpsw_ale_control_set(priv->ale,
				portcmd(switch_config).port,
				ALE_RATE_LIMIT, 0);
		else if ((portcmd(switch_config).port <= 2) &&
				((portcmd(switch_config).addr_type
					== ADDR_TYPE_BROADCAST) ||
				(portcmd(switch_config).addr_type
					== ADDR_TYPE_MULTICAST)) &&
				(portcmd(switch_config).limit <= 255)) {
			ret = cpsw_rate_limit(priv->ale,
				portcmd(switch_config).enable,
				portcmd(switch_config).direction,
				portcmd(switch_config).port,
				portcmd(switch_config).addr_type,
				portcmd(switch_config).limit);
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_VID_INGRESS_CHECK:
		if (portcmd(switch_config).port <= 2) {
			cpsw_ale_control_set(priv->ale,
				portcmd(switch_config).port,
				ALE_PORT_DROP_UNTAGGED, 1);
			cpsw_ale_control_set(priv->ale,
				portcmd(switch_config).port,
				ALE_PORT_DROP_UNKNOWN_VLAN, 1);
			ret = 0;
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_ADD_UNKNOWN_VLAN_INFO:
		if ((portcmd(switch_config).port <= 7) &&
				(portcmd(switch_config).
					reg_multi_port_mask <= 7) &&
				(portcmd(switch_config).
					unknown_reg_multi_port_mask <= 7) &&
				(portcmd(switch_config).
					unknown_vlan_member <= 7)) {
			cpsw_ale_control_set(priv->ale, 0,
				ALE_PORT_UNTAGGED_EGRESS,
				portcmd(switch_config).port);
			cpsw_ale_control_set(priv->ale, 0,
				ALE_PORT_UNKNOWN_REG_MCAST_FLOOD,
				portcmd(switch_config).reg_multi_port_mask);
			cpsw_ale_control_set(priv->ale, 0,
				ALE_PORT_UNKNOWN_MCAST_FLOOD,
				portcmd(switch_config).
					unknown_reg_multi_port_mask);
			cpsw_ale_control_set(priv->ale, 0,
				ALE_PORT_UNKNOWN_VLAN_MEMBER,
				portcmd(switch_config).unknown_vlan_member);
			ret = 0;
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_802_1:
	{
		char addr_802_1[6] = {0x01, 0x80, 0xc2, 0x0, 0x0, 0x03};
		if (portcmd(switch_config).enable)
			ret = cpsw_ale_add_mcast(priv->ale,
					addr_802_1, 0x7, 0, 0);
		else
			ret = cpsw_ale_add_mcast(priv->ale,
					addr_802_1, 0x3, 0, 0);
		break;
	}

	case CONFIG_SWITCH_MACAUTH:
		ret = cpsw_ale_control_set(priv->ale, priv->host_port,
			ALE_AUTH_ENABLE, 1);
		break;

	case CONFIG_SWITCH_SET_PORT_CONFIG:
	{
		struct ethtool_cmd ecmd;

		if (portcmd(switch_config).port == 1)
			if (priv->slaves[0].phy) {
				ecmd.phy_address = priv->slaves[0].phy->addr;
				ret = phy_ethtool_gset(priv->slaves[0].phy,
						&ecmd);
			} else {
				dev_err(priv->dev, "Phy not Found\n");
				ret = -EFAULT;
				break;
			}
		else if (portcmd(switch_config).port == 2)
			if (priv->slaves[1].phy) {
				ecmd.phy_address = priv->slaves[1].phy->addr;
				ret = phy_ethtool_gset(priv->slaves[1].phy,
						&ecmd);
			} else {
				dev_err(priv->dev, "Phy not Found\n");
				ret = -EFAULT;
				break;
			}
		else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
			break;
		}

		if (((portcmd(switch_config).limit == 0) ||
			(portcmd(switch_config).limit == 10) ||
			(portcmd(switch_config).limit == 100) ||
			(portcmd(switch_config).limit == 1000))) {

			if (portcmd(switch_config).limit == 0)
				ecmd.autoneg = AUTONEG_ENABLE;
			else {
				ecmd.autoneg = AUTONEG_DISABLE;
				ecmd.speed =
					portcmd(switch_config).limit;
				if (portcmd(switch_config).direction
						== 1)
					ecmd.duplex = DUPLEX_FULL;
				else
					ecmd.duplex = DUPLEX_HALF;
			}

			if (portcmd(switch_config).port == 1)
				ret = phy_ethtool_sset(priv->slaves[0].phy,
					&ecmd);
			else
				ret = phy_ethtool_sset(priv->slaves[1].phy,
					&ecmd);
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;
	}

	case CONFIG_SWITCH_GET_PORT_CONFIG:
	{
		struct ethtool_cmd ecmd;

		if (portcmd(switch_config).port == 1)
			if (priv->slaves[0].phy) {
				ecmd.phy_address = priv->slaves[0].phy->addr;
				ret = phy_ethtool_gset(priv->slaves[0].phy,
						&ecmd);
			} else {
				dev_err(priv->dev, "Phy not Found\n");
				ret = -EFAULT;
				break;
			}
		else if (portcmd(switch_config).port == 2)
			if (priv->slaves[1].phy) {
				ecmd.phy_address = priv->slaves[1].phy->addr;
				ret = phy_ethtool_gset(priv->slaves[1].phy,
						&ecmd);
			} else {
				dev_err(priv->dev, "Phy not Found\n");
				ret = -EFAULT;
				break;
			}
		else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
			break;
		}

		if (!ret) {
			portcmd(switch_config).limit = ecmd.speed;
			if (ecmd.duplex == DUPLEX_FULL)
				portcmd(switch_config).direction = 1;
			else
				portcmd(switch_config).direction = 0;
			ret = copy_to_user(ifrq->ifr_data, switch_config,
				sizeof(struct net_switch_config)) ?
				-EFAULT : 0;
		}

		break;
	}

	case CONFIG_SWITCH_PORT_STATE:
		if (portcmd(switch_config).port <= 2) {
			ret = cpsw_set_port_state(priv,
				portcmd(switch_config).port,
				portcmd(switch_config).port_state);
		} else {
			dev_err(priv->dev, "Invalid Arguments\n");
			ret = -EFAULT;
		}
		break;

	case CONFIG_SWITCH_RESET:
		ret = cpsw_ndo_stop(ndev);
		if (!ret)
			ret = cpsw_ndo_open(ndev);
		break;

	default:
		ret = -EOPNOTSUPP;
	}

	kfree(switch_config);
	return ret;
}

#else
#define cpsw_switch_config_ioctl(ndev, ifrq, cmd) (-EOPNOTSUPP)
#endif /* CONFIG_TI_CPSW_DUAL_EMAC */

static int cpsw_ndo_do_ioctl(struct net_device *ndev, struct ifreq *ifrq,
		int cmd)
{
	if (!(netif_running(ndev)))
		return -EINVAL;

	switch (cmd) {

	case SIOCSWITCHCONFIG:
		return cpsw_switch_config_ioctl(ndev, ifrq, cmd);

	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static void cpsw_ndo_change_rx_flags(struct net_device *ndev, int flags)
{
	/*
	 * The switch cannot operate in promiscuous mode without substantial
	 * headache.  For promiscuous mode to work, we would need to put the
	 * ALE in bypass mode and route all traffic to the host port.
	 * Subsequently, the host will need to operate as a "bridge", learn,
	 * and flood as needed.  For now, we simply complain here and
	 * do nothing about it :-)
	 */
	if ((flags & IFF_PROMISC) && (ndev->flags & IFF_PROMISC))
		dev_err(&ndev->dev, "promiscuity ignored!\n");

	/*
	 * The switch cannot filter multicast traffic unless it is configured
	 * in "VLAN Aware" mode.  Unfortunately, VLAN awareness requires a
	 * whole bunch of additional logic that this driver does not implement
	 * at present.
	 */
	if ((flags & IFF_ALLMULTI) && !(ndev->flags & IFF_ALLMULTI))
		dev_err(&ndev->dev, "multicast traffic cannot be filtered!\n");
}

static int cpsw_ndo_set_mac_address(struct net_device *ndev, void *p)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	struct sockaddr *addr = (struct sockaddr *)p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	cpsw_ale_del_ucast(priv->ale, priv->mac_addr, priv->host_port);

	memcpy(priv->mac_addr, addr->sa_data, ETH_ALEN);
	memcpy(ndev->dev_addr, priv->mac_addr, ETH_ALEN);

	cpsw_ale_add_ucast(priv->ale, priv->mac_addr, priv->host_port,
			   0);
	for_each_slave(priv, cpsw_set_slave_mac, priv);
	return 0;
}

static void cpsw_ndo_tx_timeout(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	msg(err, tx_err, "transmit timeout, restarting dma");
	priv->stats.tx_errors++;
	cpsw_intr_disable(priv);
	cpdma_ctlr_int_ctrl(priv->dma, false);
	cpdma_chan_stop(priv->txch);
	cpdma_chan_start(priv->txch);
	cpdma_ctlr_int_ctrl(priv->dma, true);
	cpsw_intr_enable(priv);
	cpdma_ctlr_eoi(priv->dma);
}

static struct net_device_stats *cpsw_ndo_get_stats(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	return &priv->stats;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void cpsw_ndo_poll_controller(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	cpsw_intr_disable(priv);
	cpdma_ctlr_int_ctrl(priv->dma, false);
	cpsw_interrupt(ndev->irq, priv);
	cpdma_ctlr_int_ctrl(priv->dma, true);
	cpsw_intr_enable(priv);
	cpdma_ctlr_eoi(priv->dma);
}
#endif

/**
 * cpsw_get_coalesce : Get interrupt coalesce settings for this device
 * @ndev : CPSW network adapter
 * @coal : ethtool coalesce settings structure
 *
 * Fetch the current interrupt coalesce settings
 *
 */
static int cpsw_get_coalesce(struct net_device *ndev,
				struct ethtool_coalesce *coal)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	coal->rx_coalesce_usecs = priv->coal_intvl;
	return 0;
}

/**
 * cpsw_set_coalesce : Set interrupt coalesce settings for this device
 * @ndev : CPSW network adapter
 * @coal : ethtool coalesce settings structure
 *
 * Set interrupt coalesce parameters
 *
 */
static int cpsw_set_coalesce(struct net_device *ndev,
				struct ethtool_coalesce *coal)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	u32 int_ctrl;
	u32 num_interrupts = 0;
	u32 prescale = 0;
	u32 addnl_dvdr = 1;
	u32 coal_intvl = 0;

	if (!coal->rx_coalesce_usecs)
		return -EINVAL;

	coal_intvl = coal->rx_coalesce_usecs;

	int_ctrl =  __raw_readl(&priv->ss_regs->int_control);
	prescale = priv->bus_freq_mhz * 4;

	if (coal_intvl < CPSW_CMINTMIN_INTVL)
		coal_intvl = CPSW_CMINTMIN_INTVL;

	if (coal_intvl > CPSW_CMINTMAX_INTVL) {
		/*
		 * Interrupt pacer works with 4us Pulse, we can
		 * throttle further by dilating the 4us pulse.
		 */
		addnl_dvdr = CPSW_INTPRESCALE_MASK / prescale;

		if (addnl_dvdr > 1) {
			prescale *= addnl_dvdr;
			if (coal_intvl > (CPSW_CMINTMAX_INTVL * addnl_dvdr))
				coal_intvl = (CPSW_CMINTMAX_INTVL
						* addnl_dvdr);
		} else {
			addnl_dvdr = 1;
			coal_intvl = CPSW_CMINTMAX_INTVL;
		}
	}

	num_interrupts = (1000 * addnl_dvdr) / coal_intvl;

	int_ctrl |= CPSW_INTPACEEN;
	int_ctrl &= (~CPSW_INTPRESCALE_MASK);
	int_ctrl |= (prescale & CPSW_INTPRESCALE_MASK);
	__raw_writel(int_ctrl, &priv->ss_regs->int_control);

	__raw_writel(num_interrupts, &priv->ss_regs->rx_imax);
	__raw_writel(num_interrupts, &priv->ss_regs->tx_imax);

	printk(KERN_INFO"Set coalesce to %d usecs.\n", coal_intvl);
	priv->coal_intvl = coal_intvl;

	return 0;
}

#ifdef VLAN_SUPPORT

#ifdef CONFIG_TI_CPSW_DUAL_EMAC

static inline void cpsw_add_vlan_ale_entry(struct cpsw_priv *priv,
					   unsigned short vid)
{
	cpsw_ale_add_vlan(priv->ale, vid, 1 << priv->host_port |
			1 << (priv->emac_port + 1), 0,
			1 << priv->host_port | 1 << (priv->emac_port + 1), 0);
	cpsw_ale_vlan_add_ucast(priv->ale, priv->mac_addr, priv->host_port,
			0, vid);
	cpsw_ale_vlan_add_mcast(priv->ale, priv->ndev->broadcast,
			1 << priv->host_port | 1 << (priv->emac_port + 1),
			vid, 0, 0);
}

#else

static inline void cpsw_add_vlan_ale_entry(struct cpsw_priv *priv,
				unsigned short vid)
{
	cpsw_ale_add_vlan(priv->ale, vid, ALE_ALL_PORTS << priv->host_port,
			0, ALE_ALL_PORTS << priv->host_port,
			(BIT(1) | BIT(2)) << priv->host_port);
	cpsw_ale_vlan_add_ucast(priv->ale, priv->mac_addr,
			priv->host_port, 0, vid);
	cpsw_ale_vlan_add_mcast(priv->ale, priv->ndev->broadcast,
			ALE_ALL_PORTS << priv->host_port, vid, 0, 0);
}

#endif

static void cpsw_ndo_vlan_rx_add_vid(struct net_device *ndev,
		unsigned short vid)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	spin_lock(&priv->lock);

	dev_dbg(priv->dev, "Adding vlanid %d to vlan filter\n", vid);
	if (vid) {
		cpsw_add_vlan_ale_entry(priv, vid);
	} else {
		cpsw_ale_add_vlan(priv->ale, vid,
				ALE_ALL_PORTS << priv->host_port,
				ALE_ALL_PORTS << priv->host_port,
				ALE_ALL_PORTS << priv->host_port,
				(BIT(1) | BIT(2)) << priv->host_port);
	}

	spin_unlock(&priv->lock);
}

static void cpsw_ndo_vlan_rx_kill_vid(struct net_device *ndev,
		unsigned short vid)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	spin_lock(&priv->lock);

	dev_dbg(priv->dev, "removing vlanid %d from vlan filter\n", vid);
	cpsw_ale_del_vlan(priv->ale, vid, ALE_ALL_PORTS << priv->host_port);
	cpsw_ale_vlan_del_ucast(priv->ale, priv->mac_addr,
				priv->host_port, vid);
	cpsw_ale_vlan_del_mcast(priv->ale, priv->ndev->broadcast, 0, vid);

	spin_unlock(&priv->lock);
}

#endif /* VLAN_SUPPORT */

static const struct net_device_ops cpsw_netdev_ops = {
	.ndo_open		= cpsw_ndo_open,
	.ndo_stop		= cpsw_ndo_stop,
	.ndo_start_xmit		= cpsw_ndo_start_xmit,
	.ndo_change_rx_flags	= cpsw_ndo_change_rx_flags,
	.ndo_set_mac_address	= cpsw_ndo_set_mac_address,
	.ndo_do_ioctl		= cpsw_ndo_do_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_tx_timeout		= cpsw_ndo_tx_timeout,
	.ndo_get_stats		= cpsw_ndo_get_stats,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= cpsw_ndo_poll_controller,
#endif
#ifdef VLAN_SUPPORT
	.ndo_vlan_rx_add_vid	= cpsw_ndo_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= cpsw_ndo_vlan_rx_kill_vid,
#endif
};

static void cpsw_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	strcpy(info->driver, "TI CPSW Driver v1.0");
	strcpy(info->version, "1.0");
	strcpy(info->bus_info, priv->pdev->name);
}

static u32 cpsw_get_msglevel(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	return priv->msg_enable;
}

static void cpsw_set_msglevel(struct net_device *ndev, u32 value)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	priv->msg_enable = value;
}

/**
 * cpsw_get_settings: Get EMAC settings
 * @ndev: CPSW network adapter
 * @ecmd: ethtool command
 *
 * Executes ethool get command
 *
 */
static int cpsw_get_settings(struct net_device *ndev,
			     struct ethtool_cmd *ecmd)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	int slave_no = cpsw_slave_phy_index(priv);

	if (priv->slaves[slave_no].phy)
		return phy_ethtool_gset(priv->slaves[slave_no].phy, ecmd);
	else
		return -EOPNOTSUPP;
}

/**
 * cpsw_set_settings: Set EMAC settings
 * @ndev: CPSW network adapter
 * @ecmd: ethtool command
 *
 * Executes ethool set command
 *
 */
static int cpsw_set_settings(struct net_device *ndev, struct ethtool_cmd *ecmd)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	int slave_no = cpsw_slave_phy_index(priv);

	if (priv->slaves[slave_no].phy)
		return phy_ethtool_sset(priv->slaves[slave_no].phy, ecmd);
	else
		return -EOPNOTSUPP;
}

static const struct ethtool_ops cpsw_ethtool_ops = {
	.get_drvinfo	= cpsw_get_drvinfo,
	.get_msglevel	= cpsw_get_msglevel,
	.set_msglevel	= cpsw_set_msglevel,
	.get_link	= ethtool_op_get_link,
	.get_settings	= cpsw_get_settings,
	.set_settings	= cpsw_set_settings,
	.get_coalesce	= cpsw_get_coalesce,
	.set_coalesce	= cpsw_set_coalesce,
};

static void cpsw_slave_init(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	void __iomem		*regs = priv->regs;
	int			slave_num = slave->slave_num;
	struct cpsw_slave_data	*data = priv->data.slave_data + slave_num;

	slave->data	= data;
	slave->regs	= regs + data->slave_reg_ofs;
	slave->sliver	= regs + data->sliver_reg_ofs;
	slave->port_vlan = data->dual_emac_reserved_vlan;
}

#ifdef CONFIG_TI_CPSW_DUAL_EMAC

static int cpsw_init_slave_emac(struct platform_device *pdev,
				struct cpsw_priv *priv)
{
	struct cpsw_platform_data	*data = pdev->dev.platform_data;
	struct net_device		*ndev;
	struct cpsw_priv		*priv_sl2;
	int ret = 0, i;

	ndev = alloc_etherdev(sizeof(struct cpsw_priv));
	if (!ndev) {
		pr_err("cpsw: error allocating net_device\n");
		return -ENOMEM;
	}

	priv_sl2 = netdev_priv(ndev);
	spin_lock_init(&priv_sl2->lock);
	priv_sl2->data = *data;
	priv_sl2->pdev = pdev;
	priv_sl2->ndev = ndev;
	priv_sl2->dev  = &ndev->dev;
	priv_sl2->msg_enable = netif_msg_init(debug_level, CPSW_DEBUG);
	priv_sl2->rx_packet_max = max(rx_packet_max, 128);

	if (is_valid_ether_addr(data->slave_data[1].mac_addr)) {
		memcpy(priv_sl2->mac_addr, data->slave_data[1].mac_addr,
			ETH_ALEN);
		pr_info("cpsw: Detected MACID = %pM\n", priv_sl2->mac_addr);
	} else {
		random_ether_addr(priv_sl2->mac_addr);
		pr_info("cpsw: Random MACID = %pM\n", priv_sl2->mac_addr);
	}
	memcpy(ndev->dev_addr, priv_sl2->mac_addr, ETH_ALEN);

	priv_sl2->slaves = priv->slaves;
	priv_sl2->clk = priv->clk;

	priv_sl2->coal_intvl = 0;
	priv_sl2->bus_freq_mhz = clk_get_rate(priv_sl2->clk) / 1000000;

	priv_sl2->cpsw_res = priv->cpsw_res;
	priv_sl2->regs = priv->regs;
	priv_sl2->host_port = priv->host_port;
	priv_sl2->host_port_regs = priv->host_port_regs;
	priv_sl2->hw_stats = priv->hw_stats;
	priv_sl2->cpsw_ss_res = priv->cpsw_ss_res;
	priv_sl2->ss_regs = priv->ss_regs;
	priv_sl2->dma = priv->dma;
	priv_sl2->txch = priv->txch;
	priv_sl2->rxch = priv->rxch;
	priv_sl2->ale = priv->ale;
	priv_sl2->emac_port = 1;
	priv->slaves[1].ndev = ndev;
	for_each_slave(priv_sl2, cpsw_slave_init, priv_sl2);

	for (i = 0; i < priv->num_irqs; i++) {
		priv_sl2->irqs_table[i] = priv->irqs_table[i];
		priv_sl2->num_irqs = priv->num_irqs;
	}

#ifdef VLAN_SUPPORT
	ndev->features |= NETIF_F_HW_VLAN_FILTER;
#endif

	ndev->flags |= IFF_ALLMULTI;	/* see cpsw_ndo_change_rx_flags() */

	ndev->netdev_ops = &cpsw_netdev_ops;
	SET_ETHTOOL_OPS(ndev, &cpsw_ethtool_ops);
	netif_napi_add(ndev, &priv_sl2->napi, cpsw_poll, CPSW_POLL_WEIGHT);

	/* register the network device */
	SET_NETDEV_DEV(ndev, &pdev->dev);
	ret = register_netdev(ndev);
	if (ret) {
		pr_err("cpsw: error registering net device\n");
		ret = -ENODEV;
	}

	return ret;
}

static inline void cpsw_deinit_slave_emac(struct cpsw_priv *priv)
{
	struct net_device *ndev = priv->slaves[1].ndev;

	unregister_netdev(ndev);
	free_netdev(ndev);
}

#else
#define cpsw_init_slave_emac(pdev, priv)	0
#define cpsw_deinit_slave_emac(priv)
#endif

static int __devinit cpsw_probe(struct platform_device *pdev)
{
	struct cpsw_platform_data	*data = pdev->dev.platform_data;
	struct net_device		*ndev;
	struct cpsw_priv		*priv;
	struct cpdma_params		dma_params;
	struct cpsw_ale_params		ale_params;
	void __iomem			*regs;
	int ret = 0, i, k = 0;

	if (!data) {
		pr_err("cpsw: platform data missing\n");
		return -ENODEV;
	}

	ndev = alloc_etherdev(sizeof(struct cpsw_priv));
	if (!ndev) {
		pr_err("cpsw: error allocating net_device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ndev);
	priv = netdev_priv(ndev);
	spin_lock_init(&priv->lock);
	priv->data = *data;
	priv->pdev = pdev;
	priv->ndev = ndev;
	priv->dev  = &ndev->dev;
	priv->msg_enable = netif_msg_init(debug_level, CPSW_DEBUG);
	priv->rx_packet_max = max(rx_packet_max, 128);

	if (is_valid_ether_addr(data->mac_addr)) {
		memcpy(priv->mac_addr, data->mac_addr, ETH_ALEN);
		printk(KERN_INFO"Detected MACID=%x:%x:%x:%x:%x:%x\n",
			priv->mac_addr[0], priv->mac_addr[1],
			priv->mac_addr[2], priv->mac_addr[3],
			priv->mac_addr[4], priv->mac_addr[5]);
	} else {
		random_ether_addr(priv->mac_addr);
		printk(KERN_INFO"Random MACID=%x:%x:%x:%x:%x:%x\n",
			priv->mac_addr[0], priv->mac_addr[1],
			priv->mac_addr[2], priv->mac_addr[3],
			priv->mac_addr[4], priv->mac_addr[5]);
	}

	memcpy(ndev->dev_addr, priv->mac_addr, ETH_ALEN);

	priv->slaves = kzalloc(sizeof(struct cpsw_slave) * data->slaves,
			       GFP_KERNEL);
	if (!priv->slaves) {
		dev_err(priv->dev, "failed to allocate slave ports\n");
		ret = -EBUSY;
		goto clean_ndev_ret;
	}
	for (i = 0; i < data->slaves; i++)
		priv->slaves[i].slave_num = i;

	priv->slaves[0].ndev = ndev;
	priv->emac_port = 0;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	priv->clk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "fck is not found\n");
		ret = -ENODEV;
		goto clean_slave_ret;
	}

	priv->coal_intvl = 0;
	priv->bus_freq_mhz = clk_get_rate(priv->clk) / 1000000;

	priv->cpsw_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!priv->cpsw_res) {
		dev_err(priv->dev, "error getting i/o resource\n");
		ret = -ENOENT;
		goto clean_clk_ret;
	}

	if (!request_mem_region(priv->cpsw_res->start,
		resource_size(priv->cpsw_res), ndev->name)) {
		dev_err(priv->dev, "failed request i/o region\n");
		ret = -ENXIO;
		goto clean_clk_ret;
	}

	regs = ioremap(priv->cpsw_res->start, resource_size(priv->cpsw_res));
	if (!regs) {
		dev_err(priv->dev, "unable to map i/o region\n");
		goto clean_cpsw_iores_ret;
	}
	priv->regs = regs;
	priv->host_port = data->host_port_num;
	priv->host_port_regs = regs + data->host_port_reg_ofs;
	priv->hw_stats = regs + data->hw_stats_reg_ofs;

	priv->cpsw_ss_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!priv->cpsw_ss_res) {
		dev_err(priv->dev, "error getting i/o resource\n");
		ret = -ENOENT;
		goto clean_clk_ret;
	}

	if (!request_mem_region(priv->cpsw_ss_res->start,
		resource_size(priv->cpsw_ss_res), ndev->name)) {
		dev_err(priv->dev, "failed request i/o region\n");
		ret = -ENXIO;
		goto clean_clk_ret;
	}

	regs = ioremap(priv->cpsw_ss_res->start,
			resource_size(priv->cpsw_ss_res));
	if (!regs) {
		dev_err(priv->dev, "unable to map i/o region\n");
		goto clean_cpsw_ss_iores_ret;
	}
	priv->ss_regs = regs;

	for_each_slave(priv, cpsw_slave_init, priv);

	omap_ctrl_writel(CPSW_TIMER_MASK, CPSW_TIMER_CAP_REG);

	dmtimer_rx = omap_dm_timer_request_specific(CPSW_RX_TIMER_REQ);
	if (!dmtimer_rx) {
		dev_err(priv->dev, "Error getting Rx Timer resource\n");
		ret = -ENODEV;
		goto clean_iomap_ret;
	}
	dmtimer_tx = omap_dm_timer_request_specific(CPSW_TX_TIMER_REQ);
	if (!dmtimer_tx) {
		dev_err(priv->dev, "Error getting Tx Timer resource\n");
		ret = -ENODEV;
		goto clean_timer_rx_ret;
	}

	memset(&dma_params, 0, sizeof(dma_params));
	dma_params.dev			= &pdev->dev;
	dma_params.dmaregs		= (void __iomem *)(((u32)priv->regs) +
						data->cpdma_reg_ofs);
	dma_params.rxthresh		= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_RXTHRESH);
	dma_params.rxfree		= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_RXFREE);

	if (data->version == CPSW_VERSION_2) {
		dma_params.txhdp	= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_TXHDP_VER2);
		dma_params.rxhdp	= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_RXHDP_VER2);
		dma_params.txcp		= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_TXCP_VER2);
		dma_params.rxcp		= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_RXCP_VER2);
	} else {
		dma_params.txhdp	= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_TXHDP_VER1);
		dma_params.rxhdp	= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_RXHDP_VER1);
		dma_params.txcp		= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_TXCP_VER1);
		dma_params.rxcp		= (void __iomem *)(((u32)priv->regs) +
					data->cpdma_reg_ofs + CPDMA_RXCP_VER1);
	}

	dma_params.num_chan		= data->channels;
	dma_params.has_soft_reset	= true;
	dma_params.min_packet_size	= CPSW_MIN_PACKET_SIZE;
	dma_params.desc_mem_size	= data->bd_ram_size;
	dma_params.desc_align		= 16;
	dma_params.has_ext_regs		= true;
	dma_params.desc_mem_phys	= data->no_bd_ram ? 0 :
			(u32 __force)priv->cpsw_res->start + data->bd_ram_ofs;
	dma_params.desc_hw_addr		= data->hw_ram_addr ?
				data->hw_ram_addr : dma_params.desc_mem_phys ;

	priv->dma = cpdma_ctlr_create(&dma_params);
	if (!priv->dma) {
		dev_err(priv->dev, "error initializing dma\n");
		ret = -ENOMEM;
		goto clean_timer_ret;
	}

	priv->txch = cpdma_chan_create(priv->dma, tx_chan_num(0),
				       cpsw_tx_handler);
	priv->rxch = cpdma_chan_create(priv->dma, rx_chan_num(0),
				       cpsw_rx_handler);

	if (WARN_ON(!priv->txch || !priv->rxch)) {
		dev_err(priv->dev, "error initializing dma channels\n");
		ret = -ENOMEM;
		goto clean_dma_ret;
	}

	memset(&ale_params, 0, sizeof(ale_params));
	ale_params.dev		= &ndev->dev;
	ale_params.ale_regs	= (void *)((u32)priv->regs) +
					((u32)data->ale_reg_ofs);
	ale_params.ale_ageout	= ale_ageout;
	ale_params.ale_entries	= data->ale_entries;
	ale_params.ale_ports	= data->slaves;

	priv->ale = cpsw_ale_create(&ale_params);
	if (!priv->ale) {
		dev_err(priv->dev, "error initializing ale engine\n");
		ret = -ENODEV;
		goto clean_dma_ret;
	}

	while ((i = platform_get_irq(pdev, k)) >= 0) {
		if (request_irq(i, cpsw_interrupt, IRQF_DISABLED,
				dev_name(&pdev->dev), priv)) {
			dev_err(priv->dev, "error attaching irq\n");
			goto clean_ale_ret;
		}
		priv->irqs_table[k] = i;
		priv->num_irqs = ++k;
	}

#ifdef VLAN_SUPPORT
	ndev->features |= NETIF_F_HW_VLAN_FILTER;
#endif

	ndev->flags |= IFF_ALLMULTI;	/* see cpsw_ndo_change_rx_flags() */

	ndev->netdev_ops = &cpsw_netdev_ops;
	SET_ETHTOOL_OPS(ndev, &cpsw_ethtool_ops);
	netif_napi_add(ndev, &priv->napi, cpsw_poll, CPSW_POLL_WEIGHT);

	/* register the network device */
	SET_NETDEV_DEV(ndev, &pdev->dev);
	ret = register_netdev(ndev);
	if (ret) {
		dev_err(priv->dev, "error registering net device\n");
		ret = -ENODEV;
		goto clean_irq_ret;
	}

	msg(notice, probe, "initialized device (regs %x, irq %d)\n",
	    priv->cpsw_res->start, ndev->irq);

	ret = cpsw_init_slave_emac(pdev, priv);
	if (ret) {
		dev_err(priv->dev, "error probe slave 2 emac interface\n");
		goto clean_irq_ret;
	}

	return 0;

clean_irq_ret:
	free_irq(ndev->irq, priv);
clean_ale_ret:
	cpsw_ale_destroy(priv->ale);
clean_dma_ret:
	cpdma_chan_destroy(priv->txch);
	cpdma_chan_destroy(priv->rxch);
	cpdma_ctlr_destroy(priv->dma);
clean_timer_ret:
	omap_dm_timer_free(dmtimer_tx);
clean_timer_rx_ret:
	omap_dm_timer_free(dmtimer_rx);
clean_iomap_ret:
	iounmap(priv->regs);
clean_cpsw_ss_iores_ret:
	release_mem_region(priv->cpsw_ss_res->start,
				resource_size(priv->cpsw_ss_res));
clean_cpsw_iores_ret:
	release_mem_region(priv->cpsw_res->start,
				resource_size(priv->cpsw_res));
clean_clk_ret:
	clk_put(priv->clk);
clean_slave_ret:
	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	kfree(priv->slaves);
clean_ndev_ret:
	free_netdev(ndev);
	return ret;
}

static int __devexit cpsw_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct cpsw_priv *priv = netdev_priv(ndev);
	u32 i;

	msg(notice, probe, "removing device\n");
	platform_set_drvdata(pdev, NULL);

	cpsw_deinit_slave_emac(priv);
	omap_dm_timer_free(dmtimer_rx);
	omap_dm_timer_free(dmtimer_tx);
	for (i = 0; i < priv->num_irqs; i++)
		free_irq(priv->irqs_table[i], priv);
	cpsw_ale_destroy(priv->ale);
	cpdma_chan_destroy(priv->txch);
	cpdma_chan_destroy(priv->rxch);
	cpdma_ctlr_destroy(priv->dma);
	iounmap(priv->regs);
	release_mem_region(priv->cpsw_res->start,
				resource_size(priv->cpsw_res));
	release_mem_region(priv->cpsw_ss_res->start,
				resource_size(priv->cpsw_ss_res));
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	clk_put(priv->clk);
	kfree(priv->slaves);
	unregister_netdev(ndev);
	free_netdev(ndev);

	return 0;
}

#ifdef CONFIG_TI_CPSW_DUAL_EMAC

static inline void cpsw_start_slaves_interface(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	u32 i = 0;

	for (i = 0; i < priv->data.slaves; i++) {
		ndev = cpsw_get_slave_ndev(priv, i);
		if (netif_running(ndev))
			cpsw_ndo_open(ndev);
	}
}

static inline void cpsw_stop_slaves_interface(struct net_device *ndev)
{
	u32 i = 0;
	struct cpsw_priv *priv = netdev_priv(ndev);

	for (i = 0; i < priv->data.slaves; i++) {
		ndev = cpsw_get_slave_ndev(priv, i);
		if (netif_running(ndev))
			cpsw_ndo_stop(ndev);
	}
}

#else

#define cpsw_start_slaves_interface(ndev)		\
	if (netif_running(ndev))			\
		cpsw_ndo_open(ndev)
#define cpsw_stop_slaves_interface(ndev)		\
	if (netif_running(ndev))			\
		cpsw_ndo_stop(ndev)

#endif

static int cpsw_suspend(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct net_device	*ndev = platform_get_drvdata(pdev);

	cpsw_stop_slaves_interface(ndev);
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int cpsw_resume(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct net_device	*ndev = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);
	cpsw_start_slaves_interface(ndev);

	return 0;
}

static const struct dev_pm_ops cpsw_pm_ops = {
	.suspend	= cpsw_suspend,
	.resume		= cpsw_resume,
};

static struct platform_driver cpsw_driver = {
	.driver = {
		.name	 = "cpsw",
		.owner	 = THIS_MODULE,
		.pm	 = &cpsw_pm_ops,
	},
	.probe = cpsw_probe,
	.remove = __devexit_p(cpsw_remove),
};

static int __init cpsw_init(void)
{
	return platform_driver_register(&cpsw_driver);
}
late_initcall(cpsw_init);

static void __exit cpsw_exit(void)
{
	platform_driver_unregister(&cpsw_driver);
}
module_exit(cpsw_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI CPSW Ethernet driver");
