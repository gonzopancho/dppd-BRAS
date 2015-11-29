/*
  Copyright(c) 2010-2014 Intel Corporation.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "handle_fwd.h"
#include "tx_pkt.h"
#include "stats.h"
#include "handle_routing.h"

static inline void handle_fwd(struct rte_mbuf *rx_mbuf, struct task_fwd *ptask);

void handle_fwd_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_fwd *port = (struct task_fwd *)ptask;
	uint16_t j;
#ifdef BRAS_PREFETCH_OFFSET
	for (j = 0; (j < BRAS_PREFETCH_OFFSET) && (j < n_pkts); ++j) {
		PREFETCH0(rx_mbuf[j]);
	}
	for (j = 1; (j < BRAS_PREFETCH_OFFSET) && (j < n_pkts); ++j) {
		PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[j - 1], struct ether_hdr *));
	}
#endif
	for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
#ifdef BRAS_PREFETCH_OFFSET
		PREFETCH0(rx_mbuf[j + PREFETCH_OFFSET]);
		PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[j + PREFETCH_OFFSET - 1], struct ether_hdr *));
#endif
		handle_fwd(rx_mbuf[j], port);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		handle_fwd(rx_mbuf[j], port);
	}
#endif
	port->tx_pkt(&port->base);
}

static inline void handle_fwd(struct rte_mbuf *rx_mbuf, struct task_fwd *ptask)
{
	struct ether_hdr *peth = rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *);
	struct ipv4_hdr *pip;

	DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "RX", rx_mbuf);

	if (peth->ether_type == ETYPE_8021ad) {
		struct vlan_hdr *psvlan, *pcvlan;
		/* Skip and Store SVLAN and CVLAN */
		psvlan = (struct vlan_hdr *)(peth + 1);
		if (((psvlan->eth_proto) & 0xFF) != ETYPE_VLAN) {
			uint16_t proto = (psvlan->eth_proto) & 0xFF;
			mprintf("Unexpected proto in QinQ = %#04x\n", proto);
			INCR_TX_DROP_COUNT(ptask->base.stats, 1);
			rte_pktmbuf_free(rx_mbuf);
			return;
		}

		pcvlan = (struct vlan_hdr *)(psvlan + 1);
		pip = (struct ipv4_hdr *)(pcvlan + 1);
	}
	else {
		pip = (struct ipv4_hdr *)(peth + 1);
	}

	if ((pip->version_ihl >> 4) == 4) {
		if (pip->time_to_live) {
			pip->time_to_live--;
		}
		else {
			mprintf("TTL = 0 => Dropping\n");
			INCR_TX_DROP_COUNT(ptask->base.stats, 1);
			rte_pktmbuf_free(rx_mbuf);
			return;
		}
		pip->hdr_checksum = 0;
#ifdef SOFT_CRC
		pip->ip_sum = tgen_ip_cksum((void *)pip, sizeof(struct ipv4_hdr), 0);
#elif defined(HARD_CRC)
		rx_mbuf->pkt.vlan_macip.data = (sizeof(struct ether_hdr) << 9) | (sizeof(struct ipv4_hdr));
		rx_mbuf->ol_flags |= PKT_TX_IP_CKSUM;
#endif

		uint8_t tx_portid = 0;
		if (ptask->runtime_flags & TASK_ROUTING) {
			if ((tx_portid = route(rx_mbuf, ptask->next_hop, ptask->ipv4_lpm, pip)) == ROUTE_ERR) {
				rte_pktmbuf_free(rx_mbuf);
				INCR_TX_DROP_COUNT(ptask->base.stats, 1);
				return;
			}
		}
		else {
			ether_addr_copy(&ptask->edaddr, &peth->d_addr);
		}

		DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "TX", rx_mbuf);
		tx_buf_pkt_single(&ptask->base, rx_mbuf, tx_portid);
	}
	else if ((pip->version_ihl >> 4) == 6) {
		struct ipv6_hdr *pip6 = (struct ipv6_hdr *)(peth + 1);
		ether_addr_copy(&ptask->edaddr, &peth->d_addr);
		/* Decrement TTL */
		if (pip6->hop_limits) {
			pip6->hop_limits--;
		}
		else {
			mprintf("TTL = 0 => Dropping\n");
			INCR_TX_DROP_COUNT(ptask->base.stats, 1);
			rte_pktmbuf_free(rx_mbuf);
			return;
		}
		tx_buf_pkt_single(&ptask->base, rx_mbuf, 0);
		return;
	}
	else {
		mprintf("Core %u: Error in Fwd mode: version=%u, protocol = %u\n", ptask->lconf->corenb, pip->version_ihl, pip->next_proto_id);
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}
}
