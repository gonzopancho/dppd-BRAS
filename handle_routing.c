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

#include "handle_routing.h"
#include "tx_pkt.h"
#include "stats.h"
#include "gre.h"

static inline void handle_routing(struct rte_mbuf *rx_mbuf, struct task_routing *ptask);

void handle_routing_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_routing *port = (struct task_routing *)ptask;
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
		handle_routing(rx_mbuf[j], port);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		handle_routing(rx_mbuf[j], port);
	}
#endif
	port->tx_pkt(&port->base);
}

static inline void handle_routing(struct rte_mbuf *rx_mbuf, struct task_routing *ptask)
{
	struct ether_hdr *peth = rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *);
	struct ipv4_hdr *p_tunnel_ip = (struct ipv4_hdr *)(peth + 1);

	DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "RX", rx_mbuf);

	uint8_t tx_portid = 0;
	switch (peth->ether_type) {
	case ETYPE_IPv4:
		if (likely((p_tunnel_ip->version_ihl >> 4) == 4 && p_tunnel_ip->next_proto_id == IPPROTO_GRE)) {
			struct gre_hdr *pgre = (struct gre_hdr *)(p_tunnel_ip + 1);
			struct ipv4_hdr *pip = (struct ipv4_hdr *)(pgre + 1);
			if (unlikely((tx_portid = route(rx_mbuf, ptask->next_hop, ptask->ipv4_lpm, pip)) == ROUTE_ERR)) {
				rte_pktmbuf_free(rx_mbuf);
				INCR_TX_DROP_COUNT(ptask->base.stats, 1);
				return;
			}
		}
		else {
			// Basic IPv4 routing not supported yet
			mprintf("Error Routing ip_v = %u, next_proto_id = %u\n", p_tunnel_ip->version_ihl, p_tunnel_ip->next_proto_id);
			rte_pktmbuf_free(rx_mbuf);
			return;
		}
		break;
	case ETYPE_IPv6:
		// Routing not supported on V6 yet - only support adding MPLS tags
		// MPLS Encapsulation
		peth = mpls_encap_v6(rx_mbuf);
		ether_addr_copy(&ptask->edaddr, &peth->d_addr);
		break;
	case ETYPE_MPLSU:
		mprintf("Core %u Error Adding MPLS to a MPLS packet\n", ptask->lconf->corenb);
		rte_pktmbuf_free(rx_mbuf);
		return;
	default:
		mprintf("Error Adding MPLS to a non IPv4 or IPv6 packet ether_type %#06x\n", peth->ether_type);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}

	DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "TX", rx_mbuf);
	tx_buf_pkt_single(&ptask->base, rx_mbuf, tx_portid);
}
