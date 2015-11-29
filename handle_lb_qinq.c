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

#include <rte_mbuf.h>
#include <rte_ip.h>
#include <rte_byteorder.h>

#include "handle_lb_qinq.h"
#include "tx_pkt.h"
#include "etypes.h"
#include "display.h"
#include "stats.h"
#include "qinq.h"
#include "quit.h"
#include "prefetch.h"
#include "defines.h"

static inline uint8_t handle_lb_qinq(struct rte_mbuf *rx_mbuf, struct task_lb_qinq *ptask, uint8_t *dest_wt);

void handle_lb_qinq_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_lb_qinq *port = (struct task_lb_qinq *)ptask;
	uint16_t j;
	uint8_t dest_wt[MAX_RING_BURST];
	uint16_t not_dropped = 0;
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
		rx_mbuf[not_dropped] = rx_mbuf[j];
		not_dropped += handle_lb_qinq(rx_mbuf[j], port, &dest_wt[not_dropped]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		rx_mbuf[not_dropped] = rx_mbuf[j];
		not_dropped += handle_lb_qinq(rx_mbuf[j], port, &dest_wt[not_dropped]);
	}
#endif

	if (likely(not_dropped)) {
		for (j = 0; j < not_dropped; ++j) {
			tx_buf_pkt_single(&port->base, rx_mbuf[j], dest_wt[j]);
		}
		port->tx_pkt(&port->base);
	}
}

struct cpe_packet {
	struct qinq_hdr qinq_hdr;
	union {
		struct ipv4_hdr ipv4_hdr;
		struct ipv6_hdr ipv6_hdr;
	};
} __attribute__((packed));

static inline uint8_t handle_lb_qinq(struct rte_mbuf *rx_mbuf, struct task_lb_qinq *ptask, uint8_t *dest_wt)
{
	struct cpe_packet *packet = rte_pktmbuf_mtod(rx_mbuf, struct cpe_packet *);

	if (unlikely(packet->qinq_hdr.svlan.eth_proto != ETYPE_8021ad)) {
		/* might receive LLDP from the L2 switch... */
		if (packet->qinq_hdr.svlan.eth_proto != ETYPE_LLDP) {
			mprintf("Core LB: Error getting non Q in Q packets in LB for Q in Q mode\n");
			DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "TX", rx_mbuf);
		}
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return 0;
	}

	TGEN_PANIC(packet->qinq_hdr.cvlan.eth_proto != ETYPE_VLAN, "Error in ETYPE_8021ad\n");

	uint8_t worker = 0;
	uint8_t proto = 0xFF;
	switch (packet->qinq_hdr.ether_type) {
	case ETYPE_IPv4: {
		if (unlikely((packet->ipv4_hdr.version_ihl >> 4) != 4)) {
			mprintf("Invalid Version %u for ETYPE_IPv4\n", packet->ipv4_hdr.version_ihl);
			INCR_TX_DROP_COUNT(ptask->base.stats, 1);
			rte_pktmbuf_free(rx_mbuf);
			return 0;
		}
		uint16_t svlan = packet->qinq_hdr.svlan.vlan_tci;
		uint16_t cvlan = packet->qinq_hdr.cvlan.vlan_tci;
		prefetch_nta(&ptask->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)]);
		worker = ptask->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)];
#ifdef ENABLE_VERBOSITY
		if (verbose == 2) {
			uint16_t cvlan = rte_be_to_cpu_16(packet->qinq_hdr.cvlan.vlan_tci) & 0xFFF;
			size_t pos = offsetof(struct cpe_packet, qinq_hdr.cvlan.vlan_tci);
			mprintf("qinq = %u, worker = %u, pos = %lu\n", cvlan, worker, pos);
		}
#endif
		proto = IPV4;
		break;
	}
	case ETYPE_IPv6: {
		if (unlikely((packet->ipv4_hdr.version_ihl >> 4) != 6)) {
			mprintf("Invalid Version %u for ETYPE_IPv6\n", packet->ipv4_hdr.version_ihl);
			INCR_TX_DROP_COUNT(ptask->base.stats, 1);
			rte_pktmbuf_free(rx_mbuf);
			return 0;
		}
		/* Use IP Destination when IPV6 QinQ */
		if (ptask->bit_mask != 0xff) {
			worker = ((uint8_t *)packet)[61] & ptask->bit_mask;
		}
		else {
			worker = ((uint8_t *)packet)[61] % ptask->nb_worker_threads;
		}
		proto = IPV6;
		break;
	}
	case ETYPE_ARP: {
		uint16_t svlan = packet->qinq_hdr.svlan.vlan_tci;
		uint16_t cvlan = packet->qinq_hdr.cvlan.vlan_tci;
		prefetch_nta(&ptask->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)]);
		worker = ptask->worker_thread_table[PKT_TO_LUTQINQ(svlan, cvlan)];
		proto = ARP;
		break;
	}
	default:
		TGEN_PANIC(1, "Error in ETYPE_8021ad: ether_type = %#06x\n", packet->qinq_hdr.ether_type);
	}

	*dest_wt = worker + proto * ptask->nb_worker_threads;
	return 1;
}
