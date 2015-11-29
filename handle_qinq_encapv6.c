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

#include <rte_hash.h>

#include "handle_qinq_encapv6.h"
#include "handle_qinq_encapv4.h"

#include "display.h"
#include "qinq.h"
#include "defines.h"
#include "classify.h"
#include "tx_pkt.h"
#include "hash_entry_types.h"

static inline void handle_qinq_encap_v6(struct rte_mbuf *rx_mbuf, struct task_qinq_encap_v6 *ptask);

void handle_qinq_encap_v6_untag_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_qinq_encap_v6 *port = (struct task_qinq_encap_v6 *)ptask;
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
		if (likely(untag_or_drop(rx_mbuf[j]))) {
			handle_qinq_encap_v6(rx_mbuf[j], port);
		}
		else {
			INCR_TX_DROP_COUNT(port->base.stats, 1);
			rte_pktmbuf_free(rx_mbuf[j]);
		}
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		if (likely(untag_or_drop(rx_mbuf[j]))) {
			handle_qinq_encap_v6(rx_mbuf[j], port);
		}
		else {
			INCR_TX_DROP_COUNT(port->base.stats, 1);
			rte_pktmbuf_free(rx_mbuf[j]);
		}
	}
#endif
	port->tx_pkt(&port->base);
}

void handle_qinq_encap_v6_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_qinq_encap_v6 *port = (struct task_qinq_encap_v6 *)ptask;
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
		handle_qinq_encap_v6(rx_mbuf[j], port);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		handle_qinq_encap_v6(rx_mbuf[j], port);
	}
#endif
	port->tx_pkt(&port->base);
}

static inline void handle_qinq_encap_v6(struct rte_mbuf *rx_mbuf, struct task_qinq_encap_v6 *ptask)
{
	DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "RX", rx_mbuf);


	struct qinq_hdr *pqinq = (struct qinq_hdr *)rte_pktmbuf_prepend(rx_mbuf, 2 * sizeof(struct vlan_hdr));

	TGEN_ASSERT(pqinq);
	struct ipv6_hdr *pip6 = (struct ipv6_hdr *)(pqinq + 1);

	if (pip6->hop_limits) {
		pip6->hop_limits--;
	}
	else {
		mprintf("TTL = 0 => Dropping\n");
		rte_pktmbuf_free(rx_mbuf);
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		return;
	}

	struct rte_hash_ext ext;
	int32_t hash_index = rte_hash_ext_lookup(ptask->cpe_table, (const void *)pip6->dst_addr, &ext);

	struct cpe_table *table = (struct cpe_table *)((uint8_t *)ext.bucket_entry + 16);
	if (unlikely(hash_index < 0)) {
#ifdef ENABLE_VERBOSITY
		if (unlikely(verbose)) {
			mprintf("core Unknown IP " IPv6_BYTES_FMT "\n", IPv6_BYTES(pip6->dst_addr));
		}
#endif
		rte_pktmbuf_free(rx_mbuf);
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		return;
	}
	else if (unlikely(hash_index >= MAX_GRE)) {
		mprintf("Invalid hash_index = %x\n", hash_index);
		rte_pktmbuf_free(rx_mbuf);
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		return;
	}

	/* will also overwrite part of the destination addr */
	(*(uint64_t *)pqinq) = table->mac_port_8bytes;
	pqinq->svlan.eth_proto = ETYPE_8021ad;
	pqinq->cvlan.eth_proto = ETYPE_VLAN;
	pqinq->svlan.vlan_tci = table->qinq_svlan;
	pqinq->cvlan.vlan_tci = table->qinq_cvlan;
	pqinq->ether_type = ETYPE_IPv6;

	/* classification can only be done from this point */
	if (ptask->runtime_flags & TASK_CLASSIFY) {
		classify_packet(rx_mbuf, table->user);
	}

	tx_buf_pkt_single(&ptask->base, rx_mbuf, 0);
}
