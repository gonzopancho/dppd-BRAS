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

#include <rte_cycles.h>
#include <rte_hash.h>

#include "handle_qinq_decapv6.h"
#include "display.h"
#include "classify.h"
#include "stats.h"
#include "tx_pkt.h"
#include "handle_routing.h"
#include "pkt_prototypes.h"
#include "tgen_assert.h"

static inline void handle_qinq_decap_v6(struct rte_mbuf *rx_mbuf, struct task_qinq_decap_v6 *ptask);

void handle_qinq_decap_v6_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_qinq_decap_v6 *port = (struct task_qinq_decap_v6 *)ptask;
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
		handle_qinq_decap_v6(rx_mbuf[j], port);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		handle_qinq_decap_v6(rx_mbuf[j], port);
	}
#endif
	port->tx_pkt(&port->base);
}

static inline void handle_qinq_decap_v6(struct rte_mbuf *rx_mbuf, struct task_qinq_decap_v6 *ptask)
{
	struct qinq_hdr *pqinq = rte_pktmbuf_mtod(rx_mbuf, struct qinq_hdr *);
	struct ipv6_hdr *pip6 = (struct ipv6_hdr *)(pqinq + 1);

	DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "TX", rx_mbuf);

	struct cpe_table_hash_entry entry;
	uint16_t svlan = pqinq->svlan.vlan_tci & 0xFF0F;
	uint16_t cvlan = pqinq->cvlan.vlan_tci & 0xFF0F;

	rte_memcpy(&entry.key, pip6->src_addr, sizeof(struct hash_gre_struct));
	entry.data.mac_port_8bytes = *((uint64_t *)(((uint8_t *)pqinq) + 5)) << 16;
	entry.data.qinq_svlan = svlan;
	entry.data.qinq_cvlan = cvlan;
	entry.data.user = ptask->user_table[PKT_TO_LUTQINQ(svlan, cvlan)];
	entry.data.tsc = rte_rdtsc() + ARP_TIMEOUT;

	struct rte_hash_ext ext;
	int32_t hash_index = rte_hash_ext_add_entry(ptask->cpe_table, &entry, &ext);

	if (unlikely(hash_index < 0)) {
		static int has_printed6 = 0;
		if (has_printed6 == 0) {
			has_printed6 = 1;
			mprintf("Failed to add key " IPv6_BYTES_FMT "\n", IPv6_BYTES(pip6->src_addr));
		}
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}
	else if (unlikely(hash_index >= MAX_GRE)) {
		mprintf("Invalid hash_index = 0x%x\n", hash_index);
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}

	pqinq = (struct qinq_hdr *)rte_pktmbuf_adj(rx_mbuf, 2 * sizeof(struct vlan_hdr));
	TGEN_ASSERT(pqinq);
	pqinq->ether_type = ETYPE_IPv6;
	// Dest MAC addresses
	ether_addr_copy(&ptask->edaddr, &pqinq->d_addr);
	tx_buf_pkt_single(&ptask->base, rx_mbuf, 0);
}
