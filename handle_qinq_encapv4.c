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

#include "handle_qinq_encapv4.h"
#include "defines.h"
#include "classify.h"
#include "stats.h"
#include "tx_pkt.h"
#include "gre_encap_decap.h"
#include "hash_entry_types.h"

static inline void restore_qinq(struct qinq_hdr *pqinq, struct cpe_table *table);

static inline void handle_qinq_encap_v4(struct rte_mbuf *rx_mbuf, struct task_qinq_encap_v4 *ptask);

void handle_qinq_encap_v4_untag_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_qinq_encap_v4 *port = (struct task_qinq_encap_v4 *)ptask;
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
			handle_qinq_encap_v4(rx_mbuf[j], port);
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
			handle_qinq_encap_v4(rx_mbuf[j], port);
		}
		else {
			INCR_TX_DROP_COUNT(port->base.stats, 1);
			rte_pktmbuf_free(rx_mbuf[j]);
		}
	}
#endif
	port->tx_pkt(&port->base);
}

void handle_qinq_encap_v4_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_qinq_encap_v4 *port = (struct task_qinq_encap_v4 *)ptask;
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
		handle_qinq_encap_v4(rx_mbuf[j], port);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		handle_qinq_encap_v4(rx_mbuf[j], port);
	}
#endif
	port->tx_pkt(&port->base);
}

static inline void handle_qinq_encap_v4(struct rte_mbuf *rx_mbuf, struct task_qinq_encap_v4 *ptask)
{
	struct ipv4_hdr *pip = (struct ipv4_hdr *)(rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *) + 1);
	struct hash_gre_struct key;

#ifdef HARD_CRC
	rx_mbuf->pkt.vlan_macip.data = (sizeof(struct ether_hdr) << 9) | (sizeof(struct ipv4_hdr));
	rx_mbuf->ol_flags |= PKT_TX_IP_CKSUM;
#endif

	key.gre_id = gre_decap(&pip, rx_mbuf);
	TGEN_ASSERT(pip);
	key.ip = pip->dst_addr;

	struct rte_hash_ext ext;
	int32_t hash_index = rte_hash_ext_lookup(ptask->cpe_table, (const void *)&key, &ext);
	struct cpe_table *table = &((struct cpe_table_hash_entry *)ext.bucket_entry)->data;

	if (unlikely(hash_index < 0)) {
#ifdef ENABLE_VERBOSITY
		if (verbose) {
			mprintf("Core Unknown IP %x/gre_id %x\n", key.ip, key.gre_id);
		}
#endif
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}
	else if (unlikely(hash_index >= MAX_GRE)) {
		mprintf("Invalid hash_index = %x\n", hash_index);
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}

	if (pip->time_to_live) {
		pip->time_to_live--;
	}
	else {
		mprintf("TTL = 0 => Dropping\n");
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}

	restore_qinq(rte_pktmbuf_mtod(rx_mbuf, struct qinq_hdr *), table);
	if (ptask->runtime_flags & TASK_CLASSIFY) {
		classify_packet(rx_mbuf, table->user);
	}

	pip->hdr_checksum = 0;

#ifndef HARD_CRC
#ifdef SOFT_CRC
	pip->ip_sum = tgen_ip_cksum((void *)pip, sizeof(struct ipv4_hdr), 0);
#endif
#endif

	DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "TX", rx_mbuf);
	tx_buf_pkt_single(&ptask->base, rx_mbuf, ptask->mapping[table->mac_port.port]);
}

static inline void restore_qinq(struct qinq_hdr *pqinq, struct cpe_table *table)
{
	rte_memcpy(pqinq, &qinq_proto, sizeof(struct qinq_hdr));
	(*(uint64_t *)(&pqinq->d_addr)) = table->mac_port_8bytes;
	/* set source as well now */
	*((uint64_t *)(&pqinq->s_addr)) = *((uint64_t *)&if_cfg[table->mac_port.port]);
	pqinq->svlan.vlan_tci = table->qinq_svlan;
	pqinq->cvlan.vlan_tci = table->qinq_cvlan;
	pqinq->svlan.eth_proto = ETYPE_8021ad;
	pqinq->cvlan.eth_proto = ETYPE_VLAN;
	pqinq->ether_type = ETYPE_IPv4;
}
