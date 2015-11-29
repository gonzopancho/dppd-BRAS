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

#include "handle_qinq_decaparp.h"

#include <rte_common.h>
#include <rte_hash.h>
#include <rte_byteorder.h>
#include <rte_ip.h>
#include <rte_cycles.h>

#include "quit.h"
#include "display.h"
#include "qinq.h"
#include "tgen_args.h"
#include "defines.h"
#include "classify.h"
#include "stats.h"
#include "control.h"
#include "arp.h"
#include "hash_entry_types.h"

static inline void handle_qinq_decap_arp(struct rte_mbuf *rx_mbuf, struct task_qinq_decap_arp *ptask);

void handle_qinq_decap_arp_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
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
		handle_qinq_decap_arp(rx_mbuf[j], (struct task_qinq_decap_arp *)ptask);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		handle_qinq_decap_arp(rx_mbuf[j], (struct task_qinq_decap_arp *)ptask);
	}
#endif
}

struct cpe_packet_arp {
	struct qinq_hdr qinq;
	struct my_arp_t arp;
} __attribute__((packed));

static inline void handle_qinq_decap_arp(struct rte_mbuf *rx_mbuf, struct task_qinq_decap_arp *ptask)
{
	const struct cpe_packet_arp *packet = rte_pktmbuf_mtod(rx_mbuf, const struct cpe_packet_arp *);

	uint32_t svlan = packet->qinq.svlan.vlan_tci & 0xFF0F;
	uint32_t cvlan = packet->qinq.cvlan.vlan_tci & 0xFF0F;

	uint32_t qinq2 = svlan << 16 | cvlan;

	struct rte_hash_ext ext2;
	int32_t hash_index = rte_hash_ext_lookup(ptask->qinq_gre, (const void *)&qinq2, &ext2);
	if (unlikely(hash_index < 0 || hash_index > MAX_GRE)) {
		mprintf("Error converting qinq %x to gre_id: %d\n", (rte_be_to_cpu_16(svlan) << 12) | rte_be_to_cpu_16(cvlan), hash_index);
		rte_pktmbuf_free(rx_mbuf);
		return;
	}

	struct qinq_to_gre_table *table = &((struct qinq_to_gre_table_hash_entry *)ext2.bucket_entry)->data;
	struct cpe_table_hash_entry entry;
	entry.key.ip = packet->arp.data.spa;
	entry.key.gre_id = table->gre_id;
	entry.data.mac_port_8bytes = *((const uint64_t *)(&packet->qinq.s_addr));
	entry.data.qinq_svlan = svlan;
	entry.data.qinq_cvlan = cvlan;
	entry.data.mac_port.port = rx_mbuf->pkt.in_port;
	entry.data.user = table->user;
	entry.data.tsc = rte_rdtsc() + ARP_TIMEOUT;

	struct rte_hash_ext ext;
	hash_index = rte_hash_ext_add_entry(ptask->cpe_table, &entry, &ext);
	if (unlikely(hash_index < 0)) {
		static int has_printed = 0;
		if (has_printed == 0) {
			has_printed = 1;
			mprintf("Failed to add key %x, gre %x\n", entry.key.ip, entry.key.gre_id);
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

	/* Should do ARP Reply (not implemented) ... */
	rte_pktmbuf_free(rx_mbuf);
}
