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

#include <rte_byteorder.h>
#include <rte_cycles.h>
#include <rte_hash.h>

#include "handle_qinq_decapv4.h"
#include "display.h"
#include "stats.h"
#include "tx_pkt.h"
#include "defines.h"
#include "gre_encap_decap.h"
#include "handle_routing.h"
#include "tgen_assert.h"

static inline void handle_qinq_decap_v4(struct rte_mbuf *rx_mbuf, struct task_qinq_decap_v4 *ptask);

void handle_qinq_decap_v4_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_qinq_decap_v4 *task = (struct task_qinq_decap_v4 *)ptask;
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
		handle_qinq_decap_v4(rx_mbuf[j], task);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		handle_qinq_decap_v4(rx_mbuf[j], task);
	}
#endif
	task->tx_pkt(&task->base);
}

#ifdef MPLS_ROUTING
#define SIZE_DELTA (20 + (unsigned)sizeof(struct mpls_hdr))
#define VLAN_MACIP_DATA ((sizeof(struct ether_hdr) + sizeof(struct mpls_hdr)) << 9) | (sizeof(struct ipv4_hdr))
#else
#define SIZE_DELTA 20
#define VLAN_MACIP_DATA (sizeof(struct ether_hdr) << 9) | (sizeof(struct ipv4_hdr))
#endif

/* Struct used for setting all the values a packet
   going to the core netwerk. Payload may follow
   after the headers, but no need to touch that. */
struct core_net_pkt {
	struct ether_hdr ether_hdr;
#ifdef MPLS_ROUTING
	uint32_t mpls;
#endif
	struct ipv4_hdr tunnel_ip_hdr;
	struct gre_hdr gre_hdr;
	struct ipv4_hdr ip_hdr;
} __attribute__((packed));

struct cpe_pkt {
	struct qinq_hdr qinq_hdr;
	struct ipv4_hdr ipv4_hdr;
} __attribute__((packed));

static inline uint8_t gre_encap_route(struct lcore_cfg *lconf, struct rte_mbuf *rx_mbuf, uint32_t gre_id, struct task_qinq_decap_v4 *ptask)
{
	struct core_net_pkt *packet = (struct core_net_pkt *)rte_pktmbuf_prepend(rx_mbuf, SIZE_DELTA);
	TGEN_ASSERT(packet);
	PREFETCH0(packet);

	const struct ipv4_hdr *pip = (const struct ipv4_hdr *)(SIZE_DELTA + ((uint8_t *)packet) + sizeof(struct qinq_hdr));
	const uint16_t ip_len = rte_be_to_cpu_16(pip->total_length);

	uint8_t next_hop_index;
	/* returns 0 on sucess, returns -ENOENT of failure (or -EINVAL if first or last parameter is NULL) */
	if (unlikely(rte_lpm_lookup(ptask->ipv4_lpm, rte_bswap32(pip->dst_addr), &next_hop_index) != 0)) {
		mprintf("lpm_lookup failed for ip %x: rc = %d\n", rte_bswap32(pip->dst_addr), -ENOENT);
		return ROUTE_ERR;
	}
	PREFETCH0(&ptask->next_hop[next_hop_index]);

	/* calculate IP CRC here to avoid problems with -O3 flag with gcc */
#ifdef SOFT_CRC
	p_tunnel_ip->ip_sum = tgen_ip_cksum((void *)p_tunnel_ip, sizeof(struct ipv4_hdr), 1);
#elif defined(HARD_CRC)
	rx_mbuf->pkt.vlan_macip.data = VLAN_MACIP_DATA;
	rx_mbuf->ol_flags |= PKT_TX_IP_CKSUM;
#endif

	const uint16_t padlen = rte_pktmbuf_pkt_len(rx_mbuf) - SIZE_DELTA - ip_len - sizeof(struct qinq_hdr);

	if (padlen) {
		rte_pktmbuf_trim(rx_mbuf, padlen);
	}

	const uint8_t port = ptask->next_hop[next_hop_index].mac_port.port;

	*((uint64_t *)(&packet->ether_hdr.d_addr)) = ptask->next_hop[next_hop_index].mac_port_8bytes;
	*((uint64_t *)(&packet->ether_hdr.s_addr)) = *((uint64_t *)&if_cfg[ptask->base.tx_params_hw.tx_port_queue[port].port]);



#ifdef MPLS_ROUTING
	packet->mpls = ptask->next_hop[next_hop_index].mpls | 0x00010000; // Set BoS to 1
#endif


#if !(__GNUC__ == 4 && __GNUC_MINOR__ < 6)
#ifdef MPLS_ROUTING
	packet->ether_hdr.ether_type = ETYPE_MPLSU;
#else
	packet->ether_hdr.ether_type = ETYPE_IPv4;
#endif
#endif

	TGEN_PANIC(rte_pktmbuf_data_len(rx_mbuf) + SIZE_DELTA  > ETHERNET_MAX_LEN,
	           "Would need to fragment packet new size = %u - not implemented\n",
	           rte_pktmbuf_data_len(rx_mbuf) + SIZE_DELTA);

	/* New IP header */
	rte_memcpy(&packet->tunnel_ip_hdr, &tunnel_ip_proto, sizeof(struct ipv4_hdr));
	packet->tunnel_ip_hdr.total_length = rte_cpu_to_be_16(ip_len + 28);
	packet->tunnel_ip_hdr.src_addr = lconf->bras_ip;

	/* Add GRE Header values */
	rte_memcpy(&packet->gre_hdr, &gre_hdr_proto, sizeof(struct gre_hdr));
	packet->gre_hdr.gre_id = rte_be_to_cpu_32(gre_id);

#if __GNUC__ == 4 && __GNUC_MINOR__ < 6
#ifdef MPLS_ROUTING
	packet->ether_hdr.ether_type = ETYPE_MPLSU;
#else
	packet->ether_hdr.ether_type = ETYPE_IPv4;
#endif
#endif
	return port;
}

void handle_qinq_decap_v4(struct rte_mbuf *rx_mbuf, struct task_qinq_decap_v4 *ptask)
{
	struct cpe_pkt *packet = rte_pktmbuf_mtod(rx_mbuf, struct cpe_pkt *);

	const uint32_t svlan = packet->qinq_hdr.svlan.vlan_tci & 0xFF0F;
	const uint32_t cvlan = packet->qinq_hdr.cvlan.vlan_tci & 0xFF0F;

	const uint32_t qinq2 = svlan << 16 | cvlan;

	struct rte_hash_ext ext;
	int32_t hash_index = rte_hash_ext_lookup(ptask->qinq_gre, (const void *)&qinq2, &ext);
	struct qinq_to_gre_table *table = &((struct qinq_to_gre_table_hash_entry *)ext.bucket_entry)->data;
	if (unlikely((hash_index < 0) || (hash_index > MAX_GRE))) {
		mprintf("Error converting qinq %x to gre_id: %d\n", (rte_be_to_cpu_16(svlan) << 12) | rte_be_to_cpu_16(cvlan), hash_index);
		rte_pktmbuf_free(rx_mbuf);
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		return;
	}
	if ((ptask->runtime_flags & TASK_UPDATE_MAC_ONLY_ARP) == 0) {
		struct cpe_table_hash_entry entry;
		entry.key.ip = packet->ipv4_hdr.src_addr;
		entry.key.gre_id = table->gre_id;
		entry.data.mac_port_8bytes = *((const uint64_t *)(&packet->qinq_hdr.s_addr));
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
	}

	if (ptask->runtime_flags & TASK_ROUTING) {
		uint8_t tx_portid;
		if (ROUTE_ERR == (tx_portid = gre_encap_route(ptask->lconf, rx_mbuf, table->gre_id, ptask))) {
			rte_pktmbuf_free(rx_mbuf);
			INCR_TX_DROP_COUNT(ptask->base.stats, 1);
			return;
		}

		DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "TX", rx_mbuf);
		tx_buf_pkt_single(&ptask->base, rx_mbuf, tx_portid);
	}
	else {
		gre_encap(ptask->lconf, rx_mbuf, table->gre_id);
		DEBUG_TCPDUMP_PACKET(&ptask->base.task_debug, "TX", rx_mbuf);
		tx_buf_pkt_single(&ptask->base, rx_mbuf, 0);
	}
}
