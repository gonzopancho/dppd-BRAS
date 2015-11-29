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
#include "handle_lb_network.h"

#include "defines.h"
#include "tgen_args.h"
#include "tx_pkt.h"
#include "display.h"
#include "stats.h"
#include "quit.h"
#include "mpls.h"
#include "etypes.h"
#include "gre.h"

static inline uint8_t handle_lb_network(struct rte_mbuf *rx_mbuf, struct task_lb_network *ptask, uint8_t *dest_wt);

void handle_lb_network_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_lb_network *port = (struct task_lb_network *)ptask;
	uint16_t not_dropped = 0;
	uint8_t dest_wt[MAX_RING_BURST];
	// process packet, i.e. decide if the packet has to be dropped or not and where the packet has to go
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
		rx_mbuf[not_dropped] = rx_mbuf[j];
		not_dropped += handle_lb_network(rx_mbuf[j], port, &dest_wt[not_dropped]);
	}
#ifdef BRAS_PREFETCH_OFFSET
	PREFETCH0(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct ether_hdr *));
	for (; (j < n_pkts); ++j) {
		rx_mbuf[not_dropped] = rx_mbuf[j];
		not_dropped += handle_lb_network(rx_mbuf[j], port, &dest_wt[not_dropped]);
	}
#endif

	/* rx_mbuf now contains the packets that have not been dropped */
	if (likely(not_dropped)) {
		for (j = 0; j < not_dropped; ++j) {
			tx_buf_pkt_single(&port->base, rx_mbuf[j], dest_wt[j]);
		}
		port->tx_pkt(&port->base);
	}
}

static inline uint8_t worker_from_mask(uint32_t val, struct task_lb_network *ptask)
{
	if (ptask->bit_mask != 0xff) {
		return val & ptask->bit_mask;
	}
	else {
		return val % ptask->nb_worker_threads;
	}
}

static inline struct ipv4_hdr *get_ip_hdr(struct rte_mbuf *rx_mbuf)
{
	struct ether_hdr *peth = rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *);

	switch (peth->ether_type) {
	case ETYPE_MPLSU: {
		/* Skip MPLS header */
		struct mpls_hdr *mpls = (struct mpls_hdr *)(peth + 1);
		if (mpls->bos == 0) {
			mpls++;
			TGEN_PANIC(mpls->bos == 0, "More than 2 MPLS stacks - unsupported\n");
		}
		return (struct ipv4_hdr *)(mpls + 1);
	}
	case ETYPE_IPv4:
	case ETYPE_IPv6:
		return (struct ipv4_hdr *)(peth + 1);
	case ETYPE_LLDP:
		rte_pktmbuf_free(rx_mbuf);
		return NULL;
	default:
		mprintf("Unexpected frame Ether type = %#06x\n", peth->ether_type);
		rte_pktmbuf_free(rx_mbuf);
		return NULL;
	}
}

static inline uint8_t handle_lb_network(struct rte_mbuf *rx_mbuf, struct task_lb_network *ptask, uint8_t *dest_wt)
{
	struct ipv4_hdr *pip = get_ip_hdr(rx_mbuf);

	if (unlikely(!pip)) {
		INCR_TX_DROP_COUNT(ptask->base.stats, 1);
		return 0;
	}

	uint8_t proto = 0xff;
	uint8_t worker = 0;
	if ((pip->version_ihl >> 4) == 4) {
		if (pip->next_proto_id == IPPROTO_GRE) {
			const struct gre_hdr *pgre = (const struct gre_hdr *)(pip + 1);

			if (pgre->bits & GRE_KEY_PRESENT) {
				uint32_t gre_id;
				if (pgre->bits & (GRE_CRC_PRESENT | GRE_ROUTING_PRESENT)) {
					gre_id = *((const uint32_t *)((const uint8_t *)pgre + 8));
				}
				else {
					gre_id = *((const uint32_t *)((const uint8_t *)pgre + 4));
				}

				gre_id = rte_be_to_cpu_32(gre_id) & 0xFFFFFFF;
				worker = worker_from_mask(gre_id, ptask);
#ifdef ENABLE_VERBOSITY
				if (verbose == 2) {
					mprintf("gre_id = %u worker = %u\n", gre_id, worker);
				}
#endif
			}
			else {
				mprintf("Key not present\n");
				worker = 0;
			}
			proto = IPV4;
		}
		else {
			worker = *((uint8_t *)pip + ptask->worker_byte_offset_ipv4);
			worker = worker_from_mask(worker, ptask);
		}
	}
	else if ((pip->version_ihl >> 4) == 6) {
		worker = *((uint8_t *)pip + ptask->worker_byte_offset_ipv6);
		worker = worker_from_mask(worker, ptask);
		proto = IPV6;
	}
	else {
		TGEN_PANIC(1, "Non IPv4 or IPv6 frame\n");
	}

	if (ptask->runtime_flags & TASK_MPLS_TAGGING) {
		struct ether_hdr *peth = rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *);
		if (peth->ether_type == ETYPE_MPLSU) {
			proto = mpls_decap(rx_mbuf);
		}
	}
	*dest_wt = worker + ptask->nb_worker_threads * proto;
	return 1;
}
