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

#ifndef _HANDLE_ROUTING_H_
#define _HANDLE_ROUTING_H_

#include <rte_lpm.h>

#include "defines.h"
#include "hash_entry_types.h"
#include "mpls.h"
#include "prefetch.h"

struct task_routing {
	struct task_base                base;
	void (*tx_pkt)(struct task_base *ptask);
	struct ether_addr 		edaddr;
	uint8_t			        tx_portid;
	struct lcore_cfg                *lconf;
	struct next_hop_struct          *next_hop;
	struct rte_lpm                  *ipv4_lpm;
};

/* LPM Routing based on IPv4 routing table */
static inline uint8_t route(struct rte_mbuf *rx_mbuf, struct next_hop_struct *nh, struct rte_lpm* ipv4_lpm, const struct ipv4_hdr *pip)
{
	uint8_t next_hop_index;
	if (unlikely(rte_lpm_lookup(ipv4_lpm, rte_bswap32(pip->dst_addr), &next_hop_index) != 0)) {
		mprintf("lpm_lookup failed for ip %x: rc = %d\n", pip->dst_addr, -ENOENT);
		return ROUTE_ERR;
	}
	prefetch_nta(&nh[next_hop_index]);

#ifdef MPLS_ROUTING
	struct ether_hdr *peth = mpls_encap(rx_mbuf, nh[next_hop_index].mpls);
#else
	struct ether_hdr *peth = rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *);
#endif

	*((uint64_t *)(&peth->d_addr)) = nh[next_hop_index].mac_port_8bytes;
	uint8_t port = nh[next_hop_index].mac_port.port;
	//*((uint64_t *)(&peth->s_addr)) = *((uint64_t *)&if_cfg[ptask->tx_params_hw.tx_port_queue[port].port]);
	peth->ether_type = ETYPE_MPLSU;
	return port;
}

void handle_routing_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts);

#endif /* _HANDLE_ROUTING_H_ */
