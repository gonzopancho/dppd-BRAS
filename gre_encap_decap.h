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

#ifndef _GRE_ENCAP_DECAP_H_
#define _GRE_ENCAP_DECAP_H_

#include <rte_mbuf.h>
#include "tgen_assert.h"

#include "pkt_prototypes.h"
#include "quit.h"

/* add gre header */
static inline void gre_encap(struct lcore_cfg *lconf, struct rte_mbuf *rx_mbuf, uint32_t gre_id)
{
	const struct ipv4_hdr *pip = (const struct ipv4_hdr *)(1 + rte_pktmbuf_mtod(rx_mbuf, const struct qinq_hdr *));
	uint16_t ip_len = rte_be_to_cpu_16(pip->total_length);


	uint16_t padlen = rte_pktmbuf_pkt_len(rx_mbuf) - 20 - ip_len - sizeof(struct qinq_hdr);

	if (padlen) {
		rte_pktmbuf_trim(rx_mbuf, padlen);
	}


	/* prepend only 20 bytes instead of 28, 8 bytes are present from the QinQ */
	struct ether_hdr *peth = (struct ether_hdr *)rte_pktmbuf_prepend(rx_mbuf, 20);
	TGEN_ASSERT(peth);
	PREFETCH0(peth);
	/* calculate IP CRC here to avoid problems with -O3 flag with gcc */
#ifdef SOFT_CRC
	p_tunnel_ip->ip_sum = tgen_ip_cksum((void *)p_tunnel_ip, sizeof(struct ipv4_hdr), 0);
#elif defined(HARD_CRC)
	rx_mbuf->pkt.vlan_macip.data = (sizeof(struct ether_hdr) << 9) | (sizeof(struct ipv4_hdr));
	rx_mbuf->ol_flags |= PKT_TX_IP_CKSUM;
#endif


	TGEN_PANIC(rte_pktmbuf_data_len(rx_mbuf) - padlen + 20 > ETHERNET_MAX_LEN,
	           "Would need to fragment packet new size = %u - not implemented\n",
	           rte_pktmbuf_data_len(rx_mbuf) - padlen + 20);


	/* new IP header */
	struct ipv4_hdr *p_tunnel_ip = (struct ipv4_hdr *)(peth + 1);
	rte_memcpy(p_tunnel_ip, &tunnel_ip_proto, sizeof(struct ipv4_hdr));
	p_tunnel_ip->total_length = rte_cpu_to_be_16(ip_len + 28);
	p_tunnel_ip->src_addr = lconf->bras_ip;

	/* Add GRE Header values */
	struct gre_hdr *pgre = (struct gre_hdr *)(p_tunnel_ip + 1);

	rte_memcpy(pgre, &gre_hdr_proto, sizeof(struct gre_hdr));
	pgre->gre_id = gre_id;
	peth->ether_type = ETYPE_IPv4;
}

#define GRE_DECAP_SIZE_DIFF (sizeof(struct gre_hdr) + sizeof(struct ipv4_hdr) - 2 * sizeof(struct vlan_hdr))
/* remove gre header and return gre id retrieves from the removed header */
static inline int32_t gre_decap(struct ipv4_hdr **pip, struct rte_mbuf *rx_mbuf)
{
	// GRE Decapsulation
	int32_t gre_id = rte_be_to_cpu_32(((struct gre_hdr *)(*pip + 1))->gre_id) & 0xFFFFFFF;
	*pip = (struct ipv4_hdr *)(rte_pktmbuf_adj(rx_mbuf, GRE_DECAP_SIZE_DIFF) + sizeof(struct qinq_hdr));
	return gre_id;
}

#endif /* _GRE_ENCAP_DECAP_H_ */
