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

#ifndef _HANDLE_QINQ_ENCAPV4_H_
#define _HANDLE_QINQ_ENCAPV4_H_

#include "mpls.h"
#include "stats.h"

struct task_qinq_encap_v4 {
	struct task_base base;
	void (*tx_pkt)(struct task_base *ptask);
	struct rte_hash  *cpe_table;
	uint8_t          runtime_flags;
	uint8_t          mapping[TGEN_MAX_PORTS];
};

/* Encapsulate IPv4 packets in QinQ. QinQ tags are derived from gre_id. */
void handle_qinq_encap_v4_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts);

/* Same functionality as handle_qinq_encap_v4_bulk but untag MPLS as well. */
void handle_qinq_encap_v4_untag_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts);

static inline uint8_t untag_or_drop(struct rte_mbuf *rx_mbuf)
{
	uint16_t eth_type = rte_pktmbuf_mtod(rx_mbuf, struct ether_hdr *)->ether_type;
	if (eth_type == ETYPE_MPLSU) {
		mpls_decap(rx_mbuf);
		return 1;
	}
	else {
		if (eth_type != ETYPE_LLDP) {
			mprintf("Error Removing MPLS: ether_type = %#06x\n", eth_type);
		}
		return 0;
	}
}

#endif /* _HANDLE_QINQ_ENCAPV4_H_ */
