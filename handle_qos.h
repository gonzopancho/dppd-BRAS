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

#ifndef _HANDLE_QOS_H_
#define _HANDLE_QOS_H_

#include <rte_mbuf.h>

#include "stats.h"
#include "qinq.h"
#include "classify.h"
#include "prefetch.h"
#include "defines.h"

struct lcore_cfg;

struct task_qos {
	struct task_base base;
	struct rte_sched_port *sched_port;
	uint16_t *user_table;
	uint32_t nb_buffered_pkts;
	uint8_t runtime_flags;
};

static inline void HandleQoSBulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts)
{
	struct task_qos *port = (struct task_qos *)ptask;
	if (port->runtime_flags & TASK_CLASSIFY) {
		uint16_t j;
#ifdef BRAS_PREFETCH_OFFSET
		for (j = 0; (j < BRAS_PREFETCH_OFFSET) && (j < n_pkts); ++j) {
			prefetch_nta(rx_mbuf[j]);
		}
		for (j = 1; (j < BRAS_PREFETCH_OFFSET) && (j < n_pkts); ++j) {
			prefetch_nta(rte_pktmbuf_mtod(rx_mbuf[j - 1], struct qinq_hdr *));
		}
#endif
		for (j = 0; j + PREFETCH_OFFSET < n_pkts; ++j) {
			prefetch_nta(rx_mbuf[j + PREFETCH_OFFSET]);
			prefetch_nta(rte_pktmbuf_mtod(rx_mbuf[j + PREFETCH_OFFSET - 1], struct qinq_hdr *));
			const struct qinq_hdr *pqinq = rte_pktmbuf_mtod(rx_mbuf[j], const struct qinq_hdr *);
			uint32_t qinq = PKT_TO_LUTQINQ(pqinq->svlan.vlan_tci, pqinq->cvlan.vlan_tci);
			classify_packet(rx_mbuf[j], port->user_table[qinq]);
		}
#ifdef BRAS_PREFETCH_OFFSET
		prefetch_nta(rte_pktmbuf_mtod(rx_mbuf[n_pkts - 1], struct qinq_hdr *));
		for (; (j < n_pkts); ++j) {
			const struct qinq_hdr *pqinq = rte_pktmbuf_mtod(rx_mbuf[j], const struct qinq_hdr *);
			uint32_t qinq = PKT_TO_LUTQINQ(pqinq->svlan.vlan_tci, pqinq->cvlan.vlan_tci);
			classify_packet(rx_mbuf[j], port->user_table[qinq]);
		}
#endif
	}
	int16_t ret = rte_sched_port_enqueue(port->sched_port, rx_mbuf, n_pkts);
	port->nb_buffered_pkts += ret;
	INCR_TX_DROP_COUNT(port->base.stats, n_pkts - ret);
}

#endif /* _HANDLE_QOS_H_ */
