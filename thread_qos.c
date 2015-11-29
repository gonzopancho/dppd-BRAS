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

#include "tgen_assert.h"
#include "handle_qos.h"
#include "thread_qos.h"
#include "control.h"
#include "display.h"

int thread_qos(struct lcore_cfg *lconf)
{
	struct rte_mbuf *rx_mbuf[MAX_RING_BURST] __rte_cache_aligned;
	struct task_qos *ptask = (struct task_qos *)lconf->task[0]; /* for QoS only one port per core */
	uint64_t cur_tsc = rte_rdtsc();
	uint64_t next_term_tsc = cur_tsc + TERM_TIMEOUT;
	uint64_t drain_tsc = cur_tsc + DRAIN_TIMEOUT;
	struct rte_mbuf *qos_mbuf[32];

	for (;;) {
		cur_tsc = rte_rdtsc();
		if (cur_tsc > drain_tsc) {
			drain_tsc = cur_tsc + DRAIN_TIMEOUT;
			FLUSH_STATS(lconf);
			/* check for termination request every timeout */
			if (cur_tsc > next_term_tsc) {
				next_term_tsc = cur_tsc + TERM_TIMEOUT;
				if (is_terminated(lconf)) {
					break;
				}
			}
		}

		uint16_t nb_rx = ptask->base.rx_pkt(rx_mbuf, &ptask->base);
		if (likely(nb_rx)) {
			TGEN_ASSERT(nb_rx <= MAX_RING_BURST);

			INCR_NBRX(nb_rx);
			INCR_RX_PKT_COUNT(ptask->base.stats, nb_rx);
			HandleQoSBulk(rx_mbuf, &ptask->base, nb_rx);
		}

		if (ptask->nb_buffered_pkts) {
			nb_rx = rte_sched_port_dequeue(ptask->sched_port, qos_mbuf, 32);
			if (likely(nb_rx)) {
				ptask->nb_buffered_pkts -= nb_rx;
				DEBUG_TCPDUMP_PACKET_BULK(&ptask->base.task_debug, "TX", qos_mbuf, nb_rx);
				ptask->base.tx_pkt_no_buf(qos_mbuf, &ptask->base, nb_rx);
			}
		}
	}
	return 0;
}
