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
#include "clock.h"
#include "rx_pkt.h"
#include "stats.h"

uint16_t rx_pkt_hw(struct rte_mbuf **rx_mbuf, struct task_base *ptask)
{
	START_EMPTY_MEASSURE();
#ifdef BRAS_RX_BULK
	uint16_t nb_rx = rte_eth_rx_burst(ptask->rx_params_hw.rx_port, ptask->rx_params_hw.rx_queue, rx_mbuf + ptask->rx_params_hw.nb_rxbulk, MAX_PKT_BURST - ptask->rx_params_hw.nb_rxbulk);
	if (likely(nb_rx > 0)) {
		ptask->rx_params_hw.nb_rxbulk += nb_rx;
		if (ptask->rx_params_hw.nb_rxbulk == MAX_PKT_BURST) {
			ptask->rx_params_hw.nb_rxbulk = 0;
			return MAX_PKT_BURST;
		}
		else {
			/* Don't increment EMPTY cycles. */
			return 0;
		}
	}
#else
	uint16_t nb_rx = rte_eth_rx_burst(ptask->rx_params_hw.rx_port, ptask->rx_params_hw.rx_queue, rx_mbuf, MAX_PKT_BURST);
	if (likely(nb_rx > 0)) {
		return nb_rx;
	}
#endif
	INCR_EMPTY_CYCLES(ptask->stats, rte_rdtsc() - cur_tsc);
	return 0;
}

uint16_t rx_pkt_sw(struct rte_mbuf **rx_mbuf, struct task_base *ptask)
{
	START_EMPTY_MEASSURE();
#ifdef BRAS_RX_BULK
	if (unlikely (rte_ring_sc_dequeue_bulk(ptask->rx_params_sw.rx_rings[ptask->rx_params_sw.last_read_ring], (void **)rx_mbuf, MAX_RING_BURST)) < 0) {
		++ptask->rx_params_sw.last_read_ring;
		if (unlikely(ptask->rx_params_sw.last_read_ring == ptask->rx_params_sw.nb_rxrings)) {
			ptask->rx_params_sw.last_read_ring = 0;
		}
		INCR_EMPTY_CYCLES(ptask->stats, rte_rdtsc() - cur_tsc);
		return 0;
	}
	else {

		return MAX_RING_BURST;
	}
#else
	uint16_t nb_rx = rte_ring_sc_dequeue_burst(ptask->rx_params_sw.rx_rings[ptask->rx_params_sw.last_read_ring], (void **)rx_mbuf, MAX_RING_BURST);
	++ptask->rx_params_sw.last_read_ring;
	if (unlikely(ptask->rx_params_sw.last_read_ring == ptask->rx_params_sw.nb_rxrings)) {
		ptask->rx_params_sw.last_read_ring = 0;
	}

	if (nb_rx != 0) {
		return nb_rx;
	}
	else {
		INCR_EMPTY_CYCLES(ptask->stats, rte_rdtsc() - cur_tsc);
		return 0;
	}
#endif
}
