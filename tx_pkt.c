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

#include <rte_ethdev.h>

#include "tx_pkt.h"
#include "stats.h"
#include "prefetch.h"
#include "tgen_assert.h"

void tx_buf_pkt_bulk(struct task_base *ptask, struct rte_mbuf **rx_mbuf, const uint16_t n_pkts, const uint8_t tx_portid)
{
	const uint16_t ntx = ptask->tx_mbuf[tx_portid].n_mbufs;
	ptask->tx_mbuf[tx_portid].n_mbufs = (ntx + n_pkts) & (MAX_PKT_BURST * 2 - 1);


	if (ntx + n_pkts < MAX_PKT_BURST * 2) {
		rte_memcpy(ptask->tx_mbuf[tx_portid].mbuf + ntx, rx_mbuf, n_pkts * sizeof(struct rte_mbuf *));
	}
	else {
		TGEN_ASSERT(ntx < MAX_PKT_BURST*2);
		const uint16_t offset = MAX_PKT_BURST * 2 - ntx;
		rte_memcpy(ptask->tx_mbuf[tx_portid].mbuf + ntx, rx_mbuf, offset * sizeof(struct rte_mbuf *));
		rte_memcpy(ptask->tx_mbuf[tx_portid].mbuf, rx_mbuf + offset, (n_pkts - offset)*sizeof(struct rte_mbuf *));
	}
}


void tx_buf_pkt_single(struct task_base *ptask, struct rte_mbuf *rx_mbuf, const uint8_t tx_portid)
{
	const uint16_t ntx = ptask->tx_mbuf[tx_portid].n_mbufs;
	ptask->tx_mbuf[tx_portid].n_mbufs = (ntx + 1) & (MAX_PKT_BURST * 2 - 1);
	ptask->tx_mbuf[tx_portid].mbuf[ntx] = rx_mbuf;
}


/* the following help functions also report stats (thus need to pass struct port) */
static inline void tx_bulk_drop(const struct tx_port_queue *port_queue, struct rte_mbuf **tx_pkts, __attribute__((unused)) struct task_base *task_base);
static inline void tx_bulk_no_drop(const struct tx_port_queue *port_queue, struct rte_mbuf **tx_pkts, __attribute__((unused)) struct task_base *task_base);
static inline void ring_enq_bulk_drop(struct rte_ring *tx_ring, struct rte_mbuf *const *tx_pkts, __attribute__((unused)) struct task_base *task_base);
static inline void ring_enq_bulk_no_drop(struct rte_ring *tx_ring, struct rte_mbuf *const *tx_pkts, __attribute__((unused)) struct task_base *task_base);

static inline void tx_burst_drop(const struct tx_port_queue *port_queue, struct rte_mbuf **tx_pkts, uint16_t n_pkts, __attribute__((unused)) struct task_base *task_base);
static inline void tx_burst_no_drop(const struct tx_port_queue *port_queue, struct rte_mbuf **tx_pkts, uint16_t n_pkts, __attribute__((unused)) struct task_base *task_base);
static inline void ring_enq_burst_drop(struct rte_ring *tx_ring, struct rte_mbuf *const *tx_pkts, uint16_t n_pkts, __attribute__((unused)) struct task_base *task_base);
static inline void ring_enq_burst_no_drop(struct rte_ring *tx_ring, struct rte_mbuf *const *tx_pkts, uint16_t n_pkts, __attribute__((unused)) struct task_base *task_base);

static inline void tx_bulk_drop(const struct tx_port_queue *port_queue, struct rte_mbuf **tx_pkts, __attribute__((unused)) struct task_base *task_base)
{
	uint16_t ntx = rte_eth_tx_burst(port_queue->port, port_queue->queue, tx_pkts, MAX_PKT_BURST);

	INCR_TX_PKT_COUNT(task_base->stats, ntx);
	if (ntx < MAX_PKT_BURST) {
		INCR_TX_DROP_COUNT(task_base->stats, MAX_PKT_BURST - ntx);

		do {
			rte_pktmbuf_free(tx_pkts[ntx++]);
		}
		while (ntx < MAX_PKT_BURST);
	}
}

static inline void tx_bulk_no_drop(const struct tx_port_queue *port_queue, struct rte_mbuf **tx_pkts, __attribute__((unused)) struct task_base *task_base)
{
	uint16_t ntx = MAX_PKT_BURST;
	uint16_t ret;
	INCR_TX_PKT_COUNT(task_base->stats, MAX_PKT_BURST);

	do {
		ret = rte_eth_tx_burst(port_queue->port, port_queue->queue, tx_pkts, ntx);
		tx_pkts += ret;
		ntx -= ret;
	}
	while (ntx);
}

static inline void ring_enq_bulk_drop(struct rte_ring *tx_ring, struct rte_mbuf *const *tx_pkts, __attribute__((unused)) struct task_base *task_base)
{
	/* return 0 on succes, -ENOBUFS on failure */
	if (unlikely(rte_ring_sp_enqueue_bulk(tx_ring, (void *const *)tx_pkts, MAX_RING_BURST))) {
		for (uint16_t k = 0; k < MAX_RING_BURST; ++k) {
			rte_pktmbuf_free(tx_pkts[k]);
		}
		INCR_TX_DROP_COUNT(task_base->stats, MAX_PKT_BURST);
	}
	else {
		INCR_TX_PKT_COUNT(task_base->stats, MAX_PKT_BURST);
	}
}

static inline void ring_enq_bulk_no_drop(struct rte_ring *tx_ring, struct rte_mbuf *const *tx_pkts, __attribute__((unused)) struct task_base *task_base)
{
	while (rte_ring_sp_enqueue_bulk(tx_ring, (void *const *)tx_pkts, MAX_RING_BURST));
	INCR_TX_PKT_COUNT(task_base->stats, MAX_RING_BURST);
}


static inline void tx_burst_drop(const struct tx_port_queue *port_queue, struct rte_mbuf **tx_pkts, uint16_t n_pkts, __attribute__((unused)) struct task_base *task_base)
{
	uint16_t ntx = rte_eth_tx_burst(port_queue->port, port_queue->queue, tx_pkts, n_pkts);

	if (ntx < n_pkts) {
		INCR_TX_DROP_COUNT(task_base->stats, n_pkts - ntx);
		do {
			rte_pktmbuf_free(tx_pkts[ntx++]);
		}
		while (ntx < n_pkts);
	}

	INCR_TX_PKT_COUNT(task_base->stats, ntx);
}

static inline void tx_burst_no_drop(const struct tx_port_queue *port_queue, struct rte_mbuf **tx_pkts, uint16_t n_pkts, __attribute__((unused)) struct task_base *task_base)
{
	uint16_t ret;

	INCR_TX_PKT_COUNT(task_base->stats, n_pkts);

	do {
		ret = rte_eth_tx_burst(port_queue->port, port_queue->queue, tx_pkts, n_pkts);
		tx_pkts += ret;
		n_pkts -= ret;
	}
	while (n_pkts);
}

static inline void ring_enq_burst_drop(struct rte_ring *tx_ring, struct rte_mbuf *const *tx_pkts, uint16_t n_pkts, __attribute__((unused)) struct task_base *task_base)
{
	/* return 0 on succes, -ENOBUFS on failure */
	if (unlikely(rte_ring_sp_enqueue_bulk(tx_ring, (void *const *)tx_pkts, n_pkts))) {
		for (uint16_t k = 0; k < n_pkts; ++k) {
			rte_pktmbuf_free(tx_pkts[k]);
		}
		INCR_TX_DROP_COUNT(task_base->stats, n_pkts);
	}
	else {
		INCR_TX_PKT_COUNT(task_base->stats, n_pkts);
	}
}

static inline void ring_enq_burst_no_drop(struct rte_ring *tx_ring, struct rte_mbuf *const *tx_pkts, uint16_t n_pkts, __attribute__((unused)) struct task_base *task_base)
{
	while (rte_ring_sp_enqueue_bulk(tx_ring, (void *const *)tx_pkts, n_pkts));
	INCR_TX_PKT_COUNT(task_base->stats, n_pkts);
}

void flush_queues_sw(struct task_base *ptask)
{
	uint16_t ntx, last_sent;
	for (uint8_t z = 0; z < ptask->tx_params_sw.nb_txrings; ++z) {
		ntx = ptask->tx_mbuf[z].n_mbufs;
		last_sent = ptask->tx_mbuf[z].last_sent;
		if (!last_sent && ntx) {
			TGEN_ASSERT(ntx <= MAX_RING_BURST);
			ring_enq_burst_drop(ptask->tx_params_sw.tx_rings[z], ptask->tx_mbuf[z].mbuf, ntx, ptask);
			ptask->tx_mbuf[z].n_mbufs = 0;
			ptask->tx_mbuf[z].last_sent = 0;

		}
		else if (last_sent && ntx > MAX_PKT_BURST) {
			TGEN_ASSERT(ntx - MAX_PKT_BURST <= MAX_RING_BURST);
			ring_enq_burst_drop(ptask->tx_params_sw.tx_rings[z], ptask->tx_mbuf[z].mbuf + MAX_PKT_BURST, ntx - MAX_PKT_BURST, ptask);
			ptask->tx_mbuf[z].n_mbufs = 0;
			ptask->tx_mbuf[z].last_sent = 0;
		}
	}

	ptask->flags &= ~FLAG_TX_FLUSH;
}

void flush_queues_hw(struct task_base *ptask)
{
	uint16_t ntx, last_sent;
	for (uint8_t z = 0; z < ptask->tx_params_hw.nb_txports; ++z) {
		ntx = ptask->tx_mbuf[z].n_mbufs;
		last_sent = ptask->tx_mbuf[z].last_sent;
		if (!last_sent && ntx) {
			tx_burst_drop(&ptask->tx_params_hw.tx_port_queue[z], ptask->tx_mbuf[z].mbuf, ntx, ptask);
			ptask->tx_mbuf[z].n_mbufs = 0;
			ptask->tx_mbuf[z].last_sent = 0;

		}
		else if (last_sent && ntx > MAX_PKT_BURST) {
			tx_burst_drop(&ptask->tx_params_hw.tx_port_queue[z], ptask->tx_mbuf[z].mbuf + MAX_PKT_BURST, ntx - MAX_PKT_BURST, ptask);
			ptask->tx_mbuf[z].n_mbufs = 0;
			ptask->tx_mbuf[z].last_sent = 0;
		}
	}

	ptask->flags &= ~FLAG_TX_FLUSH;
}

void flush_queues_no_drop_hw(struct task_base *ptask)
{
	uint16_t ntx, last_sent;
	for (uint8_t z = 0; z < ptask->tx_params_hw.nb_txports; ++z) {
		ntx = ptask->tx_mbuf[z].n_mbufs;
		last_sent = ptask->tx_mbuf[z].last_sent;
		if (!last_sent && ntx) {
			tx_burst_drop(&ptask->tx_params_hw.tx_port_queue[z], ptask->tx_mbuf[z].mbuf, ntx, ptask);
			ptask->tx_mbuf[z].n_mbufs = 0;
			ptask->tx_mbuf[z].last_sent = 0;
		}
		else if (last_sent && ntx > MAX_PKT_BURST) {
			tx_burst_drop(&ptask->tx_params_hw.tx_port_queue[z], ptask->tx_mbuf[z].mbuf + MAX_PKT_BURST, ntx - MAX_PKT_BURST, ptask);
			ptask->tx_mbuf[z].n_mbufs = 0;
			ptask->tx_mbuf[z].last_sent = 0;
		}
	}

	ptask->flags &= ~FLAG_TX_FLUSH;
}

void flush_queues_no_drop_sw(struct task_base *ptask)
{
	uint16_t ntx, last_sent;
	for (uint8_t z = 0; z < ptask->tx_params_sw.nb_txrings; ++z) {
		ntx = ptask->tx_mbuf[z].n_mbufs;
		last_sent = ptask->tx_mbuf[z].last_sent;
		if (!last_sent && ntx) {
			ring_enq_burst_no_drop(ptask->tx_params_sw.tx_rings[z], ptask->tx_mbuf[z].mbuf, ntx, ptask);
			ptask->tx_mbuf[z].n_mbufs = 0;
			ptask->tx_mbuf[z].last_sent = 0;
		}
		else if (last_sent && ntx > MAX_PKT_BURST) {
			ring_enq_burst_no_drop(ptask->tx_params_sw.tx_rings[z], ptask->tx_mbuf[z].mbuf + MAX_PKT_BURST, ntx - MAX_PKT_BURST, ptask);
			ptask->tx_mbuf[z].n_mbufs = 0;
			ptask->tx_mbuf[z].last_sent = 0;
		}
	}
	ptask->flags &= ~FLAG_TX_FLUSH;
}

void tx_pkt_no_drop_no_buf_hw(struct rte_mbuf **rx_mbuf, struct task_base *ptask, const uint16_t n_pkts)
{
	/* tx_portid will always be 0, otherwise the no_buf would not make sense. */
	tx_burst_no_drop(&ptask->tx_params_hw.tx_port_queue[0], rx_mbuf, n_pkts, ptask);
}

void tx_pkt_no_drop_no_buf_sw(struct rte_mbuf **rx_mbuf, struct task_base *ptask, const uint16_t n_pkts)
{
	ring_enq_burst_no_drop(ptask->tx_params_sw.tx_rings[0], rx_mbuf, n_pkts, ptask);
}

void tx_pkt_no_buf_hw(struct rte_mbuf **rx_mbuf, struct task_base *ptask, const uint16_t n_pkts)
{
	/* tx_portid will always be 0, otherwise the no_buf would not make sense. */
	tx_burst_drop(&ptask->tx_params_hw.tx_port_queue[0], rx_mbuf, n_pkts, ptask);
}

void tx_pkt_no_buf_sw(struct rte_mbuf **rx_mbuf, struct task_base *ptask, const uint16_t n_pkts)
{
	ring_enq_burst_drop(ptask->tx_params_sw.tx_rings[0], rx_mbuf, n_pkts, ptask);
}

void tx_pkt_no_drop_hw(struct task_base *ptask)
{
	const uint8_t buf_count = ptask->tx_params_hw.nb_txports;
	for (uint8_t tx_port = 0; tx_port < buf_count; ++tx_port) {
		const uint16_t ntx = ptask->tx_mbuf[tx_port].n_mbufs;
		const uint16_t last_sent = ptask->tx_mbuf[tx_port].last_sent;
		if (last_sent && ntx < MAX_PKT_BURST) {
			tx_bulk_no_drop(&ptask->tx_params_hw.tx_port_queue[tx_port], ptask->tx_mbuf[tx_port].mbuf + MAX_PKT_BURST, ptask);
			ptask->tx_mbuf[tx_port].last_sent = 0;
			ptask->flags &= ~FLAG_TX_FLUSH;
		}
		else if (!last_sent && ntx >= MAX_PKT_BURST) {
			tx_bulk_no_drop(&ptask->tx_params_hw.tx_port_queue[tx_port], ptask->tx_mbuf[tx_port].mbuf, ptask);
			ptask->tx_mbuf[tx_port].last_sent = 1;
			ptask->flags &= ~FLAG_TX_FLUSH;
		}
	}
}

void tx_pkt_no_drop_sw(struct task_base *ptask)
{
	const uint8_t buf_count = ptask->tx_params_sw.nb_txrings;
	for (uint8_t tx_port = 0; tx_port < buf_count; ++tx_port) {
		const uint16_t ntx = ptask->tx_mbuf[tx_port].n_mbufs;
		const uint16_t last_sent = ptask->tx_mbuf[tx_port].last_sent;
		if (last_sent && ntx < MAX_PKT_BURST) {
			ring_enq_bulk_no_drop(ptask->tx_params_sw.tx_rings[tx_port], ptask->tx_mbuf[tx_port].mbuf + MAX_PKT_BURST, ptask);
			ptask->tx_mbuf[tx_port].last_sent = 0;
			ptask->flags &= ~FLAG_TX_FLUSH;
		}
		else if (!last_sent && ntx >= MAX_PKT_BURST) {
			ring_enq_bulk_no_drop(ptask->tx_params_sw.tx_rings[tx_port], ptask->tx_mbuf[tx_port].mbuf, ptask);
			ptask->tx_mbuf[tx_port].last_sent = 1;
			ptask->flags &= ~FLAG_TX_FLUSH;
		}
	}
}

void tx_pkt_hw(struct task_base *ptask)
{
	const uint8_t buf_count = ptask->tx_params_hw.nb_txports;
	for (uint8_t tx_port = 0; tx_port < buf_count; ++tx_port) {
		const uint16_t ntx = ptask->tx_mbuf[tx_port].n_mbufs;
		const uint16_t last_sent = ptask->tx_mbuf[tx_port].last_sent;
		if (last_sent && ntx < MAX_PKT_BURST) {
			tx_bulk_drop(&ptask->tx_params_hw.tx_port_queue[tx_port], ptask->tx_mbuf[tx_port].mbuf + MAX_PKT_BURST, ptask);
			ptask->tx_mbuf[tx_port].last_sent = 0;
			ptask->flags &= ~FLAG_TX_FLUSH;
		}
		else if (!last_sent && ntx >= MAX_PKT_BURST) {
			tx_bulk_drop(&ptask->tx_params_hw.tx_port_queue[tx_port], ptask->tx_mbuf[tx_port].mbuf, ptask);
			ptask->tx_mbuf[tx_port].last_sent = 1;
			ptask->flags &= ~FLAG_TX_FLUSH;
		}
	}
}

void tx_pkt_sw(struct task_base *ptask)
{
	const uint8_t buf_count = ptask->tx_params_sw.nb_txrings;
	for (uint8_t tx_port = 0; tx_port < buf_count; ++tx_port) {
		const uint16_t ntx = ptask->tx_mbuf[tx_port].n_mbufs;
		const uint16_t last_sent = ptask->tx_mbuf[tx_port].last_sent;

		if (last_sent && ntx < MAX_PKT_BURST) {
			ring_enq_bulk_drop(ptask->tx_params_sw.tx_rings[tx_port], ptask->tx_mbuf[tx_port].mbuf + MAX_PKT_BURST, ptask);
			ptask->tx_mbuf[tx_port].last_sent = 0;
			ptask->flags &= ~FLAG_TX_FLUSH;
		}
		else if (!last_sent && ntx >= MAX_PKT_BURST) {
			ring_enq_bulk_drop(ptask->tx_params_sw.tx_rings[tx_port], ptask->tx_mbuf[tx_port].mbuf, ptask);
			ptask->tx_mbuf[tx_port].last_sent = 1;
			ptask->flags &= ~FLAG_TX_FLUSH;
		}
	}
}
