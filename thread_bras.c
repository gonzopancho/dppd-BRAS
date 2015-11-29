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
#include <rte_hash.h>

#include "thread_bras.h"
#include "display.h"
#include "stats.h"
#include "tx_pkt.h"
#include "config.h"
#include "control.h"
#include "hash_entry_types.h"
#include "defines.h"

static void update_arp_entriesv4(struct lcore_cfg *lconf, uint64_t cur_tsc);
static void update_arp_entriesv6(struct lcore_cfg *lconf, uint64_t cur_tsc);
static void update_arp_entries(struct lcore_cfg *lconf, uint64_t cur_tsc);


static void update_arp_entries(struct lcore_cfg *lconf, uint64_t cur_tsc)
{
	if (lconf->cpe_v4_table) {
		update_arp_entriesv4(lconf, cur_tsc);
	}

	if (lconf->cpe_v6_table) {
		update_arp_entriesv6(lconf, cur_tsc);
	}
}

static void update_arp_entriesv6(struct lcore_cfg *lconf, uint64_t cur_tsc)
{
	// Supposing here same size for ipv4 and ipv6 arp tables
	uint8_t *entry_bucket6 = (uint8_t *)&lconf->cpe_v6_table->key_tbl[lconf->bucket_index6 * lconf->cpe_v6_table->bucket_entries * lconf->cpe_v6_table->key_tbl_key_size];
	uint32_t *sig_bucket6 = (hash_sig_t *) & (lconf->cpe_v6_table->sig_tbl[lconf->bucket_index6 * lconf->cpe_v6_table->sig_tbl_bucket_size]);
	uint32_t table_index = lconf->bucket_index6 * lconf->cpe_v6_table->bucket_entries;

	for (uint32_t pos = 0; pos < lconf->cpe_v6_table->bucket_entries; ++pos, ++table_index) {
		struct cpe_table_hash_entry *entry6 = (struct cpe_table_hash_entry *)&entry_bucket6[pos * lconf->cpe_v6_table->key_tbl_key_size];
		if (entry6->data.tsc < cur_tsc) {
			sig_bucket6[pos] = NULL_SIGNATURE;
			entry6->data.tsc = MAX_TSC;
		}
	}
	++lconf->bucket_index6;
	lconf->bucket_index6 &= lconf->cpe_v6_table->bucket_bitmask;
}

static void update_arp_entriesv4(struct lcore_cfg *lconf, uint64_t cur_tsc)
{
	uint32_t *sig_bucket = (hash_sig_t *) & (lconf->cpe_v4_table->sig_tbl[lconf->bucket_index * lconf->cpe_v4_table->sig_tbl_bucket_size]);
	uint32_t table_index = lconf->bucket_index * lconf->cpe_v4_table->bucket_entries;

	uint8_t *entry_bucket = (uint8_t *)&lconf->cpe_v4_table->key_tbl[lconf->bucket_index * lconf->cpe_v4_table->bucket_entries * lconf->cpe_v4_table->key_tbl_key_size];

	for (uint32_t pos = 0; pos < lconf->cpe_v4_table->bucket_entries; ++pos, ++table_index) {
		struct cpe_table_hash_entry *entry = (struct cpe_table_hash_entry *)&entry_bucket[pos * lconf->cpe_v4_table->key_tbl_key_size];
		if (entry->data.tsc < cur_tsc) {
			sig_bucket[pos] = NULL_SIGNATURE;
			entry->data.tsc = MAX_TSC;
		}
	}
	++lconf->bucket_index;
	lconf->bucket_index &= lconf->cpe_v4_table->bucket_bitmask;
}

int thread_bras(struct lcore_cfg *lconf)
{
	struct rte_mbuf *rx_mbuf[MAX_RING_BURST] __rte_cache_aligned;
	struct task_base *task[MAX_TASKS_PER_CORE];
	uint64_t cur_tsc = rte_rdtsc();
	uint64_t next_term_tsc = cur_tsc + TERM_TIMEOUT;
	uint64_t drain_tsc = cur_tsc + DRAIN_TIMEOUT;
	uint64_t arp_tsc;
	const uint8_t nb_tasks = lconf->nb_tasks;

	for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
		task[task_id] = lconf->task[task_id];
	}

	/* Make sure arp timeout is checked every second - one element every arp_loop_timer */
	const uint64_t arp_loop_timer = (rte_get_tsc_hz() >> 1) / NUM_VCPES;

	if (lconf->cpe_v6_table || lconf->cpe_v4_table) {
		arp_tsc = cur_tsc + arp_loop_timer;
	}
	else {
		arp_tsc = MAX_TSC;
	}

	for (;;) {
		cur_tsc = rte_rdtsc();

		/* drain TX buffer every DRAIN_TIMEOUT */
		if (unlikely(cur_tsc > drain_tsc)) {
			drain_tsc = cur_tsc + DRAIN_TIMEOUT;

			FLUSH_STATS(lconf);
			/* check for termination request every timeout */
			if (cur_tsc > next_term_tsc) {
				next_term_tsc = cur_tsc + TERM_TIMEOUT;

				if (is_terminated(lconf)) {
					break;
				}
			}
			if (cur_tsc > arp_tsc) {
				update_arp_entries(lconf, cur_tsc);
				// do not use cur_tsc as we might be late...
				arp_tsc += arp_loop_timer;
			}

			for (uint8_t task_id = 0; task_id < nb_tasks; ++task_id) {
				if (!(task[task_id]->flags & FLAG_TX_FLUSH) && task[task_id]->flush_queues) {
					// Do not flush packets if we transmitted packets in last drain_timeout
					// This avoid flushing queue under load every x seconds
					task[task_id]->flags |= FLAG_TX_FLUSH;
					continue;
				}
				/* This part of the code is only run on low load - when we need to flush,
				   i.e. when we did not send a bulk packets within last drain_timeout
				   (16kpps if DRAIN_TIMEOUT=2msec). All queues are flushed in this case */
				task[task_id]->flush_queues(task[task_id]);
			}
		}

		for (uint8_t task_id = 0; task_id < nb_tasks; ++task_id) {
			uint16_t nb_rx = task[task_id]->rx_pkt(rx_mbuf, task[task_id]);

			if (likely(nb_rx)) {
				INCR_NBRX(nb_rx);
				INCR_RX_PKT_COUNT(task[task_id]->stats, nb_rx);
				task[task_id]->handle_pkt_bulk(rx_mbuf, task[task_id], nb_rx);
			}
		}
	}
	return 0;
}
