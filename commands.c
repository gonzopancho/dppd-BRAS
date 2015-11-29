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

#include <rte_hash.h>

#include "commands.h"
#include "stats.h"
#include "display.h"
#include "main.h"
#include "tx_worker.h"
#include "tgen_args.h"
#include "handle_qinq_decaparp.h"
#include "handle_qinq_decapv4.h"

static void print_hash_table(const struct rte_hash *h);
static void print_hash_table_size(const struct rte_hash *h);

void start_core(uint64_t core_mask)
{
	mprintf("Starting worker cores:");
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if ((core_mask & (__UINT64_C(1) << lcore_id)) && (rte_eal_get_lcore_state(lcore_id) != RUNNING)) {
			mprintf(" %u", lcore_id);
		}
	}
	mprintf("\n");
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if ((core_mask & (__UINT64_C(1) << lcore_id)) && (rte_eal_get_lcore_state(lcore_id) != RUNNING)) {
			rte_eal_remote_launch(tgen_work_thread, &rte_cfg.nb_ports, lcore_id);
		}
	}
}

void stop_core(uint64_t core_mask)
{
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (core_mask & (__UINT64_C(1) << lcore_id)) {
			lcore_cfg[lcore_id].flags |= PCFG_TERMINATE;
		}
	}

	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (core_mask & (__UINT64_C(1) << lcore_id)) {
			if ((rte_eal_get_lcore_state(lcore_id) == RUNNING) || (rte_eal_get_lcore_state(lcore_id) == FINISHED)) {
				mprintf("stopping core %u...", lcore_id);
				rte_eal_wait_lcore(lcore_id);
				mprintf(" OK\n");
			}
			else {
				mprintf("core %u in state %d\n", lcore_id, rte_eal_get_lcore_state(lcore_id));
			}
			lcore_cfg[lcore_id].flags &= ~PCFG_TERMINATE;
		}
	}
}

void pool_count(void)
{
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		struct lcore_cfg *lconf = &lcore_cfg[lcore_id];

		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[task_id];
			if (startup_cfg->rx_port != NO_PORT_AVAIL && startup_cfg->pool) {
				const uint32_t count = rte_mempool_count(startup_cfg->pool);
				const uint32_t free_count = rte_mempool_free_count(startup_cfg->pool);
				mprintf("\t\tCore %u task %u count=%u, free count=%u\n", lcore_id, task_id, count, free_count);
			}
		}
	}
}

#ifdef BRAS_CMD_STAT_RX
static void stat_rx_get(uint64_t core_mask, uint32_t mode)
{
	mprintf("core_mask = %lx\n", core_mask);
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (core_mask & (__UINT64_C(1) << lcore_id)) {
			struct lcore_cfg *lconf = &lcore_cfg[lcore_id];
			mprintf("core %u(%32s):", lcore_id, lconf->name);
			for (uint16_t k = 0; k < MAX_RING_BURST + 1; ++k) {
				mprintf(" %lu", rte_atomic64_read(&lconf->nb_rx_buckets[k]));
			}
			mprintf("\n");
			if (mode) {
				for (uint16_t k = 0; k < MAX_RING_BURST + 1; ++k) {
					rte_atomic64_set(&lconf->nb_rx_buckets[k], 0);
				}
			}
		}
	}
}
#endif

static void stat_port_get(uint32_t port_mask)
{
	struct rte_eth_stats stats;
	for (uint8_t port = 0; port < TGEN_MAX_PORTS; ++port) {
		if (port_mask & (1U << port)) {
			if ((tgen_cfg.flags & TGSF_USE_VF) == 0) {
				for (uint8_t i = 0; i < 16; ++i) {
					int ret;
					ret = rte_eth_dev_set_rx_queue_stats_mapping(port, i, i);
					if (ret) {
						mprintf("rte_eth_dev_set_rx_queue_stats_mapping() failed: error %d\n", ret);
					}
					ret = rte_eth_dev_set_tx_queue_stats_mapping(port, i, i);
					if (ret) {
						mprintf("rte_eth_dev_set_tx_queue_stats_mapping() failed: error %d\n", ret);
					}
				}
			}
			rte_eth_stats_get(port, &stats);
			mprintf("port %u: i=%lu, o=%lu L2i=%lu L2o=%lu no_buf=%lu\n",
			        port, stats.ipackets, stats.opackets, stats.ilbpackets, stats.olbpackets, stats.rx_nombuf);
			if ((tgen_cfg.flags & TGSF_USE_VF) == 0) {
				for (uint8_t i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS; ++i) {
					mprintf("q %u: i=%lu, o=%lu e=%lu\n", i, stats.q_ipackets[i], stats.q_opackets[i], stats.q_errors[i]);
				}
			}
		}
	}
}

void cmd_tcpdump(uint8_t lcore_id, uint8_t task_id, uint32_t nb_packets)
{
	mprintf("tcpdump %u %u %u\n", lcore_id, task_id, nb_packets);
#ifdef BRAS_CMD_TCPDUMP
	if (lcore_id > RTE_MAX_LCORE) {
		mprintf("core_id to high, maximum allowed is: %u\n", RTE_MAX_LCORE);
	}
	else if (task_id >= lcore_cfg[lcore_id].nb_tasks) {
		mprintf("task_id to high, should be in [0, %u]\n", lcore_cfg[lcore_id].nb_tasks - 1);
	}
	else {
		rte_atomic32_set(&lcore_cfg[lcore_id].task[task_id]->task_debug.nb_print, nb_packets);
	}
#else
	mprintf("tcpdump disabled at compile time\n");
#endif
}

void cmd_hashdump(uint8_t lcore_id, uint8_t task_id, uint32_t table_id)
{
	mprintf("hashdump %u %u\n", lcore_id, table_id);
	switch (table_id) {
	case 20:
	case 2: {
		enum port_mode mode = lcore_cfg[lcore_id].startup_cfg[task_id].mode;

		struct rte_hash *qinq_gre = NULL;
		if (mode == QINQ_DECAP_ARP) {
			qinq_gre = ((struct task_qinq_decap_arp *)lcore_cfg[lcore_id].task[task_id])->qinq_gre;
		}
		else if (mode == QINQ_DECAP_V4) {
			qinq_gre = ((struct task_qinq_decap_v4 *)lcore_cfg[lcore_id].task[task_id])->qinq_gre;
		}
		else {
			mprintf("Error: no hash in this task\n");
		}

		if (table_id == 2) {
			print_hash_table(qinq_gre);
		}
		else {
			print_hash_table_size(qinq_gre);
		}
	}
	break;
	case 6:
		print_hash_table(lcore_cfg[lcore_id].cpe_v6_table);
		break;
	case 21:
		print_hash_table_size(lcore_cfg[lcore_id].cpe_v4_table);
		break;
	default:
		print_hash_table(lcore_cfg[lcore_id].cpe_v4_table);
	}
}

void cmd_stat(const char *mode, uint32_t id)
{
	if (strcmp(mode, "port") == 0) {
		unsigned mask = 1U << id;
		if ((id >= TGEN_MAX_PORTS) || !(tgen_used_port_mask & mask)) {
			mprintf("Invalid port id %u\n", id);
		}
		else {
			stat_port_get(mask);
		}
	}
#ifdef BRAS_CMD_STAT_RX
	else if (strcmp(mode, "rx") == 0) {
		stat_rx_get(tgen_used_core_mask, id);
	}
#endif
	else if (strcmp(mode, "all_ports") == 0) {
		stat_port_get(tgen_used_port_mask);
	}
	else if (strcmp(mode, "size") == 0) {
		mprintf("size = %lu\n", sizeof(struct lcore_cfg));
	}
}

static void print_hash_table_size(const struct rte_hash *h)
{
	uint32_t count = 0, total_count = 0;
	if (h == NULL) {
		return;
	}
	for (uint32_t bucket_index = 0; bucket_index < h->num_buckets; ++bucket_index) {
		uint32_t *sig_bucket = (hash_sig_t *) & (h->sig_tbl[bucket_index * h->sig_tbl_bucket_size]);
		for (uint32_t i = 0; i < h->bucket_entries; ++i) {
			if (sig_bucket[i]) {
				++count;
			}
		}
		if (count) {
			mprintf("bucket %u count %u\n", bucket_index, count);
			total_count += count;
			count = 0;
		}
	}
	mprintf("sig_tbl_size = %u, key_tbl_size=%u, hash_tbl_size=%lu\n", h->sig_tbl_bucket_size * h->num_buckets, h->key_tbl_key_size * h->num_buckets * h->bucket_entries, sizeof(struct rte_hash));
	mprintf("sig bkt size = %u bkt entries = %u num_buckets = %u key size = %u - %u elements in hash table\n", h->sig_tbl_bucket_size, h->bucket_entries, h->num_buckets, h->key_tbl_key_size, total_count);
}

static void print_hash_table(const struct rte_hash *h)
{
	uint32_t count = 0, total_count = 0;
	if (h == NULL) {
		return;
	}
	for (uint32_t bucket_index = 0; bucket_index < h->num_buckets; ++bucket_index) {
		uint32_t *sig_bucket = (hash_sig_t *) & (h->sig_tbl[bucket_index * h->sig_tbl_bucket_size]);
		uint8_t *key_bucket = &(h->key_tbl[bucket_index * h->bucket_entries * h->key_tbl_key_size]);
		for (uint32_t i = 0; i < h->bucket_entries; ++i) {
			if (sig_bucket[i]) {
				++count;
				mprintf("sig_bucket[%u][%u] = %x, key_bucket = %014lx\n", bucket_index, i, sig_bucket[i], *(uint64_t *)&key_bucket[i * h->key_tbl_key_size]);
			}
		}
		if (count) {
			total_count += count;
			count = 0;
		}
	}
	mprintf("sig bkt size = %u bkt entries = %u key size = %u - %u elements in hash table\n", h->sig_tbl_bucket_size, h->bucket_entries, h->key_tbl_key_size, total_count);
}
