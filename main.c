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

#include "main.h"

#include <string.h>
#include <locale.h>

#include <rte_malloc.h>
#include <rte_cycles.h>
#include <rte_string_fns.h>
#include <rte_ethdev.h>
#include <rte_hash.h>

#include "tgen_args.h"
#include "display.h"
#include "route.h"
#include "clock.h"
#include "quit.h"
#include "read_config.h"
#include "defines.h"
#include "tgen_assert.h"

enum port_type {PORT_IGB = 1, PORT_IXGBE = 10};
uint32_t tgen_used_port_mask = 0;

/* port configuration */
struct tgen_port_conf {
	struct rte_mempool *pool[32];  /* Rx/Tx mempool */
	uint8_t socket;
	enum port_type type;           /* 10Gb or 1Gb */
	uint16_t max_tx_queue;         /* max number of Tx queues */
	uint16_t n_txqueues;           /* number of used Tx queues */
	uint16_t n_rxqueues;           /* number of used Rx queues */
	uint16_t nb_txd;
	uint16_t nb_rxd;
	struct ether_addr eth_addr;    /* port MAC address */
} __rte_cache_aligned;

static struct tgen_port_conf tgen_port_conf[TGEN_MAX_PORTS];
uint8_t port_status[TGEN_MAX_PORTS] = {0};

uint8_t lb_nb_txrings = 0xff;


/* standard mbuf initialization procedure */
static void tgen_pktmbuf_init(struct rte_mempool *mp, void *opaque_arg, void *_m, unsigned i)
{
	struct rte_mbuf *mbuf = _m;
	mbuf->pkt.vlan_macip.f.l2_len = sizeof(struct ether_hdr);
	mbuf->pkt.vlan_macip.f.l3_len = sizeof(struct ipv4_hdr);
	rte_pktmbuf_init(mp, opaque_arg, mbuf, i);
}

static void __attribute__((noreturn)) tgen_usage(const char *prgname)
{
	mprintf("\nUsage: %s [-f CONFIG_FILE] [-a|-e] [-s|-i]\n"
	        "\t-f CONFIG_FILE : configuration file to load, ./tgen.cfg by default\n"
	        "\t-a : autostart all ports (by default)\n"
	        "\t-e : don't autostart\n"
	        "\t-s : check configuration file syntax and exit\n"
	        "\t-i : check initialization sequence and exit\n"
	        , prgname);
	exit(EXIT_FAILURE);
}

/* initialize rte devices and check the number of available ports */
static uint8_t init_rte_dev(void)
{
	uint8_t nb_ports;
	struct rte_eth_dev_info dev_info;


	/* initialize driver(s) */
	TGEN_PANIC(rte_ixgbe_pmd_init() < 0, "\tError: Cannot init ixgbe pmd\n");

	if (tgen_cfg.flags & TGSF_USE_VF) {
		TGEN_PANIC(rte_ixgbevf_pmd_init() < 0, "\tError: cannot init ixgbevf pmd\n");
	}

	TGEN_PANIC(rte_eal_pci_probe() < 0, "\tError: Cannot probe PCI\n");

	/* get available ports configuration */
	nb_ports = rte_eth_dev_count();
	TGEN_PANIC(nb_ports == 0, "\tError: DPDK could not find any port\n");
	mprintf("\tDPDK has found %u ports\n", nb_ports);

	if (nb_ports > TGEN_MAX_PORTS) {
		mprintf("\tWarning: I can deal with at most %u ports."
		        " Please update TGEN_MAX_PORTS and recompile.\n", TGEN_MAX_PORTS);

		nb_ports = TGEN_MAX_PORTS;
	}

	TGEN_PANIC(tgen_used_port_mask & ~((1U << nb_ports) - 1),
	           "\tError: invalid port(s) specified, used port mask is %#10x\n", tgen_used_port_mask);

	/* read max TX queues per port */
	for (uint8_t port_id = 0; port_id < nb_ports; ++port_id) {
		/* skip ports that are not enabled */
		if ((tgen_used_port_mask & (1U << port_id)) == 0) {
			continue;
		}
		rte_eth_dev_info_get(port_id, &dev_info);
		tgen_port_conf[port_id].max_tx_queue = dev_info.max_tx_queues;
		mprintf("\tPort %u, Max TX queue = %u\n", port_id, dev_info.max_tx_queues);
		if (strcmp(dev_info.driver_name, "rte_ixgbe_pmd") == 0) {
			tgen_port_conf[port_id].type = PORT_IXGBE;
		}
		else {
			tgen_port_conf[port_id].type = PORT_IGB;
		}
	}

	return nb_ports;
}

static void check_consistent_cfg(void)
{
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master) {
			continue;
		}
		const struct lcore_cfg *lconf = &lcore_cfg[lcore_id];
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			const struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[task_id];
			TGEN_PANIC((startup_cfg->flags & PORT_STARTUP_RX_RING) && (startup_cfg->rx_rings[0] == 0),
			           "Configuration Error - Core %u task %u Receiving from ring, but nobody xmitting to this ring\n", lcore_id, task_id);

			for (uint8_t ring_idx = 0; ring_idx < startup_cfg->nb_rxrings; ++ring_idx) {
				mprintf("\t\tCore %u, task %u, rx_ring[%u] %p\n", lcore_id, task_id, ring_idx, startup_cfg->rx_rings[ring_idx]);
			}
			if (startup_cfg->nb_txports == 0 && startup_cfg->nb_txrings == 0) {
				TGEN_PANIC(!(startup_cfg->mode & QINQ_DECAP_ARP) && !(lconf->flags & PCFG_DROP),
				           "\t\tCore %u task %u does not transmit or drop packet: no tx_ports and no tx_rings\n", lcore_id, task_id);
			}
		}
	}
}

static void setup_all_task_structs(void)
{
	struct lcore_cfg *lconf;

	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master) {
			continue;
		}
		lconf = &lcore_cfg[lcore_id];
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			lconf->startup_cfg[task_id].lconf = lconf;
			lconf->task[task_id] = init_task_struct(&lconf->startup_cfg[task_id]);
		}
	}
}

static void configure_if_tx_queues(struct task_startup_cfg *startup_cfg, uint8_t socket)
{
	uint8_t if_port;
	for (uint8_t i = 0; i < startup_cfg->nb_txports; ++i) {
		if_port = startup_cfg->tx_port_queue[i].port;

		TGEN_PANIC(if_port == NO_PORT_AVAIL, "port misconfigured, exiting\n");

		TGEN_PANIC((tgen_used_port_mask & (1U << if_port)) == 0, "\tPort %u not used, skipping...\n", if_port);


		startup_cfg->tx_port_queue[i].queue = tgen_port_conf[if_port].n_txqueues;
		tgen_port_conf[if_port].socket = socket;
		tgen_port_conf[if_port].n_txqueues++;

		tgen_port_conf[if_port].nb_txd = NB_TX_RING_DESC;
	}
}

static void configure_if_rx_queues(struct task_startup_cfg *startup_cfg, uint8_t socket)
{
	if (startup_cfg->rx_port != NO_PORT_AVAIL) {

		TGEN_PANIC((tgen_used_port_mask & (1U << startup_cfg->rx_port)) == 0, "Port %u not used, aborting...\n", startup_cfg->rx_port);

		startup_cfg->rx_queue = tgen_port_conf[startup_cfg->rx_port].n_rxqueues;
		tgen_port_conf[startup_cfg->rx_port].pool[startup_cfg->rx_queue] = startup_cfg->pool;
		tgen_port_conf[startup_cfg->rx_port].n_rxqueues++;
		tgen_port_conf[startup_cfg->rx_port].socket = socket;
		tgen_port_conf[startup_cfg->rx_port].nb_rxd = NB_RX_RING_DESC;
	}
}

static void configure_if_queues(void)
{
	struct lcore_cfg *lconf;
	uint8_t socket;
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master) {
			continue;
		}

		socket = rte_lcore_to_socket_id(lcore_id);
		lconf = &lcore_cfg[lcore_id];
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[task_id];
			configure_if_tx_queues(startup_cfg, socket);
			configure_if_rx_queues(startup_cfg, socket);
		}
	}
}

static void init_routing_ports(void)
{
	struct lcore_cfg *lconf;
	struct task_startup_cfg *startup_cfg;
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master) {
			continue;
		}

		lconf = &lcore_cfg[lcore_id];
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			startup_cfg = &lconf->startup_cfg[task_id];
			/* need to add the actual i/f numbers to the tx ports of all the ports that are routing */
			if (startup_cfg->runtime_flags & TASK_ROUTING) {
				startup_cfg->nb_txports = get_nb_hop_ports();
				TGEN_ASSERT(startup_cfg->nb_txports < TGEN_MAX_PORTS);
				for (uint8_t i = 0; i < startup_cfg->nb_txports; ++i) {
					startup_cfg->tx_port_queue[i].port = get_hop_port(i);
				}
			}
		}
	}
}

static void check_no_mode_core(void)
{
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master) {
			continue;
		}

		struct lcore_cfg *lconf = &lcore_cfg[lcore_id];
		TGEN_PANIC((lconf->flags & PCFG_MODE) == 0,
		           "No mode assigned for core %u. Add mode= in configuration file\n", lcore_id);
	}
}

static void init_lcore_info(void)
{
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master) {
			continue;
		}
		lcore_cfg[lcore_id].corenb = lcore_id;
	}
}

static void init_rings(void)
{
	struct lcore_cfg *lconf;
	char ring_name[3] = "A";

	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master) {
			continue;
		}

		lconf = &lcore_cfg[lcore_id];
		uint8_t socket = rte_lcore_to_socket_id(lcore_id);
		mprintf("\t*** Initializing core %u ***\n", lcore_id);
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_startup_cfg *sstartup_cfg = &lconf->startup_cfg[task_id];
			if (sstartup_cfg->ring_size == 0) {
				sstartup_cfg->ring_size = RING_RX_SIZE;
			}
			uint8_t tot_nb_txrings = 0;
			for (uint8_t idx = 0; idx < MAX_PROTOCOLS; ++idx) {
				if (!sstartup_cfg->thread_list[idx].active) {
					continue;
				}

				for (uint8_t ring_idx = 0; ring_idx < sstartup_cfg->thread_list[idx].nb_threads; ++ring_idx, ++tot_nb_txrings) {
					TGEN_ASSERT(ring_idx < MAX_WT_PER_LB);
					TGEN_ASSERT(tot_nb_txrings < MAX_RINGS_PER_CORE);

					uint8_t lcore_worker = sstartup_cfg->thread_list[idx].thread_id[ring_idx];
					TGEN_PANIC(!rte_lcore_is_enabled(lcore_worker) || lcore_worker == tgen_cfg.master, "Invalid worker: lcore %u is not enabled\n", lcore_worker);
					struct lcore_cfg *lworker = &lcore_cfg[lcore_worker];
					uint8_t dest_task = sstartup_cfg->thread_list[idx].dest_task;
					struct task_startup_cfg *dstartup_cfg = &lworker->startup_cfg[dest_task];
					TGEN_PANIC(!(dstartup_cfg->flags & PORT_STARTUP_RX_RING), "Invalid worker: lcore %u task %u is not expecting to receive through a ring\n", lcore_worker, dest_task);

					TGEN_PANIC(dest_task >= lworker->nb_tasks, "Invalid worker: lcore %u task %u not configured\n", lcore_worker, dest_task);

					mprintf("\t\tCreating ring (size: %u) to connect core %u (socket %u) with worker core %u worker %u...\n",
					        sstartup_cfg->ring_size, lcore_id, socket, lcore_worker, ring_idx);
					/* socket used is the one that the sending core resides on */
					struct rte_ring *ring = rte_ring_create(ring_name, sstartup_cfg->ring_size, socket, RING_F_SP_ENQ | RING_F_SC_DEQ);
					TGEN_PANIC(ring == NULL, "Cannot create ring to connect I/O core %u with worker core %u\n", lcore_id, lcore_worker);

					ring_name[0]++;
					TGEN_ASSERT(dstartup_cfg->nb_rxrings < MAX_RINGS_PER_CORE);
					/* will skip inactive rings */
					sstartup_cfg->tx_rings[tot_nb_txrings] = ring;
					dstartup_cfg->rx_rings[dstartup_cfg->nb_rxrings] = ring;
					++dstartup_cfg->nb_rxrings;

					dstartup_cfg->nb_slave_threads = sstartup_cfg->thread_list[idx].nb_threads;
					mprintf("\t\tCore %u port %u tx_ring[%u] => core %u task %u rx_ring[%u] %p %s %u WT\n",
					        lcore_id, task_id, ring_idx, lcore_worker, dest_task, dstartup_cfg->nb_rxrings, ring, ring->name,
					        dstartup_cfg->nb_slave_threads);
				}

				if (LB_QINQ == sstartup_cfg->mode || LB_NETWORK == sstartup_cfg->mode) {
					if (lb_nb_txrings == 0xff) {
						lb_nb_txrings = sstartup_cfg->nb_worker_threads;
					}
					else if (lb_nb_txrings != sstartup_cfg->nb_worker_threads) {
							TGEN_PANIC(tot_nb_txrings != 1, "All LB should have same number of tx_rings: %u != %u\n", lb_nb_txrings, sstartup_cfg->nb_txrings);
					}
				}
			}
		}
	}
}

#define DATA_STRUCTS_NEED_NEXT_HOP	0x0001
#define DATA_STRUCTS_NEED_GRE_TABLE	0x0002
#define DATA_STRUCTS_NEED_USER_TABLE	0x0004
#define DATA_STRUCTS_NEED_LPM_V4	0x0010
#define DATA_STRUCTS_NEED_LPM_V6	0x0020
#define DATA_STRUCTS_NEED_WT_TABLE	0x0040

/* Decide if there are cores that need specific configurations to be loaded. */
static uint16_t data_structs_needed(struct lcore_cfg* lconf, uint8_t socket_id)
{
	uint16_t config_files = 0;
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master ||
		    rte_lcore_to_socket_id(lcore_id) != socket_id) {
			continue;
		}

		lconf = &lcore_cfg[lcore_id];

		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[task_id];

			if (QINQ_DECAP_ARP == startup_cfg->mode || QINQ_DECAP_V4 == startup_cfg->mode) {
				config_files |= DATA_STRUCTS_NEED_GRE_TABLE;
			}
			if (QOS == startup_cfg->mode || CLASSIFY == startup_cfg->mode || QINQ_DECAP_V6 == startup_cfg->mode) {
				config_files |= DATA_STRUCTS_NEED_USER_TABLE;
			}
			if (ROUTING == startup_cfg->mode || FWD == startup_cfg->mode || QINQ_DECAP_V4 == startup_cfg->mode) {
				config_files |= DATA_STRUCTS_NEED_NEXT_HOP;
			}
			if (QINQ_DECAP_V4 == startup_cfg->mode || FWD == startup_cfg->mode || ROUTING == startup_cfg->mode) {
				config_files |= DATA_STRUCTS_NEED_LPM_V4;
			}
			if (QINQ_DECAP_V6 == startup_cfg->mode) {
				config_files |= DATA_STRUCTS_NEED_LPM_V6;
			}
			if (LB_QINQ == startup_cfg->mode) {
				config_files |= DATA_STRUCTS_NEED_WT_TABLE;
			}
		}
	}
	return config_files;
}

static void setup_mempools(struct lcore_cfg* lcore_cfg)
{
	char name[64];
	struct lcore_cfg *lconf = 0;
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master) {
			continue;
		}
		lconf = &lcore_cfg[lcore_id];
		uint8_t socket = rte_lcore_to_socket_id(lcore_id);
		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[task_id];

			if (startup_cfg->rx_port != NO_PORT_AVAIL) {
				/* allocate memory pool for packets */
				if (startup_cfg->nb_mbuf == 0) {
					startup_cfg->nb_mbuf = tgen_cfg.nb_mbuf;
				}

				/* use this pool for the interface that the core is receiving from */
				sprintf(name, "core_%u_port_%u_pool", lcore_id, task_id);
				startup_cfg->pool = rte_mempool_create(name,
								       startup_cfg->nb_mbuf - 1, MBUF_SIZE,
								       MAX_PKT_BURST * 4,
								       sizeof(struct rte_pktmbuf_pool_private),
								       rte_pktmbuf_pool_init, NULL,
								       tgen_pktmbuf_init, lconf,
								       socket, 0);
				TGEN_PANIC(startup_cfg->pool == NULL, "\t\tError: cannot create mempool for core %u port %u\n", lcore_id, task_id);
				mprintf("\t\tMempool %p size = %u * %u cache %u, socket %d\n", startup_cfg->pool,
					startup_cfg->nb_mbuf, MBUF_SIZE, MAX_PKT_BURST * 4, socket);
			}
		}
	}
}

static void setup_arp_entries(struct rte_hash* cpe_table)
{
	uint32_t nb_buckets = (cpe_table->bucket_bitmask + 1);
	for (uint32_t bucket_index = 0; bucket_index < nb_buckets; ++bucket_index) {
		uint32_t *sig_bucket = (hash_sig_t *) & (cpe_table->sig_tbl[bucket_index * cpe_table->sig_tbl_bucket_size]);
		uint32_t table_index = bucket_index * cpe_table->bucket_entries;
		uint8_t *entry_bucket = (uint8_t *)&cpe_table->key_tbl[bucket_index * cpe_table->bucket_entries * cpe_table->key_tbl_key_size];
		for (uint32_t pos = 0; pos < cpe_table->bucket_entries; ++pos, ++table_index) {
			struct cpe_table_hash_entry *entry = (struct cpe_table_hash_entry *)&entry_bucket[pos * cpe_table->key_tbl_key_size];
			sig_bucket[pos] = NULL_SIGNATURE;
			entry->data.tsc = MAX_TSC;
		}
	}
}

/* Initialize cores and allocate mempools */
static void init_lcores(void)
{
	char name[64];
	struct lcore_cfg *lconf = 0;
	static uint8_t *worker_thread_table[MAX_SOCKETS] = {0};
	static uint16_t *user_table[MAX_SOCKETS] = {0};
	struct rte_lpm *ipv4_lpm[MAX_SOCKETS] = {0};
	struct rte_hash *qinq_to_gre_lookup[MAX_SOCKETS] = {0};
	struct next_hop_struct *next_hop[MAX_SOCKETS] = {0};

	/* need to allocate mempools as the first thing to use the lowest possible address range */
	setup_mempools(lcore_cfg_init);

	lcore_cfg = rte_zmalloc_socket("lcore_cfg_hp", RTE_MAX_LCORE * sizeof(struct lcore_cfg), CACHE_LINE_SIZE, rte_socket_id());
	TGEN_PANIC(lcore_cfg == NULL, "Could not allocate memory for core control structures\n");
	rte_memcpy(lcore_cfg, lcore_cfg_init, RTE_MAX_LCORE * sizeof(struct lcore_cfg));

	init_lcore_info();
	check_no_mode_core();

	mprintf("=== Initializing rings on cores ===\n");
	init_rings();

	for (uint8_t socket_id = 0; socket_id < MAX_SOCKETS; ++socket_id) {
		uint16_t data_structs_flags = data_structs_needed(lconf, socket_id);
		if (data_structs_flags & DATA_STRUCTS_NEED_WT_TABLE) {
			worker_thread_table[socket_id] = rte_zmalloc_socket(NULL , 0x1000000, CACHE_LINE_SIZE, socket_id);
			TGEN_PANIC(worker_thread_table == NULL, "Error creating worker thread table");
		}

		if (data_structs_flags & DATA_STRUCTS_NEED_GRE_TABLE) {
			mprintf("=== user <-> QinQ table configuration ===\n");
			qinq_to_gre_lookup[socket_id] = read_gre_table_config(config_path, "gre_table.cfg", worker_thread_table[socket_id], lb_nb_txrings, socket_id);
			TGEN_PANIC(NULL == qinq_to_gre_lookup[socket_id], "Failed to allocate qinq to gre lookup table\n");
		}

		if (data_structs_flags & DATA_STRUCTS_NEED_USER_TABLE) {
			mprintf("=== User table configuration ===\n");
			user_table[socket_id] = read_user_table_config(config_path, "user_table.cfg", &qinq_to_gre_lookup[socket_id], socket_id);
			TGEN_PANIC(NULL == user_table[socket_id], "Failed to allocate user lookup table\n");
		}

		if (data_structs_flags & DATA_STRUCTS_NEED_NEXT_HOP) {
			mprintf("=== Next hop configuration ===\n");
			next_hop[socket_id] = read_next_hop_config(config_path, "next_hop.cfg", &tgen_used_port_mask, socket_id);
			init_routing_ports();
		}

		if (data_structs_flags & DATA_STRUCTS_NEED_LPM_V4) {
			mprintf("=== IPv4 routing configuration ===\n");
			ipv4_lpm[socket_id] = read_lpm_v4_config(config_path, "ipv4.cfg", socket_id);
			TGEN_PANIC(NULL == ipv4_lpm[socket_id], "Failed to allocate IPv4 LPM\n");
		}

		if (data_structs_flags & DATA_STRUCTS_NEED_LPM_V6) {
			mprintf("=== IPv6 routing configuration ===\n");
			read_lpm_v6_config(config_path, "ipv6.cfg", socket_id);
		}
	}

	check_consistent_cfg();

	mprintf("=== Initializing tables, mempools and queue numbers on cores ===\n");
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (!rte_lcore_is_enabled(lcore_id) || lcore_id == tgen_cfg.master) {
			continue;
		}

		lconf = &lcore_cfg[lcore_id];
		uint8_t socket = rte_lcore_to_socket_id(lcore_id);

		for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
			struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[task_id];

			if (QOS == startup_cfg->mode) {
				rte_snprintf(name, sizeof(name), "qos_sched_port_%u_%u", lcore_id, task_id);

				startup_cfg->qos_conf.port_params.name = name;
				startup_cfg->qos_conf.port_params.socket = socket;
				startup_cfg->qos_conf.port_params.rate = TEN_GIGABIT;
				startup_cfg->sched_port = rte_sched_port_config(&startup_cfg->qos_conf.port_params);

				TGEN_PANIC(startup_cfg->sched_port == NULL, "failed to create sched_port");

				mprintf("number of pipes: %d\n\n", startup_cfg->qos_conf.port_params.n_pipes_per_subport);
				int err = rte_sched_subport_config(startup_cfg->sched_port, 0, startup_cfg->qos_conf.subport_params);
				TGEN_PANIC(err != 0, "Failed setting up sched_port subport, error: %d", err);

				/* only single subport and single pipe profile is supported */
				for (uint32_t pipe = 0; pipe < startup_cfg->qos_conf.port_params.n_pipes_per_subport; ++pipe) {
					err = rte_sched_pipe_config(startup_cfg->sched_port, 0 , pipe, 0);
					TGEN_PANIC(err != 0, "failed setting up sched port pipe, error: %d", err);
				}
			}
			if (LB_QINQ == startup_cfg->mode) {
				startup_cfg->worker_thread_table = worker_thread_table[rte_socket_id()];
			}
			if (QINQ_DECAP_ARP == startup_cfg->mode || QINQ_DECAP_V4 == startup_cfg->mode) {
				startup_cfg->qinq_gre = qinq_to_gre_lookup[rte_socket_id()];
			}
			if (QOS == startup_cfg->mode || CLASSIFY == startup_cfg->mode || QINQ_DECAP_V6 == startup_cfg->mode) {
				startup_cfg->user_table = user_table[rte_socket_id()];
			}
			if (ROUTING == startup_cfg->mode || FWD == startup_cfg->mode || QINQ_DECAP_V4 == startup_cfg->mode) {
				startup_cfg->next_hop = next_hop[rte_socket_id()];
			}
			if (QINQ_DECAP_V4 == startup_cfg->mode || FWD == startup_cfg->mode || ROUTING == startup_cfg->mode) {
				startup_cfg->ipv4_lpm = ipv4_lpm[rte_socket_id()];
			}

		}

		mprintf("\t*** Initializing core %u ***\n", lcore_id);
		if (lconf->flags & PCFG_CPETABLEv4) {
			sprintf(name, "core_%u_CPEv4Table", lcore_id);

			uint8_t table_part = lconf->startup_cfg[0].nb_slave_threads;
			if (!rte_is_power_of_2(table_part)) {
				table_part = rte_align32pow2(table_part) >> 1;
			}

			struct rte_hash_parameters hash_params = {
				.name = name,
				.entries = MAX_GRE / table_part,
				.bucket_entries = GRE_BUCKET_ENTRIES,
				.key_len = sizeof(struct hash_gre_struct),
				.entry_len = sizeof(struct cpe_table_hash_entry),
				.hash_func_init_val = 0,
				.socket_id = socket
			};
			lconf->cpe_v4_table = rte_hash_ext_create(&hash_params);
			TGEN_PANIC(lconf->cpe_v4_table == NULL, "Unable to allocate memory for IPv4 hash table on core %u\n", lcore_id);

			/* set all entries to expire at MAX_TSC (i.e. never) so that we don't waste cycles at startup going through all the empty entries */
			setup_arp_entries(lconf->cpe_v4_table);

			/* for locality, copy the pointer to the port structure where it is needed at packet handling time */
			for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {
				if (lconf->startup_cfg[task_id].flags & PORT_STARTUP_CPEv4) {
					lconf->startup_cfg[task_id].cpe_table = lconf->cpe_v4_table;
				}
			}
		}

		if (lconf->flags & PCFG_CPETABLEv6) {
			sprintf(name, "core_%u_CPEv6Table", lcore_id);

			uint8_t table_part = lconf->startup_cfg[0].nb_slave_threads;
			if (!rte_is_power_of_2(table_part)) {
				table_part = rte_align32pow2(table_part) >> 1;
			}

			struct rte_hash_parameters hash_params = {
				.name = name,
				.entries = MAX_GRE / table_part,
				.bucket_entries = GRE_BUCKET_ENTRIES,
				.key_len = sizeof(struct in6_addr),
				.entry_len = sizeof(struct cpe_table_hash_entry),
				.hash_func_init_val = 0,
				.socket_id = socket
			};
			lconf->cpe_v6_table = rte_hash_ext_create(&hash_params);
			TGEN_PANIC(lconf->cpe_v6_table == NULL, "Unable to allocate memory for IPv6 hash table on core %u\n", lcore_id);

			for (uint8_t task_id = 0; task_id < lconf->nb_tasks; ++task_id) {

				if (lconf->startup_cfg[task_id].flags & PORT_STARTUP_CPEv6) {
					lconf->startup_cfg[task_id].cpe_table = lconf->cpe_v6_table;
				}
			}
		}
	}

	configure_if_queues();
	setup_all_task_structs();
}

/* Initialize all active ports */
static void init_ports(uint8_t nb_ports)
{
	int ret;
	char dummy_pool_name[] = {'0'};
	/* initialize each port */
	mprintf("=== Initializing ports ===\n");
	mprintf("\tused port mask is 0x%x\n", tgen_used_port_mask);
	for (uint8_t portid = 0; portid < nb_ports; ++portid) {
		/* skip ports that are not enabled */
		if ((tgen_used_port_mask & (1U << portid)) == 0) {
			continue;
		}

		/* initialize port */
		mprintf("\t*** Initializing port %u ***\n", portid);
		if ((tgen_cfg.flags & TGSF_USE_VF) && (tgen_port_conf[portid].n_txqueues > 1)) {
			mprintf("\t\tport %u has more than 1 core assigned (not valid in virtualization mode)\n", portid);
			/* do not skip initializing this port, it will fail later if indeed not supported by VF PMD */
		}
		if (tgen_port_conf[portid].n_rxqueues == 0) {
			/* not receiving on this port */
			mprintf("\t\tPort %u had no RX queues, setting to 1\n", portid);
			tgen_port_conf[portid].n_rxqueues = 1;
			tgen_port_conf[portid].nb_rxd = NB_RX_RING_DESC;
			tgen_port_conf[portid].pool[0] = rte_mempool_create(dummy_pool_name, NB_RX_RING_DESC, MBUF_SIZE,
							       MAX_PKT_BURST * 4,
			                                       sizeof(struct rte_pktmbuf_pool_private),
			                                       rte_pktmbuf_pool_init, NULL,
			                                       tgen_pktmbuf_init, 0,
			                                       tgen_port_conf[portid].socket, 0);
			dummy_pool_name[0]++;
		}
		if (tgen_port_conf[portid].n_txqueues == 0) {
			/* not sending on this port */
			mprintf("\t\tPort %u had no TX queues, setting to 1\n", portid);
			tgen_port_conf[portid].n_txqueues = 1;
			tgen_port_conf[portid].nb_txd = NB_TX_RING_DESC;
		}

		mprintf("\t\tConfiguring port %u... with %u RX queues and %u TX queues\n",
		        portid,
		        tgen_port_conf[portid].n_rxqueues,
		        tgen_port_conf[portid].n_txqueues);

		ret = rte_eth_dev_configure(portid, tgen_port_conf[portid].n_rxqueues,
		                            tgen_port_conf[portid].n_txqueues, &port_conf);
		TGEN_PANIC(ret < 0, "\t\t\trte_eth_dev_configure() failed on port %u: error %d\n", portid, ret);

		switch (if_cfg_startup[portid].type) {
		case IF_CFG_HW:
			rte_eth_macaddr_get(portid, &if_cfg[portid]);
			break;
		case IF_CFG_RAND:
			srand(rte_rdtsc());
			for (uint8_t i = 0; i < 6; ++i) {
				if_cfg[portid].addr_bytes[i] = rand();
			}
			break;
		case IF_CFG_SET:
			if_cfg[portid] = if_cfg_startup[portid].addr;
			break;
		}
		mprintf("\t\tMAC address set to "MAC_BYTES_FMT"\n", MAC_BYTES(if_cfg[portid].addr_bytes));

		/* initialize RX queues */
		for (uint16_t queue_id = 0; queue_id < tgen_port_conf[portid].n_rxqueues; ++queue_id) {
			mprintf("\t\tSetting up RX queue %u on port %u on socket %u with %u desc (pool 0x%p)\n",
			        queue_id, portid, tgen_port_conf[portid].socket,
			        tgen_port_conf[portid].nb_rxd, tgen_port_conf[portid].pool[queue_id]);

			ret = rte_eth_rx_queue_setup(portid, queue_id,
			                             tgen_port_conf[portid].nb_rxd,
			                             tgen_port_conf[portid].socket, &rx_conf,
			                             tgen_port_conf[portid].pool[queue_id]);

			TGEN_PANIC(ret < 0, "\t\t\trte_eth_rx_queue_setup() failed on port %u: error %d\n", portid, ret);
		}

		/* initialize one TX queue per logical core on each port */
		for (uint16_t queue_id = 0; queue_id < tgen_port_conf[portid].n_txqueues; ++queue_id) {
			mprintf("\t\tSetting up TX queue %u on socket %u with %u desc\n",
			        queue_id, tgen_port_conf[portid].socket, tgen_port_conf[portid].nb_txd);
			ret = rte_eth_tx_queue_setup(portid, queue_id, tgen_port_conf[portid].nb_txd,
			                             tgen_port_conf[portid].socket, &tx_conf);
			TGEN_PANIC(ret < 0, "\t\t\trte_eth_tx_queue_setup() failed on port %u: error %d\n", portid, ret);
		}

		mprintf("\t\tStarting up port %u...", portid);
		ret = rte_eth_dev_start(portid);

		TGEN_PANIC(ret < 0, "\n\t\t\trte_eth_dev_start() failed on port %u: error %d\n", portid, ret);
		mprintf(" done: ");

		/* get link status */
		struct rte_eth_link link;
		rte_eth_link_get(portid, &link);
		if (link.link_status) {
			port_status[portid] = 1;
			mprintf("Link Up - speed %'u Mbps - %s\n",
			        link.link_speed,
			        (link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
			        "full-duplex" : "half-duplex");
		}
		else {
			port_status[portid] = 0;
			mprintf("Link Down\n");
		}

		if (tgen_cfg.flags & TGSF_PROMISCUOUS) {
			rte_eth_promiscuous_enable(portid);
			mprintf("\t\tport %u in promiscuous mode\n", portid);
		}

		if ((tgen_cfg.flags & TGSF_USE_VF) == 0) {
			for (uint8_t i = 0; i < 16; ++i) {
				ret = rte_eth_dev_set_rx_queue_stats_mapping(portid, i, i);
				if (ret) {
					mprintf("\t\trte_eth_dev_set_rx_queue_stats_mapping() failed: error %d\n", ret);
				}
				ret = rte_eth_dev_set_tx_queue_stats_mapping(portid, i, i);
				if (ret) {
					mprintf("\t\trte_eth_dev_set_tx_queue_stats_mapping() failed: error %d\n", ret);
				}
			}
		}
	}
}

int main(int argc, char **argv)
{
	/* set en_US locale to print big numbers with ',' */
	setlocale(LC_NUMERIC, "en_US.utf-8");

	if (tgen_parse_args(argc, argv) != 0 ||
	                tgen_read_config_file() != 0 ||
	                tgen_setup_rte(argv[0]) != 0) {
		tgen_usage(argv[0]);
	}

	if (tgen_cfg.flags & TGSF_CHECK_SYNTAX) {
		mprintf("=== Configuration file syntax has been checked ===\n\n");
		return 0;
	}

	rte_cfg.nb_ports = init_rte_dev();

	mprintf("=== Calibrating TSC overhead ===\n");
	tgen_init_tsc_overhead();
	mprintf("\tTSC running at %"PRIu64" Hz\n", rte_get_tsc_hz());

	if (rte_cfg.nb_ports > 0) {
		init_lcores();

		init_ports(rte_cfg.nb_ports);

		if (tgen_cfg.flags & TGSF_CHECK_INIT) {
			mprintf("=== Initialization sequence completed ===\n\n");
			return 0;
		}
		/* start main loop */
		run(tgen_cfg.flags);
	}

	return 0;
}
