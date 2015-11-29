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

#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

#include <rte_launch.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>

#include "commands.h"
#include "tgen_args.h"
#include "main.h"
#include "display.h"


volatile int stop_tgen = 0; /* set to 1 to stop tgen */

uint64_t tgen_used_core_mask;

#ifdef ENABLE_VERBOSITY
uint8_t verbose = 0;
#endif

static void print_rx_tx_info(void)
{
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		// master core is skipped here, since it does not have its ACTIVE flag set
		if (lcore_cfg[lcore_id].flags & PCFG_ACTIVE) {
			for (uint8_t task_id = 0; task_id < lcore_cfg[lcore_id].nb_tasks; ++task_id) {
				struct task_startup_cfg *startup_cfg = &lcore_cfg[lcore_id].startup_cfg[task_id];

				mprintf("Core %u:", lcore_id);
				if (startup_cfg->rx_port != NO_PORT_AVAIL) {
					mprintf(" RX port %u (queue %u)", startup_cfg->rx_port, startup_cfg->rx_queue);
				}
				else {
					for (uint8_t j = 0; j < startup_cfg->nb_rxrings; ++j) {
						mprintf(" RX ring[%u,%u] %p", task_id, j, startup_cfg->rx_rings[j]);
					}
				}
				mprintf(" ==>");
				for (uint8_t j = 0; j < startup_cfg->nb_txports; ++j) {
					mprintf(" TX port %u (queue %u)", startup_cfg->tx_port_queue[j].port,
					        startup_cfg->tx_port_queue[j].queue);
				}

				for (uint8_t j = 0; j < startup_cfg->nb_txrings; ++j) {
					mprintf(" TX ring %p", startup_cfg->tx_rings[j]);
				}

				mprintf("\n");
			}
			tgen_used_core_mask |= __UINT64_C(1) << lcore_id;
		}
	}
}

static void init_used_core_mask(void)
{
	/* get mask of cores with active configuration */
	tgen_used_core_mask = 0;

	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		// master core is skipped here, since it does not have its ACTIVE flag set
		if (lcore_cfg[lcore_id].flags & PCFG_ACTIVE) {
			tgen_used_core_mask |= __UINT64_C(1) << lcore_id;
		}
	}
}

static void process_input(struct screen_state *screen_state)
{
	static unsigned lcore_id, task_id, nb_packets, id;
	static char mode[20];
	const char *str = get_key(screen_state);

	if (str == NULL) {
		// No command entered
	}
	else if (sscanf(str, "tcpdump %u %u %u", &lcore_id, &task_id, &nb_packets) == 3) {
		uint64_t mask = __UINT64_C(1) << lcore_id;
		if ((lcore_id >= RTE_MAX_LCORE) || !(tgen_used_core_mask & mask)) {
			mprintf("Invalid core id %u\n", lcore_id);
		}
		else {
			cmd_tcpdump(lcore_id, task_id, nb_packets);
		}
	}
	else if (sscanf(str, "hashdump %u %u %u", &lcore_id, &task_id, &id) == 3) {
		uint64_t mask = __UINT64_C(1) << lcore_id;
		if ((lcore_id >= RTE_MAX_LCORE) || !(tgen_used_core_mask & mask)) {
			mprintf("Invalid core id %u\n", lcore_id);
		}
		else if (task_id >= lcore_cfg[lcore_id].nb_tasks) {
			mprintf("Task %u does not exist for core %u\n", task_id, lcore_id);
		}
		else {
			cmd_hashdump(lcore_id, task_id, id);
		}
	}
#ifdef ENABLE_VERBOSITY
	else if (sscanf(str, "verbose %u", &id) == 1) {
		verbose = id;
	}
#endif
	else if (sscanf(str, "stat %10s %u", mode, &id) == 2) {
		cmd_stat(mode, id);
	}
	else if (sscanf(str, "stop %u", &lcore_id) == 1) {
		uint64_t mask = __UINT64_C(1) << lcore_id;
		if ((lcore_id >= RTE_MAX_LCORE) || !(tgen_used_core_mask & mask)) {
			mprintf("Invalid core id %u\n", lcore_id);
		}
		else {
			mprintf("Sending Stop to core #%u\n", lcore_id);
			stop_core(mask);
		}
	}
	else if (sscanf(str, "start %u", &lcore_id) == 1) {
		uint64_t mask = __UINT64_C(1) << lcore_id;
		if ((lcore_id >= RTE_MAX_LCORE) || !(tgen_used_core_mask & mask)) {
			mprintf("Invalid core id %u\n", lcore_id);
		}
		else {
			mprintf("Sending Start to core #%u\n", lcore_id);
			start_core(mask);
		}
	}
	else if (strcmp(str, "stop all") == 0) {
		mprintf("Sending Stop to set of cores 0x%lx\n", tgen_used_core_mask);
		stop_core(tgen_used_core_mask);
	}
	else if (strcmp(str, "start all") == 0) {
		mprintf("Sending Start to set of cores 0x%lx\n", tgen_used_core_mask);
		start_core(tgen_used_core_mask);
	}
	else if (strcmp(str, "reset stats") == 0) {
		reset_stats();
	}
	else if (strcmp(str, "help") == 0) {
		mprintf("Available commands:\n");
		mprintf("\tstart all\n");
		mprintf("\tstop all\n");
		mprintf("\tstart <core id>\n");
		mprintf("\tstop <core id>\n");
		mprintf("\tverbose <level>\n");
		mprintf("\tstat <kind> <value>\n");
		mprintf("\treset stats\n");
		mprintf("\ttcpdump <core id> <nb packets>\n");
		mprintf("\thashdump <core id> <table id>\n");
	}
}

static void wait_all_cores(void)
{
	/* wait for all tasks to terminate */
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		// master core is skipped here, since it does not have its ACTIVE flag set
		if (lcore_cfg[lcore_id].flags & PCFG_ACTIVE) {
			rte_eal_wait_lcore(lcore_id);
		}
	}
}

/* start main loop */
void __attribute__((noreturn)) run(uint32_t flags)
{
	init_display(tgen_cfg.start_time);
	struct screen_state screen_state;
	memset(&screen_state, 0, sizeof(screen_state));
	stats_display_layout(screen_state);

	init_used_core_mask();
	print_rx_tx_info();

	/* start all tasks on worker cores */
	if (flags & TGSF_AUTOSTART) {
		start_core(tgen_used_core_mask);
	}

#ifndef BRAS_STATS
	while(1) {sleep(1000000);}
#endif

	unsigned tick_count = 0;
	uint64_t cur_tsc = rte_rdtsc();
	uint64_t next_update = cur_tsc + rte_get_tsc_hz();
	uint64_t stop_tsc = 0;
	if (tgen_cfg.duration_time != 0) {
		stop_tsc = cur_tsc + tgen_cfg.start_time*rte_get_tsc_hz() + tgen_cfg.duration_time*rte_get_tsc_hz();
	}
	// 0 is default screen to display, 1 shows port related stats
	while (stop_tgen == 0) {
		cur_tsc = rte_rdtsc();
		if (tick_count != 99) {
			usleep(10000);
		}
		else {
			if (next_update > cur_tsc) {
				usleep((next_update - cur_tsc)*1000000/rte_get_tsc_hz());
			}
			next_update += rte_get_tsc_hz();
		}

		struct screen_state old = screen_state;
		process_input(&screen_state);
		if (old.chosen_screen != screen_state.chosen_screen ||
		                old.chosen_page != screen_state.chosen_page) {
			// change display state. next call to show stats will display the active screen
			stats_display_layout(screen_state);
		}

		if (++tick_count == 100) {
			tick_count = 0;

			update_stats();

			if (screen_state.chosen_screen == 0) {
				display_stats_core_ports(screen_state.chosen_page);
			}
			else if (screen_state.chosen_screen == 1) {
				display_stats_eth_ports();
			}
		}
		if (stop_tsc && cur_tsc >= stop_tsc) {
			stop_core(tgen_used_core_mask);
			stop_tgen = 1;
		}
	}

	mprintf("total RX: %"PRIu64", total TX: %"PRIu64", avarage RX: %"PRIu64" pps, avarage TX: %"PRIu64" pps\n",
		global_total_rx(),
		global_total_tx(),
		global_avg_rx(),
		global_avg_tx());
	wait_all_cores();
	end_display();
	exit(EXIT_SUCCESS);
}
