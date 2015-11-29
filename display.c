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

#include <curses.h>
#include <rte_cycles.h>

#include "display.h"
#include "commands.h"
#include "main.h"
#include "stats.h"
#include "tgen_args.h"
#include "display_utils.h"
#include "tgen_assert.h"

#define VERSION_STR "010"
/* Set up the display mutex  as recursive. This enables threads to use
   display_[un]lock() to lock  the display when multiple  calls to for
   instance mprintf() need to be made. */

static pthread_mutex_t mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;

static void display_lock(void)
{
	pthread_mutex_lock(&mutex);
}

static void display_unlock(void)
{
	pthread_mutex_unlock(&mutex);
}

#ifdef BRAS_STATS

struct global_stats {
	uint64_t tx_pps;
	uint64_t rx_pps;
	uint64_t tx_tot;
	uint64_t rx_tot;
	uint64_t rx_tot_beg;
	uint64_t tx_tot_beg;
	uint64_t avg_start;
	uint8_t  started_avg;
	uint64_t rx_avg;
	uint64_t tx_avg;
};

struct port_stats {
	uint64_t tot_tx_pkt_count;
	uint64_t tot_tx_pkt_drop;
	uint64_t tot_rx_pkt_count;

	uint64_t prev_tsc;
	uint64_t cur_tsc;

	uint32_t diff_tx_pkt_count;
	uint32_t diff_tx_pkt_drop;
	uint32_t diff_rx_pkt_count;
	uint32_t diff_empty_cycles;

	uint32_t last_tx_pkt_count;
	uint32_t last_tx_pkt_drop;
	uint32_t last_rx_pkt_count;
	uint32_t last_empty_cycles;
};

struct core_port {
	struct stats *stats;
	struct port_stats *port_stats;
	uint8_t lcore_id;
	uint8_t port_id;
	/* flags set if total RX/TX values need to be reported set at
	   initialization time, only need to access stats values in port */
	uint8_t flags;
};

struct lcore_stats {
	struct port_stats port_stats[MAX_TASKS_PER_CORE];
};

struct eth_stats {
	uint64_t no_mbufs;
	uint64_t ierrors;
	uint64_t prev_no_mbufs;
	uint64_t prev_ierrors;
};

/* Advanced text output */
static WINDOW *scr = NULL, *win_txt, *win_cmd, *win_stat, *win_title, *win_help;

/* Stores all readed values from the cores, displaying is done afterwards because
   displaying introduces overhead. If displaying was done right after the values
   are read, inaccuracy is introduced for later cores */
static struct lcore_stats  lcore_stats[RTE_MAX_LCORE];
static struct core_port    core_ports[RTE_MAX_LCORE *MAX_TASKS_PER_CORE];
static struct core_port    *core_port_ordered[RTE_MAX_LCORE*MAX_TASKS_PER_CORE];
static struct global_stats global_stats;
static struct eth_stats    eth_stats[16];
static uint8_t nb_tasks_tot;
static uint8_t nb_interface;
static uint16_t core_port_height;
static uint64_t start_tsc;

/* Colors used in the interface */
#define WHITE_ON_BLUE           1
#define BLACK_ON_CYAN           2
#define BLACK_ON_WHITE          3
#define BLACK_ON_YELLOW         4
#define WHITE_ON_RED            5
#define GREEN_ON_BLUE           6
#define YELLOW_ON_BLUE          7

static WINDOW *create_subwindow(int height, int width, int y_pos, int x_pos)
{
	WINDOW *win = subwin(scr, height, width, y_pos, x_pos);
	touchwin(scr);
	return win;
}

/* Format string capable [mv]waddstr() wrappers */
#define waddstrf(win, fmt, ...) do {			\
		char buf[1024];				\
		snprintf(buf, 1024, fmt, __VA_ARGS__);	\
		waddstr(win, buf);			\
	} while (0)

#define mvwaddstrf(win, y, x, fmt, ...) do {		\
		wmove(win, y, x);			\
		waddstrf(win, fmt, __VA_ARGS__);	\
	} while (0)


#define PORT_STATS_RX 0x01
#define PORT_STATS_TX 0x02


// Red: link down; Green: link up
static short link_color(const uint8_t if_port)
{
	return COLOR_PAIR(port_status[if_port] ? GREEN_ON_BLUE : WHITE_ON_RED);
}

static void init_core_port(struct core_port *core_port, uint8_t lcore_id, uint8_t port_id, struct stats *stats, uint8_t rx_flag, uint8_t tx_flag)
{
	core_port->lcore_id = lcore_id;
	core_port->port_id = port_id;
	core_port->stats = stats;

	core_port->port_stats = &lcore_stats[lcore_id].port_stats[port_id];
	core_port->flags |= rx_flag;
	core_port->flags |= tx_flag;
}

static struct core_port *set_line_no(const uint8_t lcore_id, const uint8_t port_id)
{
	for (uint8_t active_core_port = 0; active_core_port < nb_tasks_tot; ++active_core_port) {
		struct core_port *core_port = &core_ports[active_core_port];
		if (lcore_id == core_port->lcore_id && port_id == core_port->port_id) {
			return core_port;
		}
	}
	return NULL;
}

static void init_active_eth_ports(void)
{
	nb_interface = rte_eth_dev_count();
}

/* Populate active_core_ports for stats reporting, the order of the cores matters
   for reporting the most accurate results. TX cores should updated first (to prevent
   negative Loss stats). This will also calculate the number of core ports used by
   other display functions. */
static void init_active_core_ports(void)
{
	struct lcore_cfg *lconf;
	/* add cores that are receiving from and sending to physical ports first */
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		lconf = &lcore_cfg[lcore_id];
		if (lconf->flags & PCFG_ACTIVE) {
			for (uint8_t port_id = 0; port_id < lconf->nb_tasks; ++port_id) {
				struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[port_id];
				struct stats *stats = lconf->task[port_id]->stats;
				if (startup_cfg->nb_rxrings == 0 && startup_cfg->nb_txrings == 0) {
					init_core_port(&core_ports[nb_tasks_tot], lcore_id, port_id, stats, PORT_STATS_RX, PORT_STATS_TX);
					++nb_tasks_tot;
				}
			}
		}
	}

	/* add cores that are sending to physical ports second */
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		lconf = &lcore_cfg[lcore_id];
		if (lconf->flags & PCFG_ACTIVE) {
			for (uint8_t port_id = 0; port_id < lconf->nb_tasks; ++port_id) {
				struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[port_id];
				struct stats *stats = lconf->task[port_id]->stats;
				if (startup_cfg->nb_rxrings != 0 && startup_cfg->nb_txrings == 0) {
					init_core_port(&core_ports[nb_tasks_tot], lcore_id, port_id, stats, 0, PORT_STATS_TX);
					++nb_tasks_tot;
				}
			}
		}
	}

	/* add cores that are receiving from physical ports third */
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		lconf = &lcore_cfg[lcore_id];
		if (lconf->flags & PCFG_ACTIVE) {
			for (uint8_t port_id = 0; port_id < lconf->nb_tasks; ++port_id) {
				struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[port_id];
				struct stats *stats = lconf->task[port_id]->stats;
				if (startup_cfg->nb_rxrings == 0 && startup_cfg->nb_txrings != 0) {
					init_core_port(&core_ports[nb_tasks_tot], lcore_id, port_id, stats, PORT_STATS_RX, 0);
					++nb_tasks_tot;
				}
			}
		}
	}

	/* add cores that are working internally (no physical ports attached) */
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		lconf = &lcore_cfg[lcore_id];
		if (lconf->flags & PCFG_ACTIVE) {
			for (uint8_t port_id = 0; port_id < lconf->nb_tasks; ++port_id) {
				struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[port_id];
				struct stats *stats = lconf->task[port_id]->stats;
				if (startup_cfg->nb_rxrings != 0 && startup_cfg->nb_txrings != 0) {
					init_core_port(&core_ports[nb_tasks_tot], lcore_id, port_id, stats, 0, 0);
					++nb_tasks_tot;
				}
			}
		}
	}
}

static void update_tsc_stats(void)
{
	for (uint8_t task_id = 0; task_id < nb_tasks_tot; ++task_id) {
		const uint8_t lcore_id = core_ports[task_id].lcore_id;
		const uint8_t port_id = core_ports[task_id].port_id;
		struct port_stats *cur_port_stats = &lcore_stats[lcore_id].port_stats[port_id];
		cur_port_stats->prev_tsc = cur_port_stats->cur_tsc;
	}

	for (uint8_t port_id = 0; port_id < nb_interface; ++port_id) {
		if (tgen_used_port_mask & (1U << port_id)) {
			eth_stats[port_id].prev_no_mbufs = eth_stats[port_id].no_mbufs;
			eth_stats[port_id].prev_ierrors = eth_stats[port_id].ierrors;
		}
	}
}

void init_display(unsigned avg_start)
{
	scr = initscr();
	start_color();
	/* Assign default foreground/background colors to color number -1 */
	use_default_colors();

	init_pair(WHITE_ON_BLUE,   COLOR_WHITE,  COLOR_BLUE);
	init_pair(BLACK_ON_CYAN,   COLOR_BLACK,  COLOR_CYAN);
	init_pair(BLACK_ON_WHITE,  COLOR_BLACK,  COLOR_WHITE);
	init_pair(BLACK_ON_YELLOW, COLOR_BLACK,  COLOR_YELLOW);
	init_pair(WHITE_ON_RED,    COLOR_WHITE,  COLOR_RED);
	init_pair(GREEN_ON_BLUE,   COLOR_GREEN,  COLOR_BLUE);
	init_pair(YELLOW_ON_BLUE,  COLOR_YELLOW, COLOR_BLUE);

	wbkgd(scr, COLOR_PAIR(WHITE_ON_BLUE));

	nodelay(scr, TRUE);
	noecho();

	/* Create fullscreen log window. When stats are displayed
	   later, it is recreated with appropriate dimensions. */
	win_txt = create_subwindow(0, 0, 0, 0);
	wbkgd(win_txt, COLOR_PAIR(0));

	idlok(win_txt, FALSE);
	/* Get scrolling */
	scrollok(win_txt, TRUE);
	/* Leave cursor where it was */
	leaveok(win_txt, TRUE);

	refresh();
	init_active_core_ports();
	init_active_eth_ports();

	core_port_height = (LINES - 5 - 2 - 3);
	if (core_port_height > nb_tasks_tot) {
		core_port_height = nb_tasks_tot;
	}
	start_tsc = rte_rdtsc();
	global_stats.avg_start = start_tsc + avg_start*rte_get_tsc_hz();
	update_stats();
	update_tsc_stats();
}

static void stats_display_eth_ports(void)
{
	wattron(win_stat, A_BOLD);
	/* Labels */
	mvwaddstr(win_stat, 3, 2,   "Port");
	mvwaddstr(win_stat, 4, 0,   "  Nb");
	mvwvline(win_stat, 4, 4,  ACS_VLINE, nb_interface + 2);
	mvwaddstr(win_stat, 4, 5,   "Name");

	mvwvline(win_stat, 2, 13,  ACS_VLINE, nb_interface + 3);
	mvwaddstr(win_stat, 3, 14, "      Statistics per second    ");
	mvwaddstr(win_stat, 4, 14, "   no mbufs (#)");
	mvwvline(win_stat, 4, 30,  ACS_VLINE, nb_interface + 1);
	mvwaddstr(win_stat, 4, 31, "    ierrors (#)");
	mvwvline(win_stat, 3, 47 ,  ACS_VLINE, nb_interface + 2);

	mvwaddstr(win_stat, 3, 48, "         Total Statistics      ");
	mvwaddstr(win_stat, 4, 48, "   no mbufs (#)");
	mvwvline(win_stat, 4, 64,  ACS_VLINE, nb_interface + 1);
	mvwaddstr(win_stat, 4, 65, "    ierrors (#)");
	mvwvline(win_stat, 3, 81,  ACS_VLINE, nb_interface + 2);

	wattroff(win_stat, A_BOLD);

	for (uint8_t i = 0; i < nb_interface; ++i) {
		mvwaddstrf(win_stat, 5 + i, 0, "%4u", i);
		mvwaddstrf(win_stat, 5 + i, 5, "%8s", (tgen_used_port_mask & (1U << i))? if_cfg_startup[i].name : "N/A");
	}
}

static void stats_display_core_ports(unsigned chosen_page)
{
	/* Sub-section separator lines */
	mvwvline(win_stat, 4,  4,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 23,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 43,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 53,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 63,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 88,  ACS_VLINE, nb_tasks_tot + 1);
	mvwvline(win_stat, 4, 103, ACS_VLINE, nb_tasks_tot + 1);

	wattron(win_stat, A_BOLD);
	/* Section separators (bold) */
	mvwvline(win_stat, 3, 13, ACS_VLINE, nb_tasks_tot + 2);
	mvwvline(win_stat, 3, 33, ACS_VLINE, nb_tasks_tot + 2);
	mvwvline(win_stat, 3, 73, ACS_VLINE, nb_tasks_tot + 2);

	/* Labels */
	mvwaddstr(win_stat, 3, 2,   "Core");
	mvwaddstr(win_stat, 4, 0,   "  Nb");
	mvwaddstr(win_stat, 4, 5,   "Name");

	mvwaddstr(win_stat, 3, 14, " Port Nb/Ring Name");
	mvwaddstr(win_stat, 4, 14, "       RX");
	mvwaddstr(win_stat, 4, 24, "       TX");

	mvwaddstr(win_stat, 3, 34, "         Statistics per second         ");
	mvwaddstr(win_stat, 4, 34, " Idle (%)");
#ifdef FULL_PRECISION_STATS
	mvwaddstr(win_stat, 4, 44, "   RX    ");
	mvwaddstr(win_stat, 4, 54, "   TX    ");
	mvwaddstr(win_stat, 4, 64, " Drop    ");
#else
	mvwaddstr(win_stat, 4, 44, "   RX (k)");
	mvwaddstr(win_stat, 4, 54, "   TX (k)");
	mvwaddstr(win_stat, 4, 64, " Drop (k)");
#endif

	mvwaddstr(win_stat, 3, 74, "              Total Statistics             ");
	mvwaddstr(win_stat, 4, 74, "            RX");
	mvwaddstr(win_stat, 4, 89, "            TX");
	mvwaddstr(win_stat, 4, 104, "          Drop");
	wattroff(win_stat, A_BOLD);

	uint16_t line_no = 0;
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		const struct lcore_cfg *const cur_core = &lcore_cfg[lcore_id];

		if (!(cur_core->flags & PCFG_ACTIVE)) {
			continue;
		}

		for (uint8_t task_id = 0; task_id < cur_core->nb_tasks; ++task_id) {
			const struct task_startup_cfg *const startup_cfg = &cur_core->startup_cfg[task_id];

			if (line_no >= core_port_height * chosen_page && line_no < core_port_height * (chosen_page + 1)) {
				// Core number and name
				mvwaddstrf(win_stat, line_no % core_port_height + 5, 0, "%2u/%1u", lcore_id, task_id);
				mvwaddstrf(win_stat, line_no % core_port_height + 5, 5, "%s", task_id == 0 ? cur_core->name : "");

				// Rx port information
				if (startup_cfg->nb_rxrings == 0) {
					wbkgdset(win_stat, link_color(startup_cfg->rx_port));
					mvwaddstrf(win_stat, line_no % core_port_height + 5, 16, "%2u", startup_cfg->rx_port);
					wbkgdset(win_stat, COLOR_PAIR(WHITE_ON_BLUE));
				}

				for (uint8_t ring_id = 0; ring_id < startup_cfg->nb_rxrings; ++ring_id) {
					mvwaddstr(win_stat, line_no % core_port_height + 5, 14 + ring_id, startup_cfg->rx_rings[ring_id]->name);
				}

				// Tx port information
				if (startup_cfg->runtime_flags & TASK_ROUTING) {
					wbkgdset(win_stat, COLOR_PAIR(YELLOW_ON_BLUE));
					mvwaddstr(win_stat, line_no % core_port_height + 5, 25, "(r:");
					uint8_t pos = 28;
					for (uint8_t i = 0; i < startup_cfg->nb_txports; ++i) {
						if (i) {
							mvwaddstr(win_stat, line_no % core_port_height + 5, pos, ",");
							++pos;
						}
						wbkgdset(win_stat, link_color(startup_cfg->tx_port_queue[i].port));
						mvwaddstrf(win_stat, line_no % core_port_height + 5, pos, "%u", startup_cfg->tx_port_queue[i].port);
						wbkgdset(win_stat, COLOR_PAIR(WHITE_ON_BLUE));
						++pos;
					}
					wbkgdset(win_stat, COLOR_PAIR(YELLOW_ON_BLUE));
					mvwaddstr(win_stat, line_no % core_port_height + 5, pos, ")");
					wbkgdset(win_stat, COLOR_PAIR(WHITE_ON_BLUE));
				}
				else {
					uint8_t pos = 26;
					for (uint8_t i = 0; i < startup_cfg->nb_txports; ++i) {
						if (i) {
							mvwaddstr(win_stat, line_no % core_port_height + 5, pos, ",");
							++pos;
						}

						wbkgdset(win_stat, link_color(startup_cfg->tx_port_queue[i].port));
						mvwaddstrf(win_stat, line_no % core_port_height + 5, pos, "%u", startup_cfg->tx_port_queue[i].port);
						wbkgdset(win_stat, COLOR_PAIR(WHITE_ON_BLUE));
						pos++;
					}

					for (uint8_t ring_id = 0; ring_id < startup_cfg->nb_txrings; ++ring_id) {
						mvwaddstr(win_stat, line_no % core_port_height + 5, 24 + ring_id, startup_cfg->tx_rings[ring_id]->name);
					}
				}
			}
			TGEN_ASSERT(line_no < RTE_MAX_LCORE*MAX_TASKS_PER_CORE);
			core_port_ordered[line_no] = set_line_no(lcore_id, task_id);
			++line_no;
		}
	}
}

void stats_display_layout(struct screen_state screen_state)
{
	// moving existing windows does not work
	delwin(win_txt);
	delwin(win_title);
	delwin(win_cmd);
	delwin(win_txt);
	delwin(win_help);

	clear();
	uint8_t cur_stats_height;
	if (screen_state.chosen_screen == 0) {
		cur_stats_height = core_port_height;
	}
	else {
		cur_stats_height = nb_interface;
	}

	win_stat = create_subwindow(cur_stats_height + 5, 0, 0, 0);
	win_title = create_subwindow(3, 40, 0, 41);
	win_cmd = create_subwindow(1, 0, cur_stats_height + 5,  0);
	win_txt = create_subwindow(LINES - cur_stats_height - 5 - 2, 0, cur_stats_height + 5 + 1, 0);
	win_help = create_subwindow(1, 0, LINES - 1, 0);

	/* Title box */
	wbkgd(win_title, COLOR_PAIR(BLACK_ON_CYAN));
	box(win_title, 0, 0);
	mvwaddstr(win_title, 1, 15, "BRAS v." VERSION_STR);

	/* Stats labels and separator lines */
	/* Upper left stats block */
	mvwaddstr(win_stat, 0, 0, "Time:");
	mvwaddstr(win_stat, 0, 12, "%:");
	mvwaddstrf(win_stat, 1, 0, "Rx: %10s pps (%10s avg)", "", "");
	mvwaddstrf(win_stat, 2, 0, "Tx: %10s pps (%10s avg)", "", "");

	/* Upper right stats block */
	mvwaddstr(win_stat, 0, 100, "Rx:");
	mvwaddstr(win_stat, 1, 100, "Tx:");
	mvwaddstr(win_stat, 2, 100, "Loss");

	if (screen_state.chosen_screen == 0) {
		stats_display_core_ports(screen_state.chosen_page);
	}
	else {
		stats_display_eth_ports();
	}

	/* Command line */
	wbkgd(win_cmd, COLOR_PAIR(BLACK_ON_YELLOW));
	idlok(win_cmd, FALSE);
	/* Move cursor at insertion point */
	leaveok(win_cmd, FALSE);

	/* Help/status bar */
	wbkgd(win_help, COLOR_PAIR(BLACK_ON_WHITE));
	waddstr(win_help, "Enter 'help' or command, <ESC> or 'quit' to exit...");
	wrefresh(win_help);

	/* Log window */
	idlok(win_txt, FALSE);
	/* Get scrolling */
	scrollok(win_txt, TRUE);
	/* Leave cursor where it was */
	leaveok(win_txt, TRUE);

	wbkgd(win_txt, COLOR_PAIR(BLACK_ON_CYAN));
	wrefresh(win_txt);

	/* Draw everything to the screen */
	refresh();
}

void end_display(void)
{
	pthread_mutex_destroy(&mutex);

	if (scr != NULL) {
		endwin();
	}
}


static void update_global_stats(uint8_t task_id, struct global_stats *global_stats)
{
	const struct port_stats *port_stats = core_ports[task_id].port_stats;
	const uint64_t delta_t = port_stats->cur_tsc - port_stats->prev_tsc;

	if (core_ports[task_id].flags & PORT_STATS_RX) {
		global_stats->rx_tot += port_stats->diff_rx_pkt_count;
		global_stats->rx_pps += (port_stats->diff_rx_pkt_count) * rte_get_tsc_hz() / delta_t;
	}

	if (core_ports[task_id].flags & PORT_STATS_TX) {
		global_stats->tx_tot += port_stats->diff_tx_pkt_count;
		global_stats->tx_pps += (port_stats->diff_tx_pkt_count) * rte_get_tsc_hz() / delta_t;
	}
}

static void display_core_port_stats(uint8_t task_id)
{
	const int line_no = task_id % core_port_height;

	const struct port_stats *port_stats = core_port_ordered[task_id]->port_stats;

	/* delta_t in units of clock ticks */
	uint64_t delta_t = port_stats->cur_tsc - port_stats->prev_tsc;

	uint64_t empty_cycles = port_stats->diff_empty_cycles;

	if (empty_cycles > delta_t) {
		empty_cycles = 10000;
	}
	else {
		empty_cycles = empty_cycles * 10000 / delta_t;
	}

	// empty_cycles has 2 digits after point, (usefull when only a very small idle time)
	mvwaddstrf(win_stat, line_no + 5, 37, "%3lu.%02lu", empty_cycles / 100, empty_cycles % 100);

#ifdef  HIGH_PREC_STATS
	mvwaddstrf(win_stat, line_no + 5, 44, "%9lu", port_stats->diff_rx_pkt_count * rte_get_tsc_hz() / delta_t);
	mvwaddstrf(win_stat, line_no + 5, 54, "%9lu", port_stats->diff_tx_pkt_count * rte_get_tsc_hz() / delta_t);
	mvwaddstrf(win_stat, line_no + 5, 64, "%9lu", port_stats->diff_tx_pkt_drop * rte_get_tsc_hz() / delta_t);
#else
	// Display per second statistics in Kpps unit
	delta_t *= 1000;

	uint64_t nb_pkt;
	nb_pkt = port_stats->diff_rx_pkt_count * rte_get_tsc_hz();
	if (nb_pkt && nb_pkt < delta_t) {
		mvwaddstrf(win_stat, line_no + 5, 44, "    0.%03lu", nb_pkt * 1000 / delta_t);
	}
	else {
		mvwaddstrf(win_stat, line_no + 5, 44, "%9lu", nb_pkt / delta_t);
	}

	nb_pkt = port_stats->diff_tx_pkt_count * rte_get_tsc_hz();
	if (nb_pkt && nb_pkt < delta_t) {
		mvwaddstrf(win_stat, line_no + 5, 54, "    0.%03lu", nb_pkt * 1000 / delta_t);
	}
	else {
		mvwaddstrf(win_stat, line_no + 5, 54, "%9lu", nb_pkt / delta_t);
	}

	nb_pkt = port_stats->diff_tx_pkt_drop * rte_get_tsc_hz();
	if (nb_pkt && nb_pkt < delta_t) {
		mvwaddstrf(win_stat, line_no + 5, 64, "    0.%03lu", nb_pkt * 1000 / delta_t);
	}
	else {
		mvwaddstrf(win_stat, line_no + 5, 64, "%9lu", nb_pkt / delta_t);
	}
#endif

	// Total statistics (packets)
	mvwaddstrf(win_stat, line_no + 5, 74, "%14lu", port_stats->tot_rx_pkt_count);
	mvwaddstrf(win_stat, line_no + 5, 89, "%14lu", port_stats->tot_tx_pkt_count);
	mvwaddstrf(win_stat, line_no + 5, 104, "%14lu", port_stats->tot_tx_pkt_drop);
}

void update_stats(void)
{
	if (nb_tasks_tot == 0) {
		return;
	}

	for (uint8_t task_id = 0; task_id < nb_tasks_tot; ++task_id) {
		struct stats *stats = core_ports[task_id].stats;
		struct port_stats *cur_port_stats = core_ports[task_id].port_stats;

		/* Read TX first and RX second, in order to prevent displaying
		   a negative packet loss. Depending on the configuration
		   (when forwarding, for example), TX might be bigger than RX. */

		cur_port_stats->cur_tsc = rte_rdtsc();
		cur_port_stats->diff_tx_pkt_count = rte_atomic32_read(&stats->tx_pkt_count) - cur_port_stats->last_tx_pkt_count;
		cur_port_stats->diff_tx_pkt_drop  = rte_atomic32_read(&stats->tx_pkt_drop)  - cur_port_stats->last_tx_pkt_drop;
		cur_port_stats->diff_rx_pkt_count = rte_atomic32_read(&stats->rx_pkt_count) - cur_port_stats->last_rx_pkt_count;
		cur_port_stats->diff_empty_cycles = rte_atomic32_read(&stats->empty_cycles) - cur_port_stats->last_empty_cycles;
	}

	for (uint8_t task_id = 0; task_id < nb_tasks_tot; ++task_id) {
		struct port_stats *cur_port_stats = core_ports[task_id].port_stats;

		/* no total stats for empty loops */
		cur_port_stats->tot_rx_pkt_count  += cur_port_stats->diff_rx_pkt_count;
		cur_port_stats->last_rx_pkt_count += cur_port_stats->diff_rx_pkt_count;
		cur_port_stats->tot_tx_pkt_count  += cur_port_stats->diff_tx_pkt_count;
		cur_port_stats->last_tx_pkt_count += cur_port_stats->diff_tx_pkt_count;
		cur_port_stats->tot_tx_pkt_drop   += cur_port_stats->diff_tx_pkt_drop;
		cur_port_stats->last_tx_pkt_drop  += cur_port_stats->diff_tx_pkt_drop;

		cur_port_stats->last_empty_cycles += cur_port_stats->diff_empty_cycles;
	}

	global_stats.tx_pps = 0;
	global_stats.rx_pps = 0;

	for (uint8_t task_id = 0; task_id < nb_tasks_tot; ++task_id) {
		update_global_stats(task_id, &global_stats);
	}

	struct rte_eth_stats eth_stat;

	for (uint8_t port_id = 0; port_id < nb_interface; ++port_id) {
		if (tgen_used_port_mask & (1U << port_id)) {
			rte_eth_stats_get(port_id, &eth_stat);

			eth_stats[port_id].no_mbufs = eth_stat.rx_nombuf;
			eth_stats[port_id].ierrors = eth_stat.ierrors;
		}
	}
}

static void display_stats_general(void)
{
	uint64_t cur_tsc = rte_rdtsc();
	// Upper left stats block
	wattron(win_stat, A_BOLD);
	mvwaddstrf(win_stat, 0,  6, "%4lu", (cur_tsc - start_tsc)/rte_get_tsc_hz());
	mvwaddstrf(win_stat, 0, 15, "%.4f",  global_stats.tx_tot * 100.0 / global_stats.rx_tot);
	mvwaddstrf(win_stat, 1,  4, "%10lu", global_stats.rx_pps);
	mvwaddstrf(win_stat, 2,  4, "%10lu", global_stats.tx_pps);

	if (cur_tsc > global_stats.avg_start) {
		if (!global_stats.started_avg) {
			global_stats.rx_tot_beg = global_stats.rx_tot;
			global_stats.tx_tot_beg = global_stats.tx_tot;
			global_stats.started_avg = 1;
		}
		else {
			uint32_t avg_time_passed = (cur_tsc - global_stats.avg_start)/rte_get_tsc_hz();
			global_stats.rx_avg = (global_stats.rx_tot - global_stats.rx_tot_beg)/avg_time_passed;
			global_stats.tx_avg = (global_stats.tx_tot - global_stats.tx_tot_beg)/avg_time_passed;
			mvwaddstrf(win_stat, 1,  20, "%10lu", global_stats.rx_avg);
			mvwaddstrf(win_stat, 2,  20, "%10lu", global_stats.tx_avg);
		}
	}

	// Upper right stats block
	mvwaddstrf(win_stat, 0, 106, "%12lu", global_stats.rx_tot);
	mvwaddstrf(win_stat, 1, 106, "%12lu", global_stats.tx_tot);
	mvwaddstrf(win_stat, 2, 106, "%12lu", global_stats.rx_tot - global_stats.tx_tot);
	wattroff(win_stat, A_BOLD);
}

void display_stats_core_ports(__attribute__((unused)) unsigned chosen_page)
{
	pthread_mutex_lock(&mutex);
	for (uint8_t active_core = core_port_height * chosen_page; active_core < nb_tasks_tot && active_core < core_port_height * (chosen_page + 1); ++active_core) {
		display_core_port_stats(active_core);
	}
	display_stats_general();
	pthread_mutex_unlock(&mutex);

	update_tsc_stats();
	wrefresh(win_stat);
}

void display_stats_eth_ports(void)
{
	pthread_mutex_lock(&mutex);

	for (uint8_t port_id = 0; port_id < nb_interface; ++port_id) {
		if (tgen_used_port_mask & (1U << port_id)) {
			mvwaddstrf(win_stat, 5 + port_id, 14, "%16lu", eth_stats[port_id].no_mbufs - eth_stats[port_id].prev_no_mbufs);
			mvwaddstrf(win_stat, 5 + port_id, 31, "%16lu", eth_stats[port_id].ierrors - eth_stats[port_id].prev_ierrors);

			mvwaddstrf(win_stat, 5 + port_id, 48, "%16lu", eth_stats[port_id].no_mbufs);
			mvwaddstrf(win_stat, 5 + port_id, 65, "%16lu", eth_stats[port_id].ierrors);
		}
	}

	display_stats_general();
	pthread_mutex_unlock(&mutex);
	update_tsc_stats();
	wrefresh(win_stat);
}

char *get_key(struct screen_state *screen_state)
{
#define MAX_STRID      64
#define MAX_CMDLEN     256
	static int lastid = 0;
	static int readid = -1;
	static char str[MAX_CMDLEN] = {0};
	static char last_strs[MAX_STRID][MAX_CMDLEN] = {{0}};
	static unsigned len = 0;

	int testid;
	struct key_val key_val;
	poll_key(&key_val);

	switch (key_val.type) {
	case TYPE_SPECIAL_KEY:
		switch (key_val.val) {
		case KEY_ENTER:
			readid = -1;
			if (len == 0) {
				break;
			}
			len = 0;
			if (strstr(str, "quit") == NULL) {
				lastid = (lastid + 1) % MAX_STRID;
				strncpy(last_strs[lastid], str, MAX_CMDLEN);
				memset(str, 0, sizeof(str));
				werase(win_cmd);
				wrefresh(win_cmd);
				return last_strs[lastid];
			}
			mprintf("Leaving...\n");
			stop_core(tgen_used_core_mask);
			stop_tgen = 1;
			break;
		case KEY_BACKSPACE:

			if (len > 0) {
				str[--len] = '\0';
				werase(win_cmd);
				waddstr(win_cmd, str);
			}

			break;
		case KEY_ESC:
			/* ESC */
			mprintf("Leaving...\n");
			stop_core(tgen_used_core_mask);
			stop_tgen = 1;

			break;
		case KEY_UP:
			if (readid == -1) {
				testid = lastid;
			}
			else {
				testid = (readid + MAX_STRID - 1) % MAX_STRID;
			}
			if (last_strs[testid][0] != '\0') {
				readid = testid;
				strcpy(str, last_strs[readid]);
				len = strlen(str);
				werase(win_cmd);
				waddstr(win_cmd, str);
			}

			break;
		case KEY_DOWN:
			if (readid == -1) {
				testid = (lastid + 1) % MAX_STRID;
			}
			else {
				testid = (readid + 1) % MAX_STRID;
			}
			if (last_strs[testid][0] != '\0') {
				readid = testid;
				strcpy(str, last_strs[readid]);
				len = strlen(str);
				werase(win_cmd);
				waddstr(win_cmd, str);
			}
			break;
		case KEY_F(1):
			screen_state->chosen_screen = 0;
			break;
		case KEY_F(2):
			screen_state->chosen_screen = 1;
			break;
		case KEY_PPAGE:
			if (screen_state->chosen_page) {
				--screen_state->chosen_page;
			}
			break;
		case KEY_NPAGE:
			if (nb_tasks_tot > core_port_height * (screen_state->chosen_page + 1)) {
				++screen_state->chosen_page;
			}
			break;
		}
		break;
	case TYPE_NORMAL_KEY:
		if (len < sizeof(str) - 1) {
			str[len++] = key_val.val;
			mvwaddstr(win_cmd, 0, 0, str);
		}
		break;
	case TYPE_NOT_SUPPORTED:
		mprintf("ESC2 car %i:'%c' not supported\n", key_val.val, key_val.val);
		break;
	case TYPE_NO_KEY:
		break;
	}
	wrefresh(win_cmd);
	return NULL;
}

void reset_stats(void)
{
	memset(&global_stats, 0, sizeof(struct global_stats));
}

uint64_t global_total_tx(void)
{
	return global_stats.tx_tot;
}

uint64_t global_total_rx(void)
{
	return global_stats.rx_tot;
}

uint64_t global_avg_tx(void)
{
	return global_stats.tx_avg;
}

uint64_t global_avg_rx(void)
{
	return global_stats.rx_avg;
}

#endif

void mprintf(const char *format, ...)
{


	if (format == NULL) {
		return;
	}

	va_list ap;
	char buffer[1024];

	va_start(ap, format);
	vsnprintf(buffer, 1024, format, ap);
	va_end(ap);


	display_lock();

	// Output to log file
	static FILE *fp = NULL;
	if (fp == NULL) {
		fp = fopen("tgen.log", "w");
	}
	if (fp != NULL) {
		fputs(buffer, fp);
		fflush(fp);
	}

#ifdef BRAS_STATS
	// Output to screen
	if (scr == NULL) {
		// ncurses is not yet initialized
		fputs(buffer, stdout);
		fflush(stdout);
	}
	else {
		waddstr(win_txt, buffer);
		wrefresh(win_txt);
	}
#endif
	display_unlock();
}

#ifndef BRAS_STATS

void init_display(__attribute__((unused)) unsigned avg_start){}
void stats_display_layout(__attribute__((unused)) struct screen_state screen_state){}
void end_display(void){}

char *get_key(__attribute__((unused)) struct screen_state *screen_state){return 0;}

void reset_stats(void){}
void update_stats(void){}
void display_stats_core_ports(__attribute((unused)) unsigned chosen_page){}
void display_stats_eth_ports(void){}

uint64_t global_total_tx(void) {return 0;}
uint64_t global_total_rx(void) {return 0;}
uint64_t global_avg_tx(void) {return 0;}
uint64_t global_avg_rx(void) {return 0;}

#endif
