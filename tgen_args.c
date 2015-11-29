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

#include <unistd.h>
#include <libgen.h>

#include <rte_sched.h>
#include <rte_string_fns.h>

#include "defines.h"
#include "tgen_args.h"
#include "cfgfile.h"
#include "main.h"
#include "display.h"
#include "parse_utils.h"

#define MAX_RTE_ARGV 64
#define MAX_ARG_LEN  32

#define NB_MBUF 8*1024

/* Helper macro */
#define STR_EQ(s1, s2)	(!strcmp((s1), (s2)))

/* configuration files support */
static int get_rte_cfg(unsigned sindex, char *str, void *data);
static int get_global_cfg(unsigned sindex, char *str, void *data);
static int get_interface_cfg(unsigned sindex, char *str, void *data);
static int get_core_cfg(unsigned sindex, char *str, void *data);

static struct cfg_section eal_default_cfg = {
	.name   = "eal options",
	.parser = get_rte_cfg,
	.data   = &rte_cfg,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

static struct cfg_section interface_cfg = {
	.name   = "interface #",
	.parser = get_interface_cfg,
	.data   = &if_cfg_startup,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

static struct cfg_section settings_cfg = {
	.name   = "global",
	.parser = get_global_cfg,
	.data   = &tgen_cfg,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

static struct cfg_section core_cfg = {
	.name   = "core #",
	.parser = get_core_cfg,
	.data   = lcore_cfg_init,
	.indexp[0]  = 0,
	.nbindex = 1,
	.error  = 0
};

#define QUEUE_SIZES 128
#define NB_PIPES 32768

static struct rte_sched_port_params port_params_default = {
	.name = "port_0",
	.socket = 0,
	.mtu = 6 + 6 + 4 + 4 + 2 + 1500,
	.rate = 0,
	.frame_overhead = RTE_SCHED_FRAME_OVERHEAD_DEFAULT,
	.n_subports_per_port = 1,
	.n_pipes_per_subport = NB_PIPES,
	.qsize = {QUEUE_SIZES, QUEUE_SIZES, QUEUE_SIZES, QUEUE_SIZES},
	.pipe_profiles = NULL,
	.n_pipe_profiles = 1 /* only one profile */
};

static struct rte_sched_pipe_params pipe_params_default = {
	.tb_rate = TEN_GIGABIT / NB_PIPES,
	.tb_size = 4000000,

	.tc_rate = {TEN_GIGABIT / NB_PIPES, TEN_GIGABIT / NB_PIPES, TEN_GIGABIT / NB_PIPES, TEN_GIGABIT / NB_PIPES},
	.tc_period = 40,

	.wrr_weights = {1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1},
};

static struct rte_sched_subport_params subport_params_default = {
	.tb_rate = TEN_GIGABIT,
	.tb_size = 4000000,
	.tc_rate = {TEN_GIGABIT, TEN_GIGABIT, TEN_GIGABIT, TEN_GIGABIT},
	.tc_period = 40, /* default was 10 */
};

static const char *cfg_file = DEFAULT_CONFIG_FILE;
const char *config_path = NULL;

struct rte_cfg    rte_cfg;
struct if_cfg_startup if_cfg_startup[TGEN_MAX_PORTS];
/* contains the source mac address for all the
   interfaces to be set when transmitting packets */
struct ether_addr if_cfg[TGEN_MAX_PORTS];
struct tgen_cfg   tgen_cfg;
struct lcore_cfg *lcore_cfg;
// only used at initialization time
struct lcore_cfg  lcore_cfg_init[RTE_MAX_LCORE];

uint8_t name_to_if(const char *name)
{
	for (uint8_t i = 0; i < TGEN_MAX_PORTS; ++i) {
		if (STR_EQ(name, if_cfg_startup[i].name)) {
			return i;
		}
	}

	return NO_PORT_AVAIL;
}

/* [eal options] parser */
static int get_rte_cfg(__attribute__((unused))unsigned sindex, char *str, void *data)
{
	struct rte_cfg *pconfig = (struct rte_cfg *)data;

	if (str == NULL || pconfig == NULL) {
		return -1;
	}

	char *pkey = get_cfg_key(str);
	if (pkey == NULL) {
		return -1;
	}

	if (STR_EQ(str, "-m")) {
		pconfig->memory = atoi(pkey);
		return 0;
	}
	if (STR_EQ(str, "-n")) {
		pconfig->force_nchannel = atoi(pkey);
		if (pconfig->force_nchannel == 0 || pconfig->force_nchannel > 4) {
			mprintf("\t\tInvalid number of memory channels\n");
			return -1;
		}
		return 0;
	}
	if (STR_EQ(str, "-r")) {
		pconfig->force_nrank = atoi(pkey);
		if (pconfig->force_nrank == 0 || pconfig->force_nrank > 16) {
			mprintf("\t\tInvalid number of memory ranks\n");
			return -1;
		}
		return 0;
	}
	/* debug options */
#define PARSE_YESNO(s, v)			\
	if (STR_EQ(str, s)) {			\
		if (STR_EQ(pkey, "yes")) {	\
			v = 1;			\
			return 0;		\
		}				\
		if (STR_EQ(pkey, "no")) {	\
			v = 0;			\
			return 0;		\
		}				\
		return -1;			\
	}
	PARSE_YESNO("no-pci", pconfig->no_pci);
	PARSE_YESNO("no-hpet", pconfig->no_hpet);
	PARSE_YESNO("no-shconf", pconfig->no_shconf);
	PARSE_YESNO("no-huge", pconfig->no_hugetlbfs);
	PARSE_YESNO("no-output", pconfig->no_output);
#undef PARSE_YESNO
	if (STR_EQ(str, "huge-dir")) {
		if (pconfig->hugedir) {
			free(pconfig->hugedir);
		}
		pconfig->hugedir = strdup(pkey);
		return 0;
	}

	if (STR_EQ(str, "eal")) {
		if (pconfig->eal) {
			free(pconfig->eal);
		}
		pconfig->eal = strdup(pkey);
		return 0;
	}


	/* fail on unknown keys */
	return -1;
}

/* [global] parser */
static int get_global_cfg(__attribute__((unused))unsigned sindex, char *str, void *data)
{
	struct tgen_cfg *pset = (struct tgen_cfg *)data;

	if (str == NULL || pset == NULL) {
		return -1;
	}

	char *pkey = get_cfg_key(str);
	if (pkey == NULL) {
		return -1;
	}

	if (STR_EQ(str, "master core")) {
		pset->master = (uint32_t)atoi(pkey);
		if (pset->master >= RTE_MAX_LCORE) {
			return -1;
		}
		return 0;
	}

	if (STR_EQ(str, "start time")) {
		pset->start_time = (uint32_t)atoi(pkey);
		return 0;
	}

	if (STR_EQ(str, "duration time")) {
		pset->duration_time = (uint32_t)atoi(pkey);
		return 0;
	}

#define PARSE_YESNO(s, f)			\
	if (STR_EQ(str, s)) {			\
		if (STR_EQ(pkey, "yes")) {	\
			pset->flags |= (f);	\
			return 0;		\
		}				\
		if (STR_EQ(pkey, "no")) {	\
			pset->flags &= ~(f);	\
			return 0;		\
		}				\
		return -1;			\
	}
	PARSE_YESNO("enable 1G interfaces", TGSF_USE_1G);
	PARSE_YESNO("enable virtualization", TGSF_USE_VF);
	PARSE_YESNO("enable promiscuous", TGSF_PROMISCUOUS);
#undef PARSE_YESNO
	if (STR_EQ(str, "mempool size")) {
		pset->nb_mbuf = (uint32_t)atoi(pkey);
		return 0;
	}

	/* fail on unknown keys */
	return -1;
}

/* [interface] parser */
static int get_interface_cfg(unsigned sindex, char *str, void *data)
{
	struct if_cfg_startup *cfg = (struct if_cfg_startup *)data;

	uint8_t cur_if = sindex & ~CFG_INDEXED;

	if (cur_if >= TGEN_MAX_PORTS) {
		return -1;
	}

	cfg = &if_cfg_startup[cur_if];

	if (str == NULL || data == NULL) {
		return -1;
	}

	char *pkey = get_cfg_key(str);

	if (pkey == NULL) {
		return -1;
	}


	if (STR_EQ(str, "mac")) {
		if (STR_EQ(pkey, "hardware")) {
			cfg->type = IF_CFG_HW;
		}
		else if (STR_EQ(pkey, "random")) {
			cfg->type = IF_CFG_RAND;
		}
		else {
			cfg->type = IF_CFG_SET;
			if (get_ether_addr(&cfg->addr, pkey)) {
				return -1;
			}
		}
	}
	else if (STR_EQ(str, "name")) {
		for (uint8_t i = 0; i < TGEN_MAX_PORTS; ++i) {
			if (i != cur_if && STR_EQ(pkey, if_cfg_startup[i].name)) {
				// each name has to be unique
				return -1;
			}
		}

		strncpy(cfg->name, pkey, MAX_NAME_SIZE);
	}

	return 0;
}

/* [core] parser */
static int get_core_cfg(unsigned sindex, char *str, void *data)
{
	char *pkey;
	struct lcore_cfg *lconf = (struct lcore_cfg *)data;

	if (str == NULL || lconf == NULL || !(sindex & CFG_INDEXED)) {
		return -1;
	}

	pkey = get_cfg_key(str);
	if (pkey == NULL) {
		return -1;
	}

	uint32_t ncore = sindex & ~CFG_INDEXED;
	if (ncore >= RTE_MAX_LCORE) {
		return -1;
	}

	lconf = &lconf[ncore];

#ifdef DEPRECATED_SUPPORT
	if (STR_EQ(str, "port")) {
		mprintf("error: port is deprecated. Use task instead.\n");
		return -1;
	}
#endif
	if (STR_EQ(str, "task")) {

		uint32_t val = atoi(pkey);
		if (val >= MAX_TASKS_PER_CORE) {
			mprintf("error: to many tasks for core. Maximum allowed is %d\n", MAX_TASKS_PER_CORE);
			return -1;
		}
		lconf->active_task = val;

		lconf->startup_cfg[lconf->active_task].task = lconf->active_task;

		if (lconf->nb_tasks < lconf->active_task + 1) {
			lconf->nb_tasks = lconf->active_task + 1;
		}
		return 0;
	}

	struct task_startup_cfg *startup_cfg = &lconf->startup_cfg[lconf->active_task];
	if (STR_EQ(str, "tx port from route") && STR_EQ(pkey, "yes")) {
		startup_cfg->runtime_flags |= TASK_ROUTING;

		if (startup_cfg->mode != QINQ_DECAP_V4 && startup_cfg->mode != QINQ_DECAP_V6) {
			mprintf("tx port form route not supported in modes other than qinq decap v4 and qinq decap v6\n");
			return -1;
		}
		return 0;
	}

	if (STR_EQ(str, "update mac on arp") && STR_EQ(pkey, "yes")) {
		startup_cfg->runtime_flags |= TASK_UPDATE_MAC_ONLY_ARP;
		return 0;
	}

	if (STR_EQ(str, "tx port from learned mac")) {
		char *ports[TGEN_MAX_PORTS];
		uint8_t ret = rte_strsplit(pkey, strlen(pkey), ports, TGEN_MAX_PORTS, ',');

		if (ret == 0) {
			return -1;
		}
		uint32_t val;
		for (uint8_t i = 0; i < ret; ++i) {
			if ((val = name_to_if(ports[i])) == NO_PORT_AVAIL) {
				return -1;
			}
			startup_cfg->tx_port_queue[i].port = val;
			startup_cfg->mapping[val] = i;
		}

		startup_cfg->nb_txports = ret;
		startup_cfg->runtime_flags |= TASK_PORT_FROM_LEARNED_MAC;

		return 0;
	}

	/* Using tx port name, only a _single_ port can be assigned to a task. */
#ifdef DEPRECATED_SUPPORT
	if (STR_EQ(str, "tx port index")) {
		mprintf("error: tx port index is deprecated. Use tx port name=port_name instead."
		        "port_name should be defined in the [interface x] section.");
		return -1;
	}
#endif
	if (STR_EQ(str, "tx port name")) {
		uint32_t val;
		if ((val = name_to_if(pkey)) == NO_PORT_AVAIL) {
			mprintf("No interface found with name: %s\n", pkey);
			return -1;
		}
		if (val < TGEN_MAX_PORTS) {
			tgen_used_port_mask |= (1U << val);

			if (startup_cfg->nb_txports) {
				mprintf("only one tx port can be defined per port. If more ports are needed, use a LB for example\n");
				return -1;
			}
			startup_cfg->tx_port_queue[0].port = val;
			startup_cfg->nb_txports = 1;
			return 0;
		}
		return -1;
	}
	if (STR_EQ(str, "rx ring") && STR_EQ(pkey, "yes")) {
		if (startup_cfg->rx_port != NO_PORT_AVAIL) {
			mprintf("can't read both from internal ring and external port on the same port\n");
			mprintf("recommend using separate ports if this is the goal\n");
			return -1;
		}
		startup_cfg->flags |= PORT_STARTUP_RX_RING;
		return 0;
	}

	if (STR_EQ(str, "drop") && STR_EQ(pkey, "no")) {
		startup_cfg->flags |= PORT_STARTUP_NO_DROP;
		return 0;
	}
	if (STR_EQ(str, "drop") && STR_EQ(pkey, "yes")) {
		startup_cfg->flags &= ~PORT_STARTUP_NO_DROP;
		return 0;
	}

	if (STR_EQ(str, "rx port name")) {
		if (startup_cfg->flags & PORT_STARTUP_RX_RING) {
			mprintf("Can't read both from internal ring and external port on the same port.\n");
			mprintf("Recommend using separate ports if this is the goal.\n");
			return -1;
		}
		uint32_t val;
		if ((val = name_to_if(pkey)) == NO_PORT_AVAIL) {
			mprintf("No interface found with name: %s\n", pkey);
			return -1;
		}

		if (val < TGEN_MAX_PORTS) {
			tgen_used_port_mask |= (1U << val);
			startup_cfg->rx_port = val;
			return 0;
		}
		return -1;
	}

	if (STR_EQ(str, "mode")) {
		lconf->flags &= ~PCFG_MODE;
		if (STR_EQ(pkey, "none")) {
			lconf->flags |= PCFG_NONE;
			startup_cfg->mode = NONE;
		}
		if (STR_EQ(pkey, "fwd")) {
			lconf->flags |= PCFG_BRAS | PCFG_FWD;
			startup_cfg->mode = FWD;
		}
		if (STR_EQ(pkey, "drop")) {
			lconf->flags |= PCFG_DROP;
			startup_cfg->mode = COUNT_DROP;
		}
		if (STR_EQ(pkey, "routing")) {
			lconf->flags |= PCFG_BRAS;
			startup_cfg->runtime_flags |= TASK_ROUTING;
			startup_cfg->mode = ROUTING;
		}
		if (STR_EQ(pkey, "unmpls")) {
			lconf->flags |= PCFG_BRAS;
			startup_cfg->runtime_flags |= TASK_MPLS_TAGGING;
			startup_cfg->mode = UNMPLS;
		}
		if (STR_EQ(pkey, "qinqdecapv4")) {
			lconf->flags |= PCFG_BRAS | PCFG_CPETABLEv4;
			startup_cfg->flags |= PORT_STARTUP_CPEv4;
			startup_cfg->mode = QINQ_DECAP_V4;
		}
		if (STR_EQ(pkey, "qinqdecapv6")) {
			lconf->flags |= PCFG_BRAS | PCFG_CPETABLEv6;
			startup_cfg->flags |= PORT_STARTUP_CPEv6;
			startup_cfg->mode = QINQ_DECAP_V6;
		}
		if (STR_EQ(pkey, "qinqdecaparp")) {
			/* arp only implemented for ipv4 */
			lconf->flags |= PCFG_BRAS | PCFG_CPETABLEv4;
			startup_cfg->flags |= PORT_STARTUP_CPEv4;
			startup_cfg->mode = QINQ_DECAP_ARP;
		}
		if (STR_EQ(pkey, "lbqinq")) {
			lconf->flags |= PCFG_LB;
			startup_cfg->mode = LB_QINQ;
		}
		if (STR_EQ(pkey, "lbnetwork")) {
			lconf->flags |= PCFG_LB;
			startup_cfg->mode = LB_NETWORK;
		}
		if (STR_EQ(pkey, "qinqencapv4")) {
			lconf->flags |= PCFG_BRAS | PCFG_CPETABLEv4;
			startup_cfg->flags |= PORT_STARTUP_CPEv4;

			if (startup_cfg->runtime_flags & TASK_MPLS_TAGGING) {
				startup_cfg->mode = QINQ_ENCAP_V4_UNMPLS;
			}
			else {
				startup_cfg->mode = QINQ_ENCAP_V4;
			}
		}
		if (STR_EQ(pkey, "qinqencapv6")) {
			lconf->flags |= PCFG_BRAS | PCFG_CPETABLEv6;
			startup_cfg->flags |= PORT_STARTUP_CPEv6;
			if (startup_cfg->runtime_flags & TASK_MPLS_TAGGING) {
				startup_cfg->mode = QINQ_ENCAP_V6_UNMPLS;
			}
			else {
				startup_cfg->mode = QINQ_ENCAP_V6;
			}
		}
		if (STR_EQ(pkey, "qos")) {
			if (lconf->nb_tasks != 1) {
				mprintf("only support one port when doing QoS\n");
				return -1;
			}

			lconf->flags |= PCFG_QOS;
			startup_cfg->mode = QOS;
			/* configure default values for QoS (can be overwritten by values inside config file */
			startup_cfg->qos_conf.port_params = port_params_default;
			startup_cfg->qos_conf.pipe_params[0] = pipe_params_default;
			startup_cfg->qos_conf.subport_params[0] = subport_params_default;
			startup_cfg->qos_conf.port_params.pipe_profiles = startup_cfg->qos_conf.pipe_params;
		}
		if (STR_EQ(pkey, "classify")) {
			lconf->flags |= PCFG_CLASSIFY;
			startup_cfg->mode = CLASSIFY;
		}
		if (lconf->flags & PCFG_MODE) {
			lconf->flags |= PCFG_ACTIVE;
			return 0;
		}
		return -1;
	}


	if (STR_EQ(str, "network side") && STR_EQ(pkey, "yes")) {
		startup_cfg->flags |= PORT_STARTUP_NETWORK_SIDE;
		return 0;
	}

	if (STR_EQ(str, "tx threads")) {
		uint8_t dest_task = 0;
		/* if user did not specify, dest_port is left at default (first type) */
		uint8_t dest_proto = 0;

		const char *ptask = strstr(pkey, "proto=");
		if (ptask) {
			ptask += strlen("proto=");

			if (STR_EQ(ptask, "ipv4")) {
				dest_proto = IPV4;
			}
			else if (STR_EQ(ptask, "arp")) {
				dest_proto = ARP;
			}
			else if (STR_EQ(ptask, "ipv6")) {
				dest_proto = IPV6;
			}
			else {
				mprintf("proto needs to be either ipv4, arp or ipv6");
				return -1;
			}

		}

#ifdef DEPRECATED_SUPPORT
		ptask = strstr(pkey, "port=");
		if (ptask) {
			mprintf("error: port is deprecated. Use task instead.\n");
			return -1;
		}
#endif

		ptask = strstr(pkey, "task=");

		if (ptask) {
			ptask += strlen("task=");
			dest_task = atoi(ptask);
			if (dest_task > MAX_TASKS_PER_CORE) {
				return -1;
			}
		}
		else {
			dest_task = 0;
		}

		struct thread_list_cfg *thread_list_cfg = &startup_cfg->thread_list[dest_proto];
		thread_list_cfg->dest_task = dest_task;
		thread_list_cfg->active = 1;

		int8_t ret = list_set_from_str(thread_list_cfg->thread_id, pkey, MAX_WT_PER_LB);

		if (ret == -1) {
			mprintf("Error in parsing %s\n", str);
			return -1;
		}
		else {
			thread_list_cfg->nb_threads = ret;
			startup_cfg->nb_worker_threads = ret;
			startup_cfg->nb_txrings += ret;
			if (startup_cfg->nb_txrings > MAX_RINGS_PER_CORE) {
				mprintf("Maximum allowed TX rings is %u but have %u\n", MAX_RINGS_PER_CORE, startup_cfg->nb_txrings);
				return -1;
			}
			else if (thread_list_cfg->nb_threads > MAX_WT_PER_LB) {
				mprintf("Maximum worker threads allowed is %u but have %u\n", MAX_WT_PER_LB, thread_list_cfg->nb_threads);
				return -1;
			}
		}
		return 0;
	}
	if (STR_EQ(str, "tx threads from route")) {
		/* The interfaces are noted after intreface=.
		   The mapping between an interface and a
		   thread is determined by the order */

		char *if_name = strstr(pkey, " interface=");
		if (if_name == NULL) {
			mprintf("Syntax error: need to specify interfaces using interface=name0,name1\n");
			return -1;
		}

		struct thread_list_cfg *thread_list_cfg =  &startup_cfg->thread_list[0];
		thread_list_cfg->active = 1;
		int8_t ret = list_set_from_str(thread_list_cfg->thread_id, pkey, MAX_RINGS_PER_CORE);
		int8_t max_allowed = TGEN_MAX_PORTS < MAX_WT_PER_LB ? TGEN_MAX_PORTS : MAX_WT_PER_LB;
		if (ret <= 0) {
			mprintf("Error in parsing %s\n", str);
			return -1;
		}
		else if (ret > max_allowed) {
			mprintf("Maximum threads to route to is %u (given by maximum number of ports and worker threads)\n", TGEN_MAX_PORTS);
			return -1;
		}

		thread_list_cfg->nb_threads = ret;
		startup_cfg->nb_txrings = ret;

		/* there should be ret number of interfaces defined in the configuration */
		char *iface_str[TGEN_MAX_PORTS];
		if_name += strlen(" interface=");
		int n_iface = rte_strsplit(if_name, strlen(if_name), iface_str, TGEN_MAX_PORTS, ',');

		if (n_iface != ret) {
			mprintf("Error in parsing %s: expecting same number of interfaces as destination threads\n", str);
			return -1;
		}
		strip_spaces(iface_str, n_iface);
		for (uint8_t i = 0; i < n_iface; ++i) {
			if (0 == strlen(iface_str[i])) {
				mprintf("Interface without name specified\n");
				return -1;
			}

			uint8_t iface_id = name_to_if(iface_str[i]);
			if (iface_id == NO_PORT_AVAIL) {
				mprintf("interface with name %s not found", iface_str[i]);
				return -1;
			}
			startup_cfg->mapping[iface_id] = i;
		}
		startup_cfg->runtime_flags |= TASK_THREAD_FROM_INCOMING;
		return 0;
	}
	if (STR_EQ(str, "ring size")) {
		if ((startup_cfg->ring_size = atoi(pkey)) <= 0) {
			return -1;
		}
		return 0;
	}

	if (STR_EQ(str, "mempool size")) {
		startup_cfg->nb_mbuf = (uint32_t)atoi(pkey);
		return 0;
	}

	if (STR_EQ(str, "name")) {
		strncpy(lconf->name, pkey, MAX_NAME_SIZE);
		return 0;
	}
	/* MPLS configuration */
	if (STR_EQ(str, "untag mpls") && STR_EQ(pkey, "yes")) {
		startup_cfg->runtime_flags |= TASK_MPLS_TAGGING;

		if (QINQ_ENCAP_V4 == startup_cfg->mode) {
			startup_cfg->mode = QINQ_ENCAP_V4_UNMPLS;
		}
		else if (QINQ_ENCAP_V6 == startup_cfg->mode) {
			startup_cfg->mode = QINQ_ENCAP_V6_UNMPLS;
		}

		return 0;
	}

	if (STR_EQ(str, "dst mac")) { /* destination MAC address to be used for packets */
		return get_ether_addr(&startup_cfg->edaddr, pkey);
	}
	if (STR_EQ(str, "bras ipv4")) { /* source IP address to be used for packets */
		return get_ip_addr(&lconf->bras_ip, pkey);
	}
	if (STR_EQ(str, "pipes")) {
		if (!atoi(pkey) || !(rte_is_power_of_2(startup_cfg->qos_conf.port_params.n_pipes_per_subport = atoi(pkey)))) {
			return -1;
		}
		return 0;
	}
	if (STR_EQ(str, "queue size")) {
		startup_cfg->qos_conf.port_params.qsize[0] = atoi(pkey);
		startup_cfg->qos_conf.port_params.qsize[1] = atoi(pkey);
		startup_cfg->qos_conf.port_params.qsize[2] = atoi(pkey);
		startup_cfg->qos_conf.port_params.qsize[3] = atoi(pkey);
		return 0;
	}
	if (STR_EQ(str, "subport tb rate")) {
		startup_cfg->qos_conf.subport_params[0].tb_rate = STR_EQ("auto", pkey) ? TEN_GIGABIT : atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "subport tb size")) {
		startup_cfg->qos_conf.subport_params[0].tb_size = atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "subport tc 0 rate")) {
		startup_cfg->qos_conf.subport_params[0].tc_rate[0] = STR_EQ("auto", pkey) ? TEN_GIGABIT : atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "subport tc 1 rate")) {
		startup_cfg->qos_conf.subport_params[0].tc_rate[1] = STR_EQ("auto", pkey) ? TEN_GIGABIT : atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "subport tc 2 rate")) {
		startup_cfg->qos_conf.subport_params[0].tc_rate[2] = STR_EQ("auto", pkey) ? TEN_GIGABIT : atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "subport tc 3 rate")) {
		startup_cfg->qos_conf.subport_params[0].tc_rate[3] = STR_EQ("auto", pkey) ? TEN_GIGABIT : atoll(pkey);
		return 0;
	}

	if (STR_EQ(str, "subport tc rate")) {
		if (STR_EQ("auto", pkey)) {
			startup_cfg->qos_conf.subport_params[0].tc_rate[0] = TEN_GIGABIT;
			startup_cfg->qos_conf.subport_params[0].tc_rate[1] = TEN_GIGABIT;
			startup_cfg->qos_conf.subport_params[0].tc_rate[2] = TEN_GIGABIT;
			startup_cfg->qos_conf.subport_params[0].tc_rate[3] = TEN_GIGABIT;
		}
		else {
			startup_cfg->qos_conf.subport_params[0].tc_rate[0] = atoll(pkey);
			startup_cfg->qos_conf.subport_params[0].tc_rate[1] = atoll(pkey);
			startup_cfg->qos_conf.subport_params[0].tc_rate[2] = atoll(pkey);
			startup_cfg->qos_conf.subport_params[0].tc_rate[3] = atoll(pkey);
		}

		return 0;
	}
	if (STR_EQ(str, "subport tc period")) {
		startup_cfg->qos_conf.subport_params[0].tc_period = atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "pipe tb rate")) {
		if (STR_EQ("auto", pkey)) {
			uint32_t n_pipes = startup_cfg->qos_conf.port_params.n_pipes_per_subport;
			startup_cfg->qos_conf.pipe_params[0].tb_rate = TEN_GIGABIT / n_pipes;
		}
		else {
			startup_cfg->qos_conf.pipe_params[0].tb_rate = atoll(pkey);
		}
		return 0;
	}
	if (STR_EQ(str, "pipe tb size")) {
		startup_cfg->qos_conf.pipe_params[0].tb_size = atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "pipe tc rate")) {
		if (STR_EQ("auto", pkey)) {
			uint32_t n_pipes = startup_cfg->qos_conf.port_params.n_pipes_per_subport;
			n_pipes = n_pipes ? n_pipes : 1;
			startup_cfg->qos_conf.pipe_params[0].tc_rate[0] = TEN_GIGABIT / n_pipes;
			startup_cfg->qos_conf.pipe_params[0].tc_rate[1] = TEN_GIGABIT / n_pipes;
			startup_cfg->qos_conf.pipe_params[0].tc_rate[2] = TEN_GIGABIT / n_pipes;
			startup_cfg->qos_conf.pipe_params[0].tc_rate[3] = TEN_GIGABIT / n_pipes;
		}
		else {
			startup_cfg->qos_conf.pipe_params[0].tc_rate[0] = atoll(pkey);
			startup_cfg->qos_conf.pipe_params[0].tc_rate[1] = atoll(pkey);
			startup_cfg->qos_conf.pipe_params[0].tc_rate[2] = atoll(pkey);
			startup_cfg->qos_conf.pipe_params[0].tc_rate[3] = atoll(pkey);
		}
		return 0;
	}
	if (STR_EQ(str, "pipe tc 0 rate")) {
		startup_cfg->qos_conf.pipe_params[0].tc_rate[0] = atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "pipe tc 1 rate")) {
		startup_cfg->qos_conf.pipe_params[0].tc_rate[1] = atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "pipe tc 2 rate")) {
		startup_cfg->qos_conf.pipe_params[0].tc_rate[2] = atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "pipe tc 3 rate")) {
		startup_cfg->qos_conf.pipe_params[0].tc_rate[3] = atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "pipe tc period")) {
		startup_cfg->qos_conf.pipe_params[0].tc_period = atoll(pkey);
		return 0;
	}
	if (STR_EQ(str, "classify") && STR_EQ(pkey, "yes")) {
		if (startup_cfg->mode == LB_NETWORK ||
		    startup_cfg->mode == LB_QINQ) {
			return -1;        /* LB can't classify, needs to be as optimal as possible */
		}

		/* at least one core is classifying, load the user table */
		tgen_cfg.flags |= PCFG_QOS;

		startup_cfg->runtime_flags |=  TASK_CLASSIFY;
		return 0;
	}

	/* fail on unknown keys */
	return -1;
}

/* command line parameters parsing procedure */
int tgen_parse_args(int argc, char **argv)
{
	/* Default settings */
	tgen_cfg.flags |= TGSF_AUTOSTART;
	tgen_cfg.nb_mbuf = NB_MBUF;

	mprintf("=== Parsing command line ===\n");
	int opt;
	while ((opt = getopt(argc, argv, "f:aesi")) != EOF) {
		switch (opt) {
		case 'f':
			/* path to config file */
			cfg_file = optarg;
			break;
		case 'a':
			/* autostart all ports */
			tgen_cfg.flags |= TGSF_AUTOSTART;
			break;
		case 'e':
			/* don't autostart */
			tgen_cfg.flags &= ~TGSF_AUTOSTART;
			break;
		case 's':
			/* check configuration file syntax and exit */
			tgen_cfg.flags |= TGSF_CHECK_SYNTAX;
			break;
		case 'i':
			/* check initialization sequence and exit */
			tgen_cfg.flags |= TGSF_CHECK_INIT;
			break;
		default:
			mprintf("\tError while parsing command line options\n");
			return -1;
		}
	}

	/* reset getopt lib for DPDK */
	optind = 0;

	return 0;
}

static void initialize_cfg(void)
{
	// Default values
	tgen_cfg.master = RTE_MAX_LCORE;

	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		struct lcore_cfg *cur_lcore_cfg_init = &lcore_cfg_init[lcore_id];
		for (uint8_t task_id = 0; task_id < MAX_TASKS_PER_CORE; ++task_id) {
			struct task_startup_cfg *task_startup_cfg = &cur_lcore_cfg_init->startup_cfg[task_id];
			task_startup_cfg->rx_port = NO_PORT_AVAIL;

			for (uint8_t port_id = 0; port_id < TGEN_MAX_PORTS; ++port_id) {
				task_startup_cfg->tx_port_queue[port_id].port = NO_PORT_AVAIL;
			}
		}
	}
}

static int check_cfg(void)
{
	/* Sanity check */
#define RETURN_IF(cond, err)			\
	if (cond) {				\
		mprintf(err);			\
		return -1;			\
	};

	RETURN_IF(rte_cfg.force_nchannel == 0, "\tError: number of memory channels not specified in [eal options] section\n");
	RETURN_IF(tgen_cfg.master >= RTE_MAX_LCORE, "\tError: master core index not specified in [global] section\n");
	RETURN_IF(tgen_used_port_mask == 0, "\tError: No ports specified, check configuration ('tx port index' and 'rx port index')\n");
#undef RETURN_IF

	return 0;
}

/* Read config file */
int tgen_read_config_file(void)
{
	/* Initialize data structures */
	initialize_cfg();

	/* read configuration */
	mprintf("=== Parsing configuration file '%s' ===\n", cfg_file);
	struct cfg_file *pcfg = cfg_open(cfg_file);
	if (pcfg == NULL) {
		return -1;
	}
	char *full_cfg_file_path = strdup(cfg_file);
	config_path = strdup(dirname(full_cfg_file_path));
	free(full_cfg_file_path);

	typedef struct cfg_sections_t {
		const char         *name;
		struct cfg_section *data;
	} cfg_sections_t;

	cfg_sections_t config_sections[] = {
		{ "[eal options] section",      &eal_default_cfg  },
		{ "[interface #] sections",     &interface_cfg    },
		{ "[global] section",           &settings_cfg     },
		{ "[core #] sections",          &core_cfg         },
		{ NULL,                         NULL              }
	};

	for (cfg_sections_t *section = config_sections; section->name != NULL; ++section) {
		mprintf("\t*** Reading %s ***\n", section->name);
		cfg_parse(pcfg, section->data);

		if (section->data->error) {
			mprintf("\t\tError at line %u, section [%s], entry %u, \"%s\"\n"
			        , pcfg->err_line, pcfg->err_section, pcfg->err_entry + 1, pcfg->err_str);
			cfg_close(pcfg); /* cannot close before printing error, print uses internal buffer */
			return -1;
		}
	}

	cfg_close(pcfg);

	return check_cfg();
}


static void failed_rte_eal_init(__attribute__((unused))const char *prog_name)
{
	mprintf("\tError in rte_eal_init()\n");
}

int tgen_setup_rte(const char *prog_name)
{
	char *rte_argv[MAX_RTE_ARGV];
	char  rte_arg[MAX_RTE_ARGV][MAX_ARG_LEN];

	/* create mask of used cores */
	mprintf("=== Setting up RTE EAL ===\n");
	rte_cfg.core_mask = 0;
	for (uint8_t lcore_id = 0; lcore_id < RTE_MAX_LCORE; ++lcore_id) {
		if (lcore_cfg_init[lcore_id].flags & PCFG_ACTIVE) {
			rte_cfg.core_mask |= (__UINT64_C(1) << lcore_id);
		}
	}
	mprintf("\tWorker threads core mask is 0x%lx\n", rte_cfg.core_mask);
	if (rte_cfg.core_mask & (__UINT64_C(1) << tgen_cfg.master)) {
		mprintf("\tError: master core index is reused for [core %u] section.\n", tgen_cfg.master);
		return -1;
	}
	if (rte_cfg.core_mask & ((__UINT64_C(1) << tgen_cfg.master) - 1)) {
		mprintf("\tInvalid master core index %u: DPDK would use the lowest index from [core #] sections.\n",
		        tgen_cfg.master);
		return -1;
	}
	rte_cfg.core_mask |= __UINT64_C(1) << tgen_cfg.master;
	mprintf("\tWith master core index %u, full core mask is 0x%lx\n", tgen_cfg.master, rte_cfg.core_mask);

	/* fake command line parameters for rte_eal_init() */
	int argc = 0;
	rte_argv[argc] = strdup(prog_name);
	sprintf(rte_arg[++argc], "-c%lx", rte_cfg.core_mask);
	rte_argv[argc] = rte_arg[argc];

	if (rte_cfg.memory) {
		sprintf(rte_arg[++argc], "-m%u", rte_cfg.memory);
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.force_nchannel) {
		sprintf(rte_arg[++argc], "-n%u", rte_cfg.force_nchannel);
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.force_nrank) {
		sprintf(rte_arg[++argc], "-r%u", rte_cfg.force_nrank);
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.no_hugetlbfs) {
		strcpy(rte_arg[++argc], "--no-huge");
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.no_pci) {
		strcpy(rte_arg[++argc], "--no-pci");
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.no_hpet) {
		strcpy(rte_arg[++argc], "--no-hpet");
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.no_shconf) {
		strcpy(rte_arg[++argc], "--no-shconf");
		rte_argv[argc] = rte_arg[argc];
	}

	if (rte_cfg.eal != NULL) {
		char *ptr = rte_cfg.eal;
		char *ptr2;
		while (ptr != NULL) {
			ptr2 = ptr;
			ptr = strchr(ptr, ' ');
			if (ptr) {
				*ptr++ = '\0';
			}
			strcpy(rte_arg[++argc], ptr2);
			rte_argv[argc] = rte_arg[argc];
		}
	}


	if (rte_cfg.hugedir != NULL) {
		strcpy(rte_arg[++argc], "--huge-dir");
		rte_argv[argc] = rte_arg[argc];
		rte_argv[++argc] = rte_cfg.hugedir;
	}

	if (rte_cfg.no_output) {
		rte_set_log_level(0);
	}
	/* init EAL */
	mprintf("\tEAL command line:");
	if (argc >= MAX_RTE_ARGV) {
		mprintf("too many arguments for EAL\n");
		return -1;
	}

	for (int h = 0; h <= argc; ++h) {
		mprintf(" %s", rte_argv[h]);
	}
	mprintf("\n");

	rte_set_application_usage_hook(failed_rte_eal_init);
	if (rte_eal_init(++argc, rte_argv) < 0) {
		mprintf("\tError in rte_eal_init()\n");
		return -1;
	}

	return 0;
}
