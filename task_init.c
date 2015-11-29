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

#include <rte_malloc.h>
#include "rx_pkt.h"
#include "task_init.h"
#include "tx_pkt.h"
#include "quit.h"

#include "handle_none.h"
#include "handle_count_drop.h"
#include "handle_qos.h"
#include "handle_qinq_decapv4.h"
#include "handle_qinq_decapv6.h"
#include "handle_qinq_decaparp.h"
#include "handle_qinq_encapv4.h"
#include "handle_qinq_encapv6.h"
#include "handle_routing.h"
#include "handle_unmpls.h"
#include "handle_fwd.h"
#include "handle_lb_qinq.h"
#include "handle_lb_network.h"
#include "handle_classify.h"

static size_t get_task_struct_size(enum port_mode mode)
{
	switch (mode) {
	case NONE:
		return sizeof(struct task_none);
	case CLASSIFY:
		return sizeof(struct task_classify);
	case FWD:
		return sizeof(struct task_fwd);
	case ROUTING:
		return sizeof(struct task_routing);
	case UNMPLS:
		return sizeof(struct task_unmpls);
	case QINQ_DECAP_V4:
		return sizeof(struct task_qinq_decap_v4);
	case QINQ_DECAP_V6:
		return sizeof(struct task_qinq_decap_v6);
	case QINQ_DECAP_ARP:
		return sizeof(struct task_qinq_decap_arp);
	case LB_QINQ:
		return sizeof(struct task_lb_qinq);
	case LB_NETWORK:
		return sizeof(struct task_lb_network);
	case QINQ_ENCAP_V4:
		return sizeof(struct task_qinq_encap_v4);
	case QINQ_ENCAP_V4_UNMPLS:
		return sizeof(struct task_qinq_encap_v4);
	case QINQ_ENCAP_V6:
		return sizeof(struct task_qinq_encap_v6);
	case QOS:
		return sizeof(struct task_qos);
	case COUNT_DROP:
		return sizeof(struct task_base);
	default:
		TGEN_PANIC(1, "Invalid port mode, exiting...\n");
	}
}

static size_t calc_memsize(struct task_startup_cfg *startup_cfg)
{
	size_t memsize = sizeof(struct task_base);

	memsize += get_task_struct_size(startup_cfg->mode);

#ifdef BRAS_STATS
	memsize += sizeof(struct stats);
#endif

	if (startup_cfg->nb_rxrings != 0) {
		memsize += sizeof(struct rte_ring *)*startup_cfg->nb_rxrings;
	}

	if (startup_cfg->nb_txrings != 0) {
		memsize += (sizeof(struct mbuf_table_struct) + sizeof(struct rte_ring *)) * startup_cfg->nb_txrings;
	}
	else {
		memsize += (sizeof(struct mbuf_table_struct) + sizeof(struct tx_port_queue)) * startup_cfg->nb_txports;
	}

	return memsize;
}

static size_t init_rx_tx_rings_ports(struct task_startup_cfg *startup_cfg, struct task_base *task_base, size_t offset)
{
	if (startup_cfg->nb_rxrings != 0) {
		task_base->rx_pkt = rx_pkt_sw;
		task_base->rx_params_sw.nb_rxrings = startup_cfg->nb_rxrings;
		task_base->rx_params_sw.rx_rings = (struct rte_ring **)(((uint8_t *)task_base) + offset);
		offset += sizeof(struct rte_ring *)*task_base->rx_params_sw.nb_rxrings;

		for (uint8_t i = 0; i < task_base->rx_params_sw.nb_rxrings; ++i) {
			task_base->rx_params_sw.rx_rings[i] = startup_cfg->rx_rings[i];
		}
	}
	else {
		/* When receiving from a port, only a single port
		   can be used to receive packets. The related values
		   are saved inline. */
		task_base->rx_pkt = rx_pkt_hw;
		task_base->rx_params_hw.rx_port = startup_cfg->rx_port;
		task_base->rx_params_hw.rx_queue = startup_cfg->rx_queue;
	}


	if (startup_cfg->nb_txrings != 0) {
		task_base->tx_params_sw.nb_txrings = startup_cfg->nb_txrings;
		task_base->tx_mbuf = (struct mbuf_table_struct *)(((uint8_t *)task_base) + offset);
		offset += sizeof(struct mbuf_table_struct) * task_base->tx_params_sw.nb_txrings;
		task_base->tx_params_sw.tx_rings = (struct rte_ring **)(((uint8_t *)task_base) + offset);
		offset += sizeof(struct rte_ring *)*task_base->tx_params_sw.nb_txrings;

		for (uint8_t i = 0; i < task_base->tx_params_sw.nb_txrings; ++i) {
			task_base->tx_params_sw.tx_rings[i] = startup_cfg->tx_rings[i];
		}
	}
	else {
		task_base->tx_params_hw.nb_txports = startup_cfg->nb_txports;
		task_base->tx_mbuf = (struct mbuf_table_struct *)(((uint8_t *)task_base) + offset);
		offset += sizeof(struct mbuf_table_struct) * task_base->tx_params_hw.nb_txports;
		task_base->tx_params_hw.tx_port_queue = (struct tx_port_queue *)(((uint8_t *)task_base) + offset);
		offset += sizeof(struct tx_port_queue) * task_base->tx_params_hw.nb_txports;
		for (uint8_t i = 0; i < task_base->tx_params_hw.nb_txports; ++i) {
			task_base->tx_params_hw.tx_port_queue[i].port = startup_cfg->tx_port_queue[i].port;
			task_base->tx_params_hw.tx_port_queue[i].queue = startup_cfg->tx_port_queue[i].queue;
		}
	}
	return offset;
}

static void *tx_function(struct task_startup_cfg *startup_cfg)
{
	if (startup_cfg->flags & PORT_STARTUP_NO_DROP) {
		return startup_cfg->nb_txrings ? tx_pkt_no_drop_sw : tx_pkt_no_drop_hw;
	}
	else {
		return startup_cfg->nb_txrings ? tx_pkt_sw : tx_pkt_hw;
	}
}

static void *flush_function(struct task_startup_cfg *startup_cfg)
{
	if (startup_cfg->flags & PORT_STARTUP_NO_DROP) {
		return startup_cfg->nb_txrings ? flush_queues_no_drop_sw : flush_queues_no_drop_hw;
	}
	else {
		return startup_cfg->nb_txrings ? flush_queues_sw : flush_queues_hw;
	}
}

static void *no_buf_tx_function(struct task_startup_cfg *startup_cfg)
{
	if (startup_cfg->flags & PORT_STARTUP_NO_DROP) {
		return startup_cfg->nb_txrings ? tx_pkt_no_drop_no_buf_sw : tx_pkt_no_drop_no_buf_hw;
	}
	else {
		return startup_cfg->nb_txrings ? tx_pkt_no_buf_sw : tx_pkt_no_buf_hw;
	}
}

static void init_task_none(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk = handle_none_bulk;
	task_base->tx_pkt_no_buf = no_buf_tx_function(startup_cfg);
	task_base->flags |= FLAG_NEVER_FLUSH;

}

static void init_task_count_drop(struct task_base *task_base)
{
	task_base->handle_pkt_bulk = handle_count_drop_bulk;
}

static void init_task_lb_qinq(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk  = handle_lb_qinq_bulk;
	struct task_lb_qinq *task_lb_qinq = (struct task_lb_qinq *)task_base;
	task_lb_qinq->nb_worker_threads = startup_cfg->nb_worker_threads;
	task_lb_qinq->bit_mask = rte_is_power_of_2(startup_cfg->nb_worker_threads) ? startup_cfg->nb_worker_threads - 1 : 0xff;
	task_lb_qinq->worker_thread_table = startup_cfg->worker_thread_table;
	task_lb_qinq->tx_pkt = tx_function(startup_cfg);
	task_base->flush_queues = flush_function(startup_cfg);

}

static void init_task_lb_network(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk = handle_lb_network_bulk;
	struct task_lb_network *task_lb_network = (struct task_lb_network *)task_base;

	task_lb_network->runtime_flags = startup_cfg->runtime_flags;
	task_lb_network->worker_byte_offset_ipv6 = startup_cfg->flags & PORT_STARTUP_NETWORK_SIDE ? 39 : 23;
	task_lb_network->worker_byte_offset_ipv4 = startup_cfg->flags & PORT_STARTUP_NETWORK_SIDE ? 19 : 15;
	task_lb_network->nb_worker_threads       = startup_cfg->nb_worker_threads;
	/* The optimal configuration is when the number of worker threads
	   is a power of 2. In that case, a bit_mask can be used. Setting
	   the bitmask to 0xff disables the "optimal" usage of bitmasks
	   and the actual number of worker threads will be used instead. */
	task_lb_network->bit_mask = rte_is_power_of_2(startup_cfg->nb_worker_threads) ? startup_cfg->nb_worker_threads - 1 : 0xff;
	task_lb_network->tx_pkt = tx_function(startup_cfg);
	task_base->flush_queues = flush_function(startup_cfg);
}

static void init_task_qinq_decap_v4(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk = handle_qinq_decap_v4_bulk;
	struct task_qinq_decap_v4 *task_qinq_decap_v4 = (struct task_qinq_decap_v4 *)task_base;
	task_qinq_decap_v4->cpe_table = startup_cfg->cpe_table;
	task_qinq_decap_v4->lconf = startup_cfg->lconf;
	task_qinq_decap_v4->next_hop = startup_cfg->next_hop;
	task_qinq_decap_v4->ipv4_lpm = startup_cfg->ipv4_lpm;
	task_qinq_decap_v4->runtime_flags = startup_cfg->runtime_flags;
	TGEN_PANIC(startup_cfg->qinq_gre == NULL, "can't set up qinq gre\n");
	task_qinq_decap_v4->qinq_gre = startup_cfg->qinq_gre;
	task_qinq_decap_v4->tx_pkt = tx_function(startup_cfg);
	task_base->flush_queues = flush_function(startup_cfg);
}

static void init_task_qinq_decap_v6(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk = handle_qinq_decap_v6_bulk;
	struct task_qinq_decap_v6 *task_qinq_decap_v6 = (struct task_qinq_decap_v6 *)task_base;
	task_qinq_decap_v6->edaddr = startup_cfg->edaddr;
	task_qinq_decap_v6->cpe_table = startup_cfg->cpe_table;
	task_qinq_decap_v6->user_table = startup_cfg->user_table;
	task_qinq_decap_v6->tx_pkt = tx_function(startup_cfg);
	task_base->flush_queues = flush_function(startup_cfg);
}

static void init_task_qinq_decap_arp(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk = handle_qinq_decap_arp_bulk;
	struct task_qinq_decap_arp *task_qinq_decap_arp = (struct task_qinq_decap_arp *)task_base;
	task_qinq_decap_arp->cpe_table = startup_cfg->cpe_table;
	task_qinq_decap_arp->qinq_gre = startup_cfg->qinq_gre;
	task_base->flush_queues = flush_function(startup_cfg);
}

static void init_task_qos(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	/* when a port is doing QoS, the whole core is only doing QoS.
	   For this reason the HandleQoSBulk function is called directly. */
	task_base->handle_pkt_bulk = NULL;
	struct task_qos *task_qos = (struct task_qos *)task_base;
	task_qos->sched_port = startup_cfg->sched_port;
	task_qos->user_table = startup_cfg->user_table;
	task_qos->runtime_flags = startup_cfg->runtime_flags;
	task_base->tx_pkt_no_buf = no_buf_tx_function(startup_cfg);
	task_base->flags |= FLAG_NEVER_FLUSH;

}

static void init_port_encap_v4(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	if (startup_cfg->mode == QINQ_ENCAP_V4_UNMPLS) {
		task_base->handle_pkt_bulk = handle_qinq_encap_v4_untag_bulk;
	}
	else {
		task_base->handle_pkt_bulk = handle_qinq_encap_v4_bulk;
	}

	struct task_qinq_encap_v4 *task_qinq_encap_v4 = (struct task_qinq_encap_v4 *)(task_base);

	task_qinq_encap_v4->cpe_table = startup_cfg->cpe_table;
	task_qinq_encap_v4->runtime_flags = startup_cfg->runtime_flags;
	task_qinq_encap_v4->tx_pkt = tx_function(startup_cfg);


	for (uint8_t i = 0; i < TGEN_MAX_PORTS; ++i) {
		task_qinq_encap_v4->mapping[i] = startup_cfg->mapping[i];
	}

	task_base->flush_queues = flush_function(startup_cfg);
}

static void init_port_encap_v6(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{

	if (startup_cfg->mode == QINQ_ENCAP_V6_UNMPLS) {
		task_base->handle_pkt_bulk = handle_qinq_encap_v6_untag_bulk;
	}
	else {
		task_base->handle_pkt_bulk = handle_qinq_encap_v6_bulk;
	}

	struct task_qinq_encap_v6 *task_qinq_encap_v6 = (struct task_qinq_encap_v6 *)task_base;

	task_qinq_encap_v6->cpe_table = startup_cfg->cpe_table;
	task_qinq_encap_v6->runtime_flags = startup_cfg->runtime_flags;
	task_qinq_encap_v6->tx_pkt = tx_function(startup_cfg);
	task_base->flush_queues = flush_function(startup_cfg);
}

static void init_task_fwd(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk = handle_fwd_bulk;
	struct task_fwd *task_fwd = (struct task_fwd *)task_base;
	task_fwd->edaddr     = startup_cfg->edaddr;
	task_fwd->runtime_flags = startup_cfg->runtime_flags;
	task_fwd->lconf = startup_cfg->lconf;
	task_fwd->next_hop = startup_cfg->next_hop;
	task_fwd->ipv4_lpm = startup_cfg->ipv4_lpm;
	task_fwd->tx_pkt = tx_function(startup_cfg);
	task_base->flush_queues = flush_function(startup_cfg);
}

static void init_task_routing(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk = handle_routing_bulk;
	struct task_routing *task_routing = (struct task_routing *)task_base;
	task_routing->edaddr     = startup_cfg->edaddr;
	task_routing->lconf = startup_cfg->lconf;
	task_routing->tx_pkt = tx_function(startup_cfg);
	task_routing->next_hop = startup_cfg->next_hop;
	task_routing->ipv4_lpm = startup_cfg->ipv4_lpm;
	task_base->flush_queues = flush_function(startup_cfg);
}

static void init_task_unmpls(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk = handle_unmpls_bulk;
	struct task_unmpls *task_unmpls = (struct task_unmpls *)task_base;
	task_unmpls->edaddr     = startup_cfg->edaddr;
	task_unmpls->tx_pkt = tx_function(startup_cfg);
	task_unmpls->lconf = startup_cfg->lconf;
	task_base->flush_queues = flush_function(startup_cfg);
}

static void init_task_classify(struct task_base *task_base, struct task_startup_cfg *startup_cfg)
{
	task_base->handle_pkt_bulk = handle_classify_bulk;
	task_base->tx_pkt_no_buf = startup_cfg->nb_txrings ? tx_pkt_no_drop_no_buf_sw : tx_pkt_no_drop_no_buf_hw;
	task_base->flags |= FLAG_NEVER_FLUSH;
	struct task_classify *task_classify = (struct task_classify *)task_base;
	task_classify->user_table = startup_cfg->user_table;
}

struct task_base *init_task_struct(struct task_startup_cfg *startup_cfg)
{
	size_t offset = 0;
	size_t memsize = calc_memsize(startup_cfg);
	uint8_t cur_socket = rte_socket_id();
	struct task_base *task_base = rte_zmalloc_socket(NULL, memsize, CACHE_LINE_SIZE, cur_socket);
	offset += get_task_struct_size(startup_cfg->mode);

	offset = init_rx_tx_rings_ports(startup_cfg, task_base, offset);


#ifdef BRAS_STATS
	task_base->stats = (struct stats *)(((uint8_t *)task_base) + offset);
	offset += sizeof(struct stats);
#endif

#ifdef BRAS_CMD_TCPDUMP
	task_base->task_debug.core_id = startup_cfg->lconf->corenb;
	task_base->task_debug.task_id = startup_cfg->task;
#endif

	switch (startup_cfg->mode) {
	case NONE:
		init_task_none(task_base, startup_cfg);
		break;
	case FWD:
		init_task_fwd(task_base, startup_cfg);
		break;
	case ROUTING:
		init_task_routing(task_base, startup_cfg);
		break;
	case UNMPLS:
		init_task_unmpls(task_base, startup_cfg);
		break;
	case QINQ_DECAP_V4:
		init_task_qinq_decap_v4(task_base, startup_cfg);
		break;
	case QINQ_DECAP_V6:
		init_task_qinq_decap_v6(task_base, startup_cfg);
		break;
	case QINQ_DECAP_ARP:
		init_task_qinq_decap_arp(task_base, startup_cfg);
		break;
	case LB_QINQ:
		init_task_lb_qinq(task_base, startup_cfg);
		break;
	case LB_NETWORK:
		init_task_lb_network(task_base, startup_cfg);
		break;
	case QINQ_ENCAP_V4:
	case QINQ_ENCAP_V4_UNMPLS:
		init_port_encap_v4(task_base, startup_cfg);
		break;
	case QINQ_ENCAP_V6:
		init_port_encap_v6(task_base, startup_cfg);
		break;
	case QOS:
		init_task_qos(task_base, startup_cfg);
		break;
	case CLASSIFY:
		init_task_classify(task_base, startup_cfg);
		break;
	case COUNT_DROP:
		init_task_count_drop(task_base);
		break;
	default:
		TGEN_PANIC(1, "Invalid port mode, exiting...\n");
		break;
	}

	return task_base;
}
