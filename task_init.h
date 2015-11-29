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

#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include <rte_common.h>
#include <rte_sched.h>
#include "config.h"
#include "task_base.h"

struct rte_mbuf;
struct lcore_cfg;

#define TGEN_MAX_PORTS     16
#define MAX_PROTOCOLS      3
#define MAX_WT_PER_LB      12
#define MAX_RINGS_PER_CORE (MAX_WT_PER_LB*MAX_PROTOCOLS)

#if MAX_RINGS_PER_CORE < TGEN_MAX_PORTS
#error MAX_RINGS_PER_CORE < TGEN_MAX_PORTS
#endif

#define PORT_STARTUP_NO_DROP        0x01
#define PORT_STARTUP_CPEv4          0x04
#define PORT_STARTUP_CPEv6          0x08
#define PORT_STARTUP_RX_RING        0x10
#define PORT_STARTUP_NETWORK_SIDE   0x20

enum protocols {IPV4, ARP, IPV6};

struct qos_cfg {
	struct rte_sched_port_params port_params;
	struct rte_sched_subport_params subport_params[1];
	struct rte_sched_pipe_params pipe_params[1];
};

struct thread_list_cfg {
	uint32_t    thread_id[MAX_WT_PER_LB];
	uint8_t     active;
	uint8_t     nb_threads;
	uint8_t     dest_task;
};

enum port_mode {NOT_SET, NONE, FWD, ROUTING, UNMPLS,
                QINQ_DECAP_V4, QINQ_DECAP_V6, QINQ_DECAP_ARP,
                LB_QINQ, LB_NETWORK, QINQ_ENCAP_V4, QINQ_ENCAP_V4_UNMPLS,
                QINQ_ENCAP_V6, QINQ_ENCAP_V6_UNMPLS, QOS, CLASSIFY, COUNT_DROP
               };

struct port_ring_setup {
	struct rte_ring *rx_rings[MAX_RINGS_PER_CORE];
	struct rte_ring *tx_rings[MAX_RINGS_PER_CORE];
	uint8_t nb_rxrings;
	uint8_t nb_txrings;
	uint8_t nb_txports;
};

/* Configuration for port that is only used at startup. */
struct task_startup_cfg {
	struct rte_mempool     *pool;
	struct lcore_cfg       *lconf;
	uint32_t               nb_mbuf;
	uint8_t                nb_slave_threads;
	uint8_t		       nb_worker_threads;
	uint8_t		       task;
	struct thread_list_cfg thread_list[MAX_PROTOCOLS];
	uint32_t               ring_size; /* default is RX_RING_SIZE */
	struct qos_cfg         qos_conf;
	uint8_t                flags;
	uint8_t                runtime_flags;
	uint8_t                nb_txports;
	uint8_t                nb_txrings;
	uint8_t                nb_rxrings;
	uint8_t                rx_port;
	uint8_t                *worker_thread_table;
	uint16_t               *user_table;
	struct rte_ring        *rx_rings[MAX_RINGS_PER_CORE];
	struct rte_ring        *tx_rings[MAX_RINGS_PER_CORE];
	struct ether_addr      edaddr;
	struct tx_port_queue   tx_port_queue[TGEN_MAX_PORTS];
	uint8_t                rx_queue;
	/* Used to set up actual port at initialization time. */
	enum port_mode         mode;
	/* Destination output position in hw or sw when using mac learned dest port. */
	uint8_t                mapping[TGEN_MAX_PORTS];
	struct rte_hash        *cpe_table;
	struct rte_hash        *qinq_gre;
	struct rte_sched_port  *sched_port;
	struct next_hop_struct *next_hop;
	struct rte_lpm         *ipv4_lpm;
};

struct task_base *init_task_struct(struct task_startup_cfg *startup_cfg);

#endif /* _TASK_INIT_H_ */
