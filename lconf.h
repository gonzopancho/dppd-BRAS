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

#ifndef _LCONF_H_
#define _LCONF_H_

#include "task_init.h"

#define MAX_NAME_SIZE	24
#define MAX_TASKS_PER_CORE	4
#define MAX_SOCKETS		1
struct task_base;
struct rte_hash;

struct lcore_cfg {
	struct task_base	*task[MAX_TASKS_PER_CORE];
	uint8_t			nb_tasks;		// Used by ALL

#ifdef BRAS_CMD_STAT_RX
	rte_atomic64_t		nb_rx_buckets[MAX_RING_BURST + 1];
#endif

	struct rte_hash		*cpe_v4_table;		// QinQ, GRE
	struct rte_hash		*cpe_v6_table;		// QinQ, GRE

	uint32_t		bucket_index;		// QinQ, GRE
	uint32_t		bucket_index6;		// QinQ, GRE
	// Following variables are not accessed in main loop
	uint32_t		bras_ip;
	uint32_t		flags;			// PCFG_* flags below
	uint32_t		n_streams;
	uint8_t			active_task;
	uint8_t			corenb;
	char			name[MAX_NAME_SIZE];
	struct task_startup_cfg startup_cfg[MAX_TASKS_PER_CORE];
} __rte_cache_aligned;


/* flags for lcore_cfg */
#define PCFG_NONE		0x00000001
#define PCFG_FWD		0x00000002
#define PCFG_QOS		0x00000004
#define PCFG_CLASSIFY		0x00000008
#define PCFG_BRAS		0x00000020
#define PCFG_CPETABLEv4		0x00000040
#define PCFG_CPETABLEv6		0x00000080
#define PCFG_DROP		0x00000100
#define PCFG_LB			0x00000200

/* mask for core mode */
#define PCFG_MODE		(PCFG_FWD | PCFG_BRAS | PCFG_DROP | PCFG_QOS | PCFG_LB | PCFG_NONE | PCFG_CLASSIFY)


#define PCFG_TERMINATE		0x40000000 /* thread terminate flag */
#define PCFG_ACTIVE		0x80000000 /* configuration is active */


#endif /* _LCONF_H_ */
