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

#ifndef _HANDLE_QINQ_DECAPV4_H_
#define _HANDLE_QINQ_DECAPV4_H_

#include "task_base.h"

struct lcore_cfg;

struct task_qinq_decap_v4 {
	struct task_base        base;
	void (*tx_pkt)(struct task_base *ptask);
	struct rte_hash         *cpe_table;
	struct rte_hash         *qinq_gre;
	struct lcore_cfg        *lconf;
	struct next_hop_struct  *next_hop;
	struct rte_lpm          *ipv4_lpm;
	uint8_t                 runtime_flags;
};

/* Convert IPv4 packets to GRE and optionally store QinQ Tags */
void handle_qinq_decap_v4_bulk(struct rte_mbuf **rx_mbuf, struct task_base *ptask, uint16_t n_pkts);


#endif /* _HANDLE_QINQ_DECAPV4_H_ */
