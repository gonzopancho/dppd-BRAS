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

#ifdef BRAS_CMD_TCPDUMP

#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_mbuf.h>

#include "debug.h"
#include "display.h"

#define DUMP_PKT_LEN 75

void DEBUG_TCPDUMP_PACKET(struct task_debug *ptask_debug, const char *msg, struct rte_mbuf *rx_mbuf)
{
	DEBUG_TCPDUMP_PACKET_BULK(ptask_debug, msg, &rx_mbuf, 1);
}

void DEBUG_TCPDUMP_PACKET_BULK(struct task_debug *ptask_debug, const char *msg, struct rte_mbuf **rx_mbuf, uint16_t n_pkts)
{
	for (uint16_t i = 0; i < n_pkts; ++i) {
		if (rte_atomic32_read(&ptask_debug->nb_print)) {
			mprintf("dump tcp packet\n");
			rte_atomic32_dec(&ptask_debug->nb_print);

			const struct ether_hdr *peth =
			        rte_pktmbuf_mtod(rx_mbuf[i],
			                         const struct ether_hdr *);
			const struct ipv4_hdr *dpip =
			        (const struct ipv4_hdr *)(peth + 1);
			const uint8_t *pkt_bytes = (const uint8_t *)peth;
			const uint16_t len = rte_pktmbuf_pkt_len(rx_mbuf[i]);

			mprintf("Core %u Task %u %s(%u)", ptask_debug->core_id,
			        ptask_debug->task_id, msg, len);

			for (uint16_t i = 0; i < len && i < DUMP_PKT_LEN; ++i) {
				mprintf(" %02x", pkt_bytes[i]);
			}

			mprintf("Eth=%x, Proto=%#06x\n", peth->ether_type,
			        dpip->next_proto_id);
		}
	}
}

void DEBUG_PRINT_PACKET(struct task_debug *ptask_debug, struct rte_mbuf *rx_mbuf)
{
	rte_atomic32_set(&ptask_debug->nb_print, 1);
	DEBUG_TCPDUMP_PACKET_BULK(ptask_debug, "", &rx_mbuf, 1);
}

#endif
