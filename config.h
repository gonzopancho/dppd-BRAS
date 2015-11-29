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

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <rte_ethdev.h>

#define MAX_PKT_BURST   64
#define MAX_RING_BURST	64
#define RING_RX_SIZE    256

#if MAX_RING_BURST < MAX_PKT_BURST
#error MAX_RING_BURST < MAX_PKT_BURST
#endif

#define NUM_VCPES               65536
#define GRE_BUCKET_ENTRIES      4
#define MAX_GRE                 (NUM_VCPES * GRE_BUCKET_ENTRIES)

#define ETHERNET_CRC_LEN 4
#define ETHERNET_MIN_LEN 60
#define ETHERNET_MAX_LEN 1528

#define MBUF_SIZE (ETHERNET_MAX_LEN + (unsigned)sizeof(struct rte_mbuf) + RTE_PKTMBUF_HEADROOM)

#define NB_RX_RING_DESC 256
#define NB_TX_RING_DESC 256

static const struct rte_eth_conf port_conf = {
	.rxmode = {
		.split_hdr_size = 0,
		.header_split   = 0, /* Header Split disabled */
		.hw_ip_checksum = 0, /* IP checksum offload disabled */
		.hw_vlan_filter = 0, /* VLAN filtering disabled */
		.jumbo_frame    = 0, /* Jumbo frame support disabled */
		.hw_strip_crc   = 1, /* CRC stripped by hardware --- always set to 1 in VF */
		.hw_vlan_extend = 0,
	},
	.rx_adv_conf = {
		.rss_conf = {
			.rss_key = NULL,
			//.rss_hf = ETH_RSS_IPV4,
		},
	},
};

static const struct rte_eth_rxconf rx_conf = {
	.rx_free_thresh = 64,
};

static const struct rte_eth_txconf tx_conf = {
	.tx_thresh = {
		.pthresh = 32,
		.hthresh = 8,
		.wthresh = 0,
	},
	.tx_free_thresh = 0, /* Use PMD default values */
	.tx_rs_thresh = 32, /* Use PMD default values */
};

#endif /* _CONFIG_H_ */
