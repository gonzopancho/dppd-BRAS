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

#ifndef _DEFINES_H_
#define _DEFINES_H_

// with 3GHz CPU
#define DRAIN_TIMEOUT  __UINT64_C(6000000)             // drain TX buffer every 2ms
#define TERM_TIMEOUT   __UINT64_C(3000000000)          // check if terminated every 1s
#define ARP_TIMEOUT    __UINT64_C(4500000000000)       // 1500 sec

#define MAX_TSC	       __UINT64_C(0xFFFFFFFFFFFFFFFF)

// Implementation supposes that DRAIN_TIMEOUT is smaller than ARP_TIMEOUT and TERM_TIMEOUT
// As it check only ARP_TIMEOUT and TERM_TIMEOUT only when DRAIN_TIMEOUT expired
#if ARP_TIMEOUT < DRAIN_TIMEOUT
#error ARP_TIMEOUT < DRAIN_TIMEOUT
#endif
#if TERM_TIMEOUT < DRAIN_TIMEOUT
#error TERM_TIMEOUT < DRAIN_TIMEOUT
#endif

#ifndef NULL_SIGNATURE
#define NULL_SIGNATURE		0
#endif

#ifndef IPv6_BYTES
#define IPv6_BYTES_FMT	"%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x"
#define IPv6_BYTES(addr)			\
	addr[0],  addr[1],  addr[2],  addr[3],	\
	addr[4],  addr[5],  addr[6],  addr[7],	\
	addr[8],  addr[9],  addr[10], addr[11],	\
	addr[12], addr[13], addr[14], addr[15]
#endif

#ifndef MAC_BYTES
#define MAC_BYTES_FMT "%02x:%02x:%02x:%02x:%02x:%02x"

#define MAC_BYTES(addr)   \
	addr[0], addr[1], \
	addr[2], addr[3], \
	addr[4], addr[5]
#endif

#ifdef BRAS_PREFETCH_OFFSET
#define PREFETCH0(p)		rte_prefetch0(p)
#define PREFETCH_OFFSET		BRAS_PREFETCH_OFFSET
#else
#define PREFETCH0(p)		do {} while (0)
#define PREFETCH_OFFSET		0
#endif

/* assume cpu byte order is little endian */
#define PKT_TO_LUTQINQ(svlan, cvlan) ((svlan & 0x000F) << 4 | (svlan & 0xFF00) << 8 | (cvlan & 0xFF0F))

#define ROUTE_ERR 254

/* Ten gigabit expressed in bytes */
#define TEN_GIGABIT 1250000000

/* can be disabled for testing purposes to not encapsulate in MPLS after routing */
#define MPLS_ROUTING

#endif /* _DEFINES_H_ */
