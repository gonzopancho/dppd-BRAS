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

#include "tx_worker.h"
#include "display.h"
#include "thread_none.h"
#include "thread_lb.h"
#include "thread_qos.h"
#include "thread_bras.h"
#include "thread_simple.h"
#include "tgen_args.h"

/* Main working thread function */
int tgen_work_thread(__attribute__((unused)) void *dummy)
{
	uint32_t lcore_id = rte_lcore_id();
	struct lcore_cfg *lconf = &lcore_cfg[lcore_id];
	mprintf("Entering main loop on core %u\n", lcore_id);

	if (lconf->flags & PCFG_DROP) {
		return thread_simple(lconf);
	}

	if (lconf->flags & PCFG_FWD) { /* bridge Rx from ingress to Tx to egress ports */
		return thread_bras(lconf);
	}

	if (lconf->flags & PCFG_BRAS) { /* bridge Rx from ingress to Tx to egress ports, receiving Q-in-Q */
		return thread_bras(lconf);
	}

	if (lconf->flags & PCFG_NONE) {
		return thread_none(lconf);
	}

	if (lconf->flags & PCFG_CLASSIFY) {
		return thread_simple(lconf);
	}

	if (lconf->flags & PCFG_LB) {
		return thread_lb(lconf);
	}

	if (lconf->flags & PCFG_QOS) { /* special case when running qos, use optimized version */
		return thread_qos(lconf);
	}

	return 0;
}
