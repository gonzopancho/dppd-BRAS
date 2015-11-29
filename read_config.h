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

#ifndef _READ_CONFIG_H_
#define _READ_CONFIG_H_

#include <rte_lpm.h>

struct rte_lpm *read_lpm_v4_config(const char *config_path, const char *file_name, uint8_t socket);

void read_lpm_v6_config(const char *config_path, const char *file_name, uint8_t socket);

struct next_hop_struct *read_next_hop_config(const char *config_path, const char *file_name, uint32_t *used_port_mask, uint8_t socket);

/* Need to pass wt_table. This is also filled in while the configuration is loaded. */
struct rte_hash *read_gre_table_config(const char *config_path, const char *file_name, uint8_t *wt_table, uint8_t nb_worker_threads, uint8_t socket);

/* If qinq_to_gre_lookup is a NULL pointer, the hash lookup will be created. */
uint16_t *read_user_table_config(const char *config_path, const char *file_name, struct rte_hash **gre_hash, uint8_t socket);

uint8_t get_nb_hop_ports(void);

uint8_t get_hop_port(uint8_t idx);

#endif /* _READ_CONFIG_H_ */
