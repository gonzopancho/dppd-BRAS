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

#include <stdio.h>
#include <rte_common.h>
#include <rte_malloc.h>
#include <rte_string_fns.h>
#include <rte_hash.h>

#include "read_config.h"
#include "display.h"
#include "quit.h"
#include "route.h"
#include "hash_entry_types.h"
#include "tgen_args.h"
#include "parse_utils.h"
#include "defines.h"

#define MAX_HOP_INDEX	128

static uint8_t hop_ports[TGEN_MAX_PORTS];
static uint8_t nb_hop_ports;

static struct rte_hash_parameters hash_params = {
	.name = "hash_gre",
	.entries = MAX_GRE,
	.bucket_entries = GRE_BUCKET_ENTRIES,
	.key_len = sizeof(uint32_t),
	.entry_len = sizeof(uint32_t) + sizeof(struct qinq_to_gre_table), /* key + data */
	.hash_func_init_val = 0,
	.socket_id = 0 // changed to correct socket when initalizing
};

/* wrapper around fgets, skips comment lines (lines starting with #) */
static char *read_cfg_line(char *str, size_t len, FILE *file)
{
	char *ret;
	do {
		ret = fgets(str, len, file);

		if (ret == NULL) {
			return NULL;
		}
	}
	while (*str == '#');

	return str;
}

uint8_t get_nb_hop_ports(void)
{
	return nb_hop_ports;
}

uint8_t get_hop_port(uint8_t idx)
{
	return hop_ports[idx];
}

struct rte_lpm *read_lpm_v4_config(const char *config_path, const char *file_name, uint8_t socket)
{
	FILE *fp = NULL;
	struct rte_lpm *new_lpm;
	char str[255];
	char name[64];
	char path[1024];
	snprintf(path, 1024, "%s/%s", config_path, file_name);
	mprintf("Reading IPv4 configuration from %s\n", path);
	fp = fopen(path, "r");
	TGEN_PANIC(NULL == fp, "Could not open IPv4 config file %s\n", path);

	snprintf(name, sizeof(name), "IPv4_lpm_s%u", socket);
	new_lpm = rte_lpm_create(name, socket, 16 * 65536 * 32, RTE_LPM_HEAP);
	TGEN_PANIC(new_lpm == NULL, "Failed to configure LPM\n");

	int nroutes = 0;
	char *tokens[4];
	while (read_cfg_line(str, 255, fp) != NULL) {

		uint8_t ret = rte_strsplit(str, 255, tokens, 4, ',');
		TGEN_PANIC(ret != 3, "Expecting format: IPv4 address, depth, next_hop_index\n");

		uint32_t route;
		uint8_t depth;
		uint32_t next_hop_index;
		TGEN_PANIC(0 != get_ip_addr(&route, tokens[0]), "Error parsing IPv4 address\n");
		depth = atoi(tokens[1]);
		next_hop_index = atoi(tokens[2]);

		int rc = rte_lpm_add(new_lpm, route, depth, next_hop_index);
		if (rc == 0) {
			if ((++nroutes % 10000) == 0) {
				mprintf("Route %d added\n", nroutes);
			}
		}
		else {
			mprintf("Failed to add (%d) index %u ip %x depth %u in lpm\n", rc, next_hop_index, route, depth);
		}
	}
	mprintf("\tConfigured %d routes\n", nroutes);
	fclose(fp);
	fp = NULL;
	return new_lpm;
}

void read_lpm_v6_config(const char *config_path, const char *file_name, uint8_t socket)
{
	FILE *fp = NULL;
	char path[1024];
	char str[255];
	snprintf(path, 1024, "%s/%s", config_path, file_name);
	mprintf("Reading IPv6 configuration from %s\n", path);
	fp = fopen(path, "r");
	TGEN_PANIC(fp == NULL, "Could not open IPv6 config file %s\n", path);

	char *tokens[7];
	uint8_t offset = 0;
	while (read_cfg_line(str, 255, fp) != NULL) {
		uint8_t ret = rte_strsplit(str, 255, tokens, 7, ',');
		TGEN_PANIC(ret != 5 && ret != 6, "Expecting format: IPv6 address, Depth, [out_if, ] next hop IPv6 address, next hop MAC address, Next hop MPLS\n");
		strip_spaces(tokens, ret);

		struct in6_addr route;
		uint8_t depth;
		__attribute__((unused)) uint8_t out_if;
		__attribute__((unused)) struct in6_addr next_hop_addr;
		struct ether_addr mac;
		uint32_t mpls;

		TGEN_PANIC(0 != get_ip6_addr(&route, tokens[0]), "Error in routing IPv6 address\n");
		depth = atoi(tokens[1]);

		if (ret == 6) {
			out_if = atoi(tokens[2]);
			offset = 1;
		}

		TGEN_PANIC(0 != get_ip6_addr(&next_hop_addr, tokens[2 + offset]), "Error in next hop IPv6 address\n");
		TGEN_PANIC(0 != get_ether_addr(&mac, tokens[3 + offset]), "Error in next hop MAC address\n");

		mpls = atoi(tokens[4 + offset]);

		add_ipv6_route(&route, depth, &mpls, &mac, socket);
	}
	fclose(fp);
	fp = NULL;
}

struct next_hop_struct *read_next_hop_config(const char *config_path, const char *file_name, uint32_t *used_port_mask, uint8_t socket)
{
	FILE *fp = NULL;
	char str[255];
	char path[1024];
	struct next_hop_struct *next_hop_init = 0;
	next_hop_init = rte_zmalloc_socket(NULL, sizeof(struct next_hop_struct) * MAX_HOP_INDEX, CACHE_LINE_SIZE, socket);
	TGEN_PANIC(next_hop_init == NULL, "Could not allocate memory for next hop\n");
	snprintf(path, 1024, "%s/%s", config_path, file_name);
	mprintf("Reading next hop configuration from %s\n", path);
	fp = fopen(path, "r");
	TGEN_PANIC(fp == NULL, "Could not open next hop config file %s\n", path);

	// hop index, port, IP address, MAC address, MPLS
	char *tokens[6];
	while (read_cfg_line(str, 255, fp) != NULL) {
		uint8_t ret = rte_strsplit(str, 255, tokens, 6, ',');
		TGEN_PANIC(ret != 5, "Expecting format: hop index, port, IP address, MAC address, MPLS\n");
		strip_spaces(tokens, 5);

		uint32_t next_hop_index;
		next_hop_index = atoi(tokens[0]);
		TGEN_PANIC(next_hop_index > MAX_HOP_INDEX - 1, "Invalid hop index\n");

		uint8_t if_port = name_to_if(tokens[1]);

		TGEN_PANIC(if_port == NO_PORT_AVAIL, "Can't find port with name %s\n", tokens[1]);

		uint8_t k;
		for (k = 0; k < nb_hop_ports && k < TGEN_MAX_PORTS; ++k) {
			if (hop_ports[k] == if_port) {
				break;
			}
		}

		TGEN_PANIC(k == TGEN_MAX_PORTS, "To many ports defined inside next_hop, maximum allowed is %u\n", TGEN_MAX_PORTS);
		if (k == nb_hop_ports) {
			hop_ports[k] = if_port;
			++nb_hop_ports;
		}

		next_hop_init[next_hop_index].mac_port.port = k;
		TGEN_PANIC(next_hop_init[next_hop_index].mac_port.port >= TGEN_MAX_PORTS, "Invalid port in %s, maximum port allowed is %d\n", path, TGEN_MAX_PORTS);

		*used_port_mask |= (1U << next_hop_init[next_hop_index].mac_port.port);
		TGEN_PANIC(0 != get_ip_addr(&next_hop_init[next_hop_index].ip_dst, tokens[2]), "Invalid IP for next hop\n");
		TGEN_PANIC(0 != get_ether_addr(&next_hop_init[next_hop_index].mac_port.mac, tokens[3]), "Invalid MAC for next hop\n");

		next_hop_init[next_hop_index].mpls = strtol(tokens[4], NULL, 16);
	}
	fclose(fp);
	fp = NULL;
	return next_hop_init;
}

struct rte_hash *read_gre_table_config(const char *config_path, const char *file_name, uint8_t *wt_table, uint8_t nb_worker_threads, uint8_t socket)
{
	FILE *fp = NULL;
	char str[255];
	char path[1024];
	snprintf(path, 1024, "%s/%s", config_path, file_name);
	mprintf("Reading GRE configuration from file %s\n", path);
	fp = fopen(path, "r");
	TGEN_PANIC(NULL == fp, "Could not open GRE table %s\n", path);

	hash_params.socket_id = socket;
	struct rte_hash *gre_hash = rte_hash_ext_create(&hash_params);
	TGEN_PANIC(gre_hash == NULL, "Error creating qinq to gre hash lookup");

	char *tokens[3];
	while (read_cfg_line(str, 255, fp) != NULL) {
		TGEN_PANIC(2 != rte_strsplit(str, 255, tokens, 3, ','), "Expecting format: gre_id, qinq\n");
		strip_spaces(tokens, 2);
		uint32_t gre_id = atoi(tokens[0]);
		uint32_t qinq = atoi(tokens[1]) & 0xFFFFFF;

		uint16_t svlan = rte_cpu_to_be_16((qinq & 0xFFF000) >> 12);
		uint16_t cvlan = rte_cpu_to_be_16(qinq & 0xFFF);
		wt_table[PKT_TO_LUTQINQ(svlan, cvlan)] = gre_id % nb_worker_threads;
		uint32_t qinq2 = rte_cpu_to_be_16((qinq & 0xFFF000) >> 12) << 16 | rte_cpu_to_be_16(qinq & 0x000FFF);
		struct qinq_to_gre_table_hash_entry entry;
		entry.key = qinq2;
		entry.data.gre_id = gre_id;
		struct rte_hash_ext ext;
		int32_t hash_index = rte_hash_ext_add_entry(gre_hash, &entry, &ext);
		TGEN_PANIC((hash_index >= MAX_GRE) || (hash_index < 0), "Error adding key in qinq to gre hash: index = %d\n", hash_index);

	}
	fclose(fp);
	return gre_hash;
}

uint16_t *read_user_table_config(const char *config_path, const char *file_name, struct rte_hash **gre_hash, uint8_t socket)
{
	FILE *fp = NULL;
	char str[255];
	uint8_t created_hash = 0;
	uint16_t *user_table_init = rte_zmalloc_socket(NULL, 0x1000000 * sizeof(uint16_t), CACHE_LINE_SIZE, socket);
	TGEN_PANIC(user_table_init == NULL, "Error creating worker thread table");
	char path[1024];
	snprintf(path, 1024, "%s/%s", config_path, file_name);
	mprintf("Reading user table configuration from file %s\n", path);
	fp = fopen(path, "r");
	TGEN_PANIC(fp == NULL, "Could not open user table %s\n", path);


	if (*gre_hash == NULL) {
		hash_params.socket_id = socket;
		*gre_hash = rte_hash_ext_create(&hash_params);
		TGEN_PANIC(*gre_hash == NULL, "Error creating qinq to gre hash lookup");
		created_hash = 1;
	}


	char *tokens[3];
	while (read_cfg_line(str, 255, fp) != NULL) {
		TGEN_PANIC(2 != rte_strsplit(str, 255, tokens, 3, ','), "Expecting format: qinq, user\n");
		strip_spaces(tokens, 2);
		uint32_t qinq = atoi(tokens[0]) & 0xFFFFFF;
		uint32_t user = atoi(tokens[1]);

		int32_t hash_index;
		uint32_t qinq2 =  rte_cpu_to_be_16((qinq & 0xFFF000) >> 12) << 16 | rte_cpu_to_be_16(qinq & 0x000FFF);
		/* if the hash table has been created, we can reuse the same
		   entries and add the extra QoS related user lookup data */
		if (created_hash == 0) {
			/* use existing entry */
			struct rte_hash_ext ext;
			hash_index = rte_hash_ext_lookup(*gre_hash, (const void *)&qinq2, &ext);
			TGEN_PANIC((hash_index >= MAX_GRE) || (hash_index < 0), "Error retrieving key in qinq_to_gre_lookup: index = %d\n", hash_index);
			struct qinq_to_gre_table_hash_entry *existing = ext.bucket_entry;

			existing->data.user = user;
		}
		else {
			/* if we needed to create the table, we have to add new entries here
			   specifically to allow mapping qinq packets to users */
			struct qinq_to_gre_table_hash_entry entry;
			entry.key         = qinq;
			entry.data.user   = user;
			struct rte_hash_ext ext;
			hash_index = rte_hash_ext_add_entry(*gre_hash, &entry, &ext);
			TGEN_PANIC((hash_index >= MAX_GRE) || (hash_index < 0), "Error adding key in qinq_to_gre_lookup: index = %d\n", hash_index);
		}

		uint16_t svlan = rte_cpu_to_be_16((qinq & 0xFFF000) >> 12);
		uint16_t cvlan = rte_cpu_to_be_16(qinq & 0xFFF);
		user_table_init[PKT_TO_LUTQINQ(svlan, cvlan)] = user;
	}
	fclose(fp);

	return user_table_init;
}
