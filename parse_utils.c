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

#include <ctype.h>
#include <linux/in6.h>

#include <rte_ether.h>
#include <rte_string_fns.h>

#include "parse_utils.h"
#include "display.h"

int8_t get_ip_addr(uint32_t *addr, char *saddr)
{
	char *ip_parts[4];
	if (4 != rte_strsplit(saddr, strlen(saddr), ip_parts, 4, '.')) {
		return -1;
	}

	uint32_t val;
	for (uint8_t i = 0; i < 4; ++i) {
		val = atoi(ip_parts[i]);
		if (val > 255) {
			return -1;
		}
		*addr = *addr << 8 | val;
	}
	return 0;
}

int8_t get_ip6_addr(struct in6_addr *in6_addr, char *saddr)
{
	char *addr_parts[9];

	uint8_t ret = rte_strsplit(saddr, strlen(saddr), addr_parts, 9, ':');

	if (ret == 9) {
		return -1;
	}

	uint8_t omitted = 0;

	for (uint8_t i = 0, j = 0; i < ret; ++i, ++j) {
		if (*addr_parts[i] == 0) {
			/* Can only be omitted once */
			if (omitted == 0) {
				return -1;
			}
			omitted = 1;
			j += 8 - ret;
		}
		else {
			in6_addr->s6_addr[j] = strtoll(addr_parts[i], NULL, 16);
		}
	}
	return 0;
}

int8_t get_ether_addr(struct ether_addr *ether_addr, char *saddr)
{
	char *addr_parts[7];
	uint8_t ret = rte_strsplit(saddr, strlen(saddr), addr_parts, 7, ':');

	if (ret != 6) {
		return -1;
	}

	for (uint8_t i = 0; i < 6; ++i) {
		if (2 != strlen(addr_parts[i])) {
			return -1;
		}
		ether_addr->addr_bytes[i] = strtol(addr_parts[i], NULL, 16);
	}

	return 0;
}


char* get_cfg_key(char *str)
{
	char *pkey = strchr(str, '=');

	if (pkey == NULL) {
		return NULL;
	}
	*pkey++ = '\0';

	/* remove leading spaces */
	while (isspace(*pkey)) {
		pkey++;
	}
	if (*pkey == '\0') { /* an empty key */
		return NULL;
	}

	return pkey;
}

void strip_spaces(char *strings[], const uint32_t count)
{
	for (uint32_t i = 0; i < count; ++i) {
		while (isspace(strings[i][0])) {
			++strings[i];
		}
		size_t len = strlen(strings[i]);

		while (len && isspace(strings[i][len - 1])) {
			strings[i][len - 1] = '\0';
			--len;
		}
	}
}

int list_set_from_str(uint32_t *list, char *pkey, uint32_t max_list)
{
	unsigned int i, j = 0;
	uint32_t range;
	while (*pkey) {
		uint32_t val = (uint32_t)strtol(pkey, &pkey, 10);
		list[j++] = val;
		if (*pkey == '-') {
			*pkey++ = '\0';
			range = (uint32_t)strtol(pkey, &pkey, 10);
			if (range >= val) {
				for (i = val + 1; i <= range; ++i) {
					if (j == max_list) {
						mprintf("Error range > %u\n", max_list);
						return -1;
					}
					list[j++] = i;
				}
			}
			else {
				mprintf("Error range > val\n");
				return -1;
			}
		}
		if (*pkey == ',') {
			++pkey;
		}
		else {
			break;
		}
	}
	return j;
}
