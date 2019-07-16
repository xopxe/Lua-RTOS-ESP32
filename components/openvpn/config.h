/*
 * Copyright (C) 2018, IBEROXARXA SERVICIOS INTEGRALES, S.L.
 * Copyright (C) 2018, Jaume Olivé Petrus (jolive@whitecatboard.org)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Lua RTOS OpenVPN configuration file
 *
 */

#ifndef COMPONENTS_OPENVPN_CONFIG_H_
#define COMPONENTS_OPENVPN_CONFIG_H_

#include <string.h>

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <compat/include/linux/in6.h>

#include "tcpip_adapter.h"

// OpenVPN is executed into a task, so when calling to exit function
// simply delete current task
#define exit(e) \
    { \
      extern u8_t volatile _openvpn_running; \
      _openvpn_running = 0; \
      vTaskDelete(NULL); \
    }

// srandom is missing in esp-idf, so use srand instead
#define srandom(s) srand(s)

// openvpn uses small letters but lwip uses capitals
#define lwip_socket_offset LWIP_SOCKET_OFFSET

#define PACKAGE_NAME "openvpn"
#define PACKAGE "openvpn"
#define PACKAGE_VERSION "1"
#define PACKAGE_STRING "1"
#define TARGET_ALIAS ""

#define HAVE_INTTYPES_H 1
#define HAVE_GETTIMEOFDAY 1
#define HAVE_SYS_SOCKET_H 1
#define HAVE_INET_NTOP 1
#define HAVE_INET_PTON 1
#define HAVE_UNISTD_H 1
#define HAVE_STDLIB_H 1
#define HAVE_SYS_TYPES_H 1
#define HAVE_SYS_STAT_H 1
#define HAVE_FCNTL_H 1
#define HAVE_ERRNO_H 1
#define HAVE_SA_FAMILY_T 1
#define HAVE_STDARG_H 1
#define HAVE_STRING_H 1
#define HAVE_NETDB_H 1
#define HAVE_CTYPE_H 1
#define HAVE_SIGNAL_H 1
#define HAVE_CPP_VARARG_MACRO_ISO 1
#define HAVE_SETSOCKOPT 1
#define HAVE_GETSOCKNAME 1
#define HAVE_NET_IF_UTUN_H 0

#define PATH_SEPARATOR_STR "/"
#define EMPTY_ARRAY_SIZE 0

#define ENABLE_SMALL
#define ENABLE_CLIENT_ONLY
#define ENABLE_CRYPTO_MBEDTLS

#endif
