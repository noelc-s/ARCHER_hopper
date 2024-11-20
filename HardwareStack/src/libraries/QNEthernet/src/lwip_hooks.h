// SPDX-FileCopyrightText: (c) 2022-2023 Shawn Silverman <shawn@pobox.com>
// SPDX-License-Identifier: AGPL-3.0-or-later

// lwip_hooks.h contains lwIP hook declarations.
// This file is part of the QNEthernet library.

#ifndef QNETHERNET_LWIPHOOKS_H_
#define QNETHERNET_LWIPHOOKS_H_

#include "lwip/err.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"

#if QNETHERNET_ENABLE_RAW_FRAME_SUPPORT

#define LWIP_HOOK_UNKNOWN_ETH_PROTOCOL(p, netif) \
  unknown_eth_protocol((p), (netif))

err_t unknown_eth_protocol(struct pbuf *p, struct netif *netif);

#endif  // QNETHERNET_ENABLE_RAW_FRAME_SUPPORT

#endif  // QNETHERNET_LWIPHOOKS_H_
