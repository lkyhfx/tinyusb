/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if (CFG_TUH_ENABLED && CFG_TUH_HUB)

#include "usbh.h"
#include "usbh_classdriver.h"
#include "hub.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
typedef struct
{
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t port_count;
  uint8_t status_change; // data from status change interrupt endpoint

  // Consecutive failures, cleared on success. Bounded so a dead hub cannot
  // keep the host task spinning, while a transient error recovers silently.
  uint8_t status_failures;
  uint8_t control_failures;
  uint8_t reset_failures;

  // A failed status submission cannot be allowed to permanently silence the
  // hub. Retry from the host's cooperative main loop with bounded backoff.
  bool status_armed;
  bool retry_pending;
  bool clear_halt_pending;
  uint32_t last_tick_ms;
  uint32_t retry_due_ms;
  uint32_t recovery_backoff_ms;
  uint8_t pending_reset_port; // 0: no reset pending

  hub_port_status_response_t port_status;
  hub_status_response_t own_status;
} hub_interface_t;

enum { HUB_RETRY_MAX = 3, HUB_RETRY_INITIAL_MS = 200, HUB_RETRY_MAX_MS = 5000 };

static void hub_schedule_recovery(uint8_t dev_addr, bool clear_halt);

CFG_TUSB_MEM_SECTION static hub_interface_t hub_data[CFG_TUH_HUB];
CFG_TUSB_MEM_SECTION TU_ATTR_ALIGNED(4) static uint8_t _hub_buffer[sizeof(descriptor_hub_desc_t)];
static tuh_hub_diagnostics_t _hub_diag;

TU_ATTR_ALWAYS_INLINE
static inline hub_interface_t* get_itf(uint8_t dev_addr)
{
  return &hub_data[dev_addr-1-CFG_TUH_DEVICE_MAX];
}

#if CFG_TUSB_DEBUG >= 2
static char const* const _hub_feature_str[] =
{
  [HUB_FEATURE_PORT_CONNECTION          ] = "PORT_CONNECTION",
  [HUB_FEATURE_PORT_ENABLE              ] = "PORT_ENABLE",
  [HUB_FEATURE_PORT_SUSPEND             ] = "PORT_SUSPEND",
  [HUB_FEATURE_PORT_OVER_CURRENT        ] = "PORT_OVER_CURRENT",
  [HUB_FEATURE_PORT_RESET               ] = "PORT_RESET",
  [HUB_FEATURE_PORT_POWER               ] = "PORT_POWER",
  [HUB_FEATURE_PORT_LOW_SPEED           ] = "PORT_LOW_SPEED",
  [HUB_FEATURE_PORT_CONNECTION_CHANGE   ] = "PORT_CONNECTION_CHANGE",
  [HUB_FEATURE_PORT_ENABLE_CHANGE       ] = "PORT_ENABLE_CHANGE",
  [HUB_FEATURE_PORT_SUSPEND_CHANGE      ] = "PORT_SUSPEND_CHANGE",
  [HUB_FEATURE_PORT_OVER_CURRENT_CHANGE ] = "PORT_OVER_CURRENT_CHANGE",
  [HUB_FEATURE_PORT_RESET_CHANGE        ] = "PORT_RESET_CHANGE",
  [HUB_FEATURE_PORT_TEST                ] = "PORT_TEST",
  [HUB_FEATURE_PORT_INDICATOR           ] = "PORT_INDICATOR",
};
#endif

//--------------------------------------------------------------------+
// HUB
//--------------------------------------------------------------------+
bool hub_port_clear_feature(uint8_t hub_addr, uint8_t hub_port, uint8_t feature,
                            tuh_xfer_cb_t complete_cb, uintptr_t user_data)
{
  tusb_control_request_t const request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_OTHER,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = HUB_REQUEST_CLEAR_FEATURE,
    .wValue   = feature,
    .wIndex   = hub_port,
    .wLength  = 0
  };

  tuh_xfer_t xfer =
  {
    .daddr       = hub_addr,
    .ep_addr     = 0,
    .setup       = &request,
    .buffer      = NULL,
    .complete_cb = complete_cb,
    .user_data   = user_data
  };

  TU_LOG2("HUB Clear Feature: %s, addr = %u port = %u\r\n", _hub_feature_str[feature], hub_addr, hub_port);
  TU_ASSERT( tuh_control_xfer(&xfer) );
  return true;
}

bool hub_port_set_feature(uint8_t hub_addr, uint8_t hub_port, uint8_t feature,
                          tuh_xfer_cb_t complete_cb, uintptr_t user_data)
{
  tusb_control_request_t const request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_OTHER,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = HUB_REQUEST_SET_FEATURE,
    .wValue   = feature,
    .wIndex   = hub_port,
    .wLength  = 0
  };

  tuh_xfer_t xfer =
  {
    .daddr       = hub_addr,
    .ep_addr     = 0,
    .setup       = &request,
    .buffer      = NULL,
    .complete_cb = complete_cb,
    .user_data   = user_data
  };

  TU_LOG2("HUB Set Feature: %s, addr = %u port = %u\r\n", _hub_feature_str[feature], hub_addr, hub_port);
  TU_ASSERT( tuh_control_xfer(&xfer) );
  return true;
}

bool hub_port_get_status(uint8_t hub_addr, uint8_t hub_port, void* resp,
                         tuh_xfer_cb_t complete_cb, uintptr_t user_data)
{
  tusb_control_request_t const request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_OTHER,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_IN
    },
    .bRequest = HUB_REQUEST_GET_STATUS,
    .wValue   = 0,
    .wIndex   = hub_port,
    .wLength  = 4
  };

  tuh_xfer_t xfer =
  {
    .daddr       = hub_addr,
    .ep_addr     = 0,
    .setup       = &request,
    .buffer      = resp,
    .complete_cb = complete_cb,
    .user_data   = user_data
  };

  TU_LOG2("HUB Get Port Status: addr = %u port = %u\r\n", hub_addr, hub_port);
  TU_ASSERT( tuh_control_xfer(&xfer) );
  return true;
}

//--------------------------------------------------------------------+
// CLASS-USBH API (don't require to verify parameters)
//--------------------------------------------------------------------+
void hub_init(void)
{
  tu_memclr(hub_data, sizeof(hub_data));
  tu_memclr(&_hub_diag, sizeof(_hub_diag));
}

void tuh_hub_get_diagnostics(tuh_hub_diagnostics_t* diag)
{
  if (diag) *diag = _hub_diag;
}

bool hub_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
  (void) rhport;

  TU_VERIFY(TUSB_CLASS_HUB == itf_desc->bInterfaceClass &&
            0              == itf_desc->bInterfaceSubClass);

  // hub driver does not support multiple TT yet
  TU_VERIFY(itf_desc->bInterfaceProtocol <= 1);

  // msc driver length is fixed
  uint16_t const drv_len = sizeof(tusb_desc_interface_t) + sizeof(tusb_desc_endpoint_t);
  TU_ASSERT(drv_len <= max_len);

  //------------- Interrupt Status endpoint -------------//
  tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *) tu_desc_next(itf_desc);

  TU_ASSERT(TUSB_DESC_ENDPOINT  == desc_ep->bDescriptorType &&
            TUSB_XFER_INTERRUPT == desc_ep->bmAttributes.xfer, 0);
  
  TU_ASSERT(tuh_edpt_open(dev_addr, desc_ep));

  hub_interface_t* p_hub = get_itf(dev_addr);

  p_hub->itf_num = itf_desc->bInterfaceNumber;
  p_hub->ep_in   = desc_ep->bEndpointAddress;

  return true;
}

void hub_close(uint8_t dev_addr)
{
  TU_VERIFY(dev_addr > CFG_TUH_DEVICE_MAX, );
  hub_interface_t* p_hub = get_itf(dev_addr);

  if (p_hub->ep_in) tu_memclr(p_hub, sizeof( hub_interface_t));
}

bool hub_edpt_status_xfer(uint8_t dev_addr)
{
  hub_interface_t* hub_itf = get_itf(dev_addr);
  TU_VERIFY(hub_itf->ep_in != 0);
  if (hub_itf->status_armed) return true;
  const bool submitted = usbh_edpt_xfer(dev_addr, hub_itf->ep_in,
                                        &hub_itf->status_change, 1);
  if (submitted) hub_itf->status_armed = true;
  return submitted;
}

// Never run an unbounded retry loop inside a TinyUSB callback. The caller of
// tuh_hub_recovery_task() drives retries only after the due time. This also
// recovers synchronous EP0 / status endpoint submission failures.
static void hub_schedule_recovery(uint8_t dev_addr, bool clear_halt)
{
  hub_interface_t* p_hub = get_itf(dev_addr);
  if (p_hub->ep_in == 0) return;
  if (clear_halt) p_hub->clear_halt_pending = true;
  if (p_hub->retry_pending) return;
  p_hub->retry_pending = true;
  if (p_hub->recovery_backoff_ms == 0)
  {
    p_hub->recovery_backoff_ms = HUB_RETRY_INITIAL_MS;
  }
  else
  {
    uint32_t const doubled = p_hub->recovery_backoff_ms * 2u;
    p_hub->recovery_backoff_ms = doubled > HUB_RETRY_MAX_MS ? HUB_RETRY_MAX_MS : doubled;
  }
  p_hub->retry_due_ms = p_hub->last_tick_ms + p_hub->recovery_backoff_ms;
}

// Restart status polling after an error. If the controller rejects the
// submission, schedule a new attempt instead of silently abandoning the hub.
static void hub_status_rearm(uint8_t dev_addr)
{
  hub_interface_t* p_hub = get_itf(dev_addr);
  if (p_hub->ep_in == 0) return;
  if (!hub_edpt_status_xfer(dev_addr))
  {
    hub_schedule_recovery(dev_addr, false);
  }
}

static void hub_status_clear_halt_complete(tuh_xfer_t* xfer);

// A halted interrupt endpoint stays halted until the host clears it, so simply
// re-arming would complete with STALLED forever. Send the standard
// CLEAR_FEATURE(ENDPOINT_HALT). The caller bounds how many times this can run.
static bool hub_status_clear_halt(uint8_t dev_addr)
{
  hub_interface_t* p_hub = get_itf(dev_addr);
  if (p_hub->ep_in == 0) return false;

  tusb_control_request_t const request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_ENDPOINT,
      .type      = TUSB_REQ_TYPE_STANDARD,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = TUSB_REQ_CLEAR_FEATURE,
    .wValue   = 0, // ENDPOINT_HALT
    .wIndex   = p_hub->ep_in,
    .wLength  = 0
  };

  tuh_xfer_t xfer =
  {
    .daddr       = dev_addr,
    .ep_addr     = 0,
    .setup       = &request,
    .buffer      = NULL,
    .complete_cb = hub_status_clear_halt_complete,
    .user_data   = 0
  };

  if (!tuh_control_xfer(&xfer))
  {
    // EP0 may be busy; do not poll a still-halted endpoint.
    hub_schedule_recovery(dev_addr, true);
    return false;
  }
  return true;
}

static void hub_status_clear_halt_complete(tuh_xfer_t* xfer)
{
  uint8_t const daddr = xfer->daddr;
  hub_interface_t* p_hub = get_itf(daddr);
  if (p_hub->ep_in == 0) return;

  if (xfer->result == XFER_RESULT_SUCCESS)
  {
    _hub_diag.status_recoveries++;
    p_hub->status_failures = 0;
    p_hub->clear_halt_pending = false;
    p_hub->recovery_backoff_ms = 0;
    hub_status_rearm(daddr);
  }
  else
  {
    // A failed CLEAR_FEATURE must not re-arm the still-halted interrupt
    // endpoint. Retry the clear asynchronously with bounded backoff.
    _hub_diag.control_failures++;
    hub_schedule_recovery(daddr, true);
  }
}


static void connection_port_reset_complete(tuh_xfer_t* xfer);

// Called from the application main loop, never from an ISR. The host keeps
// servicing other devices between retries, even if the Hub is unplugged.
void tuh_hub_recovery_task(uint32_t now_ms)
{
  for (uint8_t i = 0; i < CFG_TUH_HUB; i++)
  {
    hub_interface_t* p_hub = &hub_data[i];
    if (p_hub->ep_in == 0) continue;
    p_hub->last_tick_ms = now_ms;
    if (!p_hub->retry_pending || (int32_t)(now_ms - p_hub->retry_due_ms) < 0) continue;

    p_hub->retry_pending = false;
    const uint8_t addr = (uint8_t)(CFG_TUH_DEVICE_MAX + 1 + i);
    if (p_hub->clear_halt_pending)
    {
      if (hub_status_clear_halt(addr))
      {
        // Completion callback will re-arm status polling.
        p_hub->clear_halt_pending = false;
      }
    }
    else if (p_hub->pending_reset_port)
    {
      // The connection change bit was already acknowledged. Retry this
      // specific port or a failed EP0 submission can strand its attach.
      uint8_t const port = p_hub->pending_reset_port;
      if (hub_port_reset(addr, port, connection_port_reset_complete, 0))
      {
        p_hub->pending_reset_port = 0;
      }
      else
      {
        _hub_diag.control_failures++;
        hub_schedule_recovery(addr, false);
      }
    }
    else
    {
      hub_status_rearm(addr);
    }
  }
}

//--------------------------------------------------------------------+
// Set Configure
//--------------------------------------------------------------------+

static void config_set_port_power (tuh_xfer_t* xfer);
static void config_port_power_complete (tuh_xfer_t* xfer);

bool hub_set_config(uint8_t dev_addr, uint8_t itf_num)
{
  hub_interface_t* p_hub = get_itf(dev_addr);
  TU_ASSERT(itf_num == p_hub->itf_num);

  // Get Hub Descriptor
  tusb_control_request_t const request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_DEVICE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_IN
    },
    .bRequest = HUB_REQUEST_GET_DESCRIPTOR,
    .wValue   = 0,
    .wIndex   = 0,
    .wLength  = sizeof(descriptor_hub_desc_t)
  };

  tuh_xfer_t xfer =
  {
    .daddr       = dev_addr,
    .ep_addr     = 0,
    .setup       = &request,
    .buffer      = _hub_buffer,
    .complete_cb = config_set_port_power,
    .user_data    = 0
  };

  TU_ASSERT( tuh_control_xfer(&xfer) );

  return true;
}

static void config_set_port_power (tuh_xfer_t* xfer)
{
  uint8_t const daddr = xfer->daddr;
  hub_interface_t* p_hub = get_itf(daddr);
  if (p_hub->ep_in == 0) return;

  if (xfer->result != XFER_RESULT_SUCCESS)
  {
    _hub_diag.control_failures++;
    if (p_hub->control_failures < HUB_RETRY_MAX)
    {
      p_hub->control_failures++;
      if (hub_set_config(daddr, p_hub->itf_num)) return;
    }
    _hub_diag.retry_exhausted++;
    p_hub->control_failures = 0;
    // Keep enumeration moving. Without a descriptor the ports cannot be
    // powered, but other devices on the host must not be held up by it.
    usbh_driver_set_config_complete(daddr, p_hub->itf_num);
    return;
  }
  p_hub->control_failures = 0;

  // only use number of ports in hub descriptor
  descriptor_hub_desc_t const* desc_hub = (descriptor_hub_desc_t const*) _hub_buffer;
  p_hub->port_count = desc_hub->bNbrPorts;
  _hub_diag.port_count = p_hub->port_count;

  // May need to GET_STATUS
  if (p_hub->port_count == 0)
  {
    usbh_driver_set_config_complete(daddr, p_hub->itf_num);
    return;
  }

  // Set Port Power to be able to detect connection, starting with port 1
  uint8_t const hub_port = 1;
  if (!hub_port_set_feature(daddr, hub_port, HUB_FEATURE_PORT_POWER,
                            config_port_power_complete, 0))
  {
    _hub_diag.control_failures++;
    // A failed EP0 submission cannot leave USBH configure hung forever.
    usbh_driver_set_config_complete(daddr, p_hub->itf_num);
    hub_schedule_recovery(daddr, false);
  }
}

static void config_port_power_complete (tuh_xfer_t* xfer)
{
  uint8_t const daddr = xfer->daddr;
  hub_interface_t* p_hub = get_itf(daddr);
  if (p_hub->ep_in == 0) return;

  uint8_t const hub_port = (uint8_t) tu_le16toh(xfer->setup->wIndex);

  if (xfer->result != XFER_RESULT_SUCCESS)
  {
    _hub_diag.control_failures++;
    if (p_hub->control_failures < HUB_RETRY_MAX)
    {
      p_hub->control_failures++;
      if (hub_port_set_feature(daddr, hub_port, HUB_FEATURE_PORT_POWER,
                               config_port_power_complete, 0)) return;
    }
    _hub_diag.retry_exhausted++;
    p_hub->control_failures = 0;
    // Port power is best effort; continue configuring the remaining ports.
  }else
  {
    p_hub->control_failures = 0;
  }

  if (hub_port >= p_hub->port_count)
  {
    // All ports are power -> queue notification status endpoint and
    // complete the SET CONFIGURATION
    if ( !hub_edpt_status_xfer(daddr) )
    {
      _hub_diag.control_failures++;
      hub_schedule_recovery(daddr, false);
    }

    usbh_driver_set_config_complete(daddr, p_hub->itf_num);
  }else
  {
    // power next port
    if (!hub_port_set_feature(daddr, (uint8_t) (hub_port + 1), HUB_FEATURE_PORT_POWER,
                              config_port_power_complete, 0))
    {
      _hub_diag.control_failures++;
      usbh_driver_set_config_complete(daddr, p_hub->itf_num);
      hub_schedule_recovery(daddr, false);
    }
  }
}

//--------------------------------------------------------------------+
// Connection Changes
//--------------------------------------------------------------------+

static void connection_get_status_complete (tuh_xfer_t* xfer);
static void connection_clear_conn_change_complete (tuh_xfer_t* xfer);
static void connection_port_reset_complete (tuh_xfer_t* xfer);
static void port_misc_clear_complete(tuh_xfer_t* xfer);
static void hub_own_get_status_complete(tuh_xfer_t* xfer);
static void hub_own_clear_complete(tuh_xfer_t* xfer);

// Handle the Hub's own local-power and over-current changes (status bit 0).
static bool hub_own_get_status(uint8_t daddr)
{
  hub_interface_t* p_hub = get_itf(daddr);
  tusb_control_request_t const request = {
    .bmRequestType_bit = {
      .recipient = TUSB_REQ_RCPT_DEVICE, .type = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_IN
    },
    .bRequest = HUB_REQUEST_GET_STATUS, .wLength = sizeof(hub_status_response_t)
  };
  tuh_xfer_t xfer = {
    .daddr = daddr, .ep_addr = 0, .setup = &request,
    .buffer = (uint8_t*)&p_hub->own_status,
    .complete_cb = hub_own_get_status_complete
  };
  return tuh_control_xfer(&xfer);
}

static bool hub_own_clear_change(uint8_t daddr, uint8_t feature)
{
  tusb_control_request_t const request = {
    .bmRequestType_bit = {
      .recipient = TUSB_REQ_RCPT_DEVICE, .type = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = HUB_REQUEST_CLEAR_FEATURE, .wValue = feature
  };
  tuh_xfer_t xfer = {
    .daddr = daddr, .ep_addr = 0, .setup = &request,
    .complete_cb = hub_own_clear_complete
  };
  return tuh_control_xfer(&xfer);
}

static void hub_own_get_status_complete(tuh_xfer_t* xfer)
{
  hub_interface_t* p_hub = get_itf(xfer->daddr);
  if (!p_hub->ep_in) return;
  if (xfer->result != XFER_RESULT_SUCCESS)
  {
    _hub_diag.control_failures++;
    hub_status_rearm(xfer->daddr);
    return;
  }
  uint8_t feature = 0;
  bool changed = false;
  if (p_hub->own_status.change.over_current)
  {
    feature = HUB_FEATURE_HUB_OVER_CURRENT_CHANGE;
    changed = true;
  }
  else if (p_hub->own_status.change.local_power_source)
  {
    feature = HUB_FEATURE_HUB_LOCAL_POWER_CHANGE;
    changed = true;
  }
  if (!changed)
  {
    hub_status_rearm(xfer->daddr);
  }
  else if (!hub_own_clear_change(xfer->daddr, feature))
  {
    _hub_diag.control_failures++;
    hub_status_rearm(xfer->daddr);
  }
}

static void hub_own_clear_complete(tuh_xfer_t* xfer)
{
  hub_interface_t* p_hub = get_itf(xfer->daddr);
  if (!p_hub->ep_in) return;
  if (xfer->result != XFER_RESULT_SUCCESS) _hub_diag.control_failures++;
  hub_status_rearm(xfer->daddr);
}

// callback as response of interrupt endpoint polling
bool hub_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) ep_addr;

  hub_interface_t* p_hub = get_itf(dev_addr);
  if (p_hub->ep_in == 0) return false;
  p_hub->status_armed = false; // completed (successfully or not)

  if (result != XFER_RESULT_SUCCESS)
  {
    _hub_diag.status_xfer_failures++;
    if (p_hub->status_failures >= HUB_RETRY_MAX)
    {
      // Bounded immediate retries, followed by slow autonomous recovery.
      // A temporary power dip must not require a Pico reset to recover.
      _hub_diag.retry_exhausted++;
      hub_schedule_recovery(dev_addr, result == XFER_RESULT_STALLED);
      return false;
    }
    p_hub->status_failures++;

    if (result == XFER_RESULT_STALLED)
    {
      (void) hub_status_clear_halt(dev_addr);
    }else
    {
      hub_status_rearm(dev_addr);
    }
    return false;
  }

  p_hub->status_failures = 0;
  p_hub->recovery_backoff_ms = 0;
  p_hub->retry_pending = false;

  // Some hubs complete with no data or an empty change bitmap. Nothing would
  // queue the next status transfer, so poll again here.
  if (xferred_bytes == 0 || p_hub->status_change == 0)
  {
    return hub_edpt_status_xfer(dev_addr);
  }

  TU_LOG2("  Port Status Change = 0x%02X\r\n", p_hub->status_change);
  _hub_diag.port_changes++;

  // Process one port at a time. Remaining change bits will be reported again
  // after completing the active control request.
  for (uint8_t port=1; port <= p_hub->port_count; port++)
  {
    if (tu_bit_test(p_hub->status_change, port))
    {
      if (!hub_port_get_status(dev_addr, port, &p_hub->port_status,
                               connection_get_status_complete, 0))
      {
        _hub_diag.control_failures++;
        hub_status_rearm(dev_addr);
      }
      return true;
    }
  }
  if (tu_bit_test(p_hub->status_change, 0))
  {
    if (!hub_own_get_status(dev_addr))
    {
      _hub_diag.control_failures++;
      hub_status_rearm(dev_addr);
    }
    return true;
  }
  // Unknown/unsupported bitmap: never abandon port-change monitoring.
  hub_status_rearm(dev_addr);
  return true;
}

static void connection_get_status_complete (tuh_xfer_t* xfer)
{
  uint8_t const daddr = xfer->daddr;
  hub_interface_t* p_hub = get_itf(daddr);
  if (p_hub->ep_in == 0) return;

  if (xfer->result != XFER_RESULT_SUCCESS)
  {
    _hub_diag.control_failures++;
    // The connection change bit is still set, so polling the status endpoint
    // again will report this port once more.
    hub_status_rearm(daddr);
    return;
  }

  uint8_t const port_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);

  // Connection change
  if (p_hub->port_status.change.connection)
  {
    // Port is powered and enabled
    //TU_VERIFY(port_status.status_current.port_power && port_status.status_current.port_enable, );

    // Acknowledge Port Connection Change
    if (!hub_port_clear_feature(daddr, port_num, HUB_FEATURE_PORT_CONNECTION_CHANGE,
                                connection_clear_conn_change_complete, 0))
    {
      _hub_diag.control_failures++;
      hub_status_rearm(daddr);
    }
  }
  else
  {
    // These latching C_PORT flags must be acknowledged, especially
    // OVER_CURRENT, or the Hub can continuously reassert its interrupt.
    uint8_t feature = 0;
    bool changed = false;
    if (p_hub->port_status.change.over_current)
    {
      feature = HUB_FEATURE_PORT_OVER_CURRENT_CHANGE;
      changed = true;
    }
    else if (p_hub->port_status.change.port_enable)
    {
      feature = HUB_FEATURE_PORT_ENABLE_CHANGE;
      changed = true;
    }
    else if (p_hub->port_status.change.reset)
    {
      feature = HUB_FEATURE_PORT_RESET_CHANGE;
      changed = true;
    }
    else if (p_hub->port_status.change.suspend)
    {
      feature = HUB_FEATURE_PORT_SUSPEND_CHANGE;
      changed = true;
    }
    if (changed)
    {
      if (!hub_port_clear_feature(daddr, port_num, feature,
                                  port_misc_clear_complete, 0))
      {
        _hub_diag.control_failures++;
        hub_status_rearm(daddr);
      }
    }
    else
    {
      hub_status_rearm(daddr);
    }
  }
}

static void port_misc_clear_complete(tuh_xfer_t* xfer)
{
  hub_interface_t* p_hub = get_itf(xfer->daddr);
  if (!p_hub->ep_in) return;
  if (xfer->result != XFER_RESULT_SUCCESS) _hub_diag.control_failures++;
  hub_status_rearm(xfer->daddr);
}

static void connection_clear_conn_change_complete (tuh_xfer_t* xfer)
{
  uint8_t const daddr = xfer->daddr;
  hub_interface_t* p_hub = get_itf(daddr);
  if (p_hub->ep_in == 0) return;

  if (xfer->result != XFER_RESULT_SUCCESS)
  {
    _hub_diag.control_failures++;
    // Clear failed, so the change bit should still be set. Re-arm and retry
    // through the normal status path.
    hub_status_rearm(daddr);
    return;
  }

  uint8_t const port_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);

  if ( p_hub->port_status.status.connection )
  {
    // Reset port if attach event
    if (!hub_port_reset(daddr, port_num, connection_port_reset_complete, 0))
    {
      _hub_diag.control_failures++;
      p_hub->pending_reset_port = port_num;
      hub_schedule_recovery(daddr, false);
    }
  }else
  {
    // submit detach event
    hcd_event_t event =
    {
      .rhport     = usbh_get_rhport(daddr),
      .event_id   = HCD_EVENT_DEVICE_REMOVE,
      .connection =
       {
         .hub_addr = daddr,
         .hub_port = port_num
       }
    };

    hcd_event_handler(&event, false);
  }
}

static void connection_port_reset_complete (tuh_xfer_t* xfer)
{
  uint8_t const daddr = xfer->daddr;
  hub_interface_t* p_hub = get_itf(daddr);
  if (p_hub->ep_in == 0) return;

  uint8_t const port_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);

  if (xfer->result != XFER_RESULT_SUCCESS)
  {
    _hub_diag.control_failures++;
    if (p_hub->reset_failures < HUB_RETRY_MAX)
    {
      p_hub->reset_failures++;
      if (hub_port_reset(daddr, port_num, connection_port_reset_complete, 0)) return;
    }
    _hub_diag.retry_exhausted++;
    p_hub->reset_failures = 0;
    // The connection bit is already cleared. Preserve a scheduled local
    // port reset instead of requiring the user to physically replug it.
    p_hub->pending_reset_port = port_num;
    hub_schedule_recovery(daddr, false);
    hcd_event_t remove_event =
    {
      .rhport     = usbh_get_rhport(daddr),
      .event_id   = HCD_EVENT_DEVICE_REMOVE,
      .connection =
      {
        .hub_addr = daddr,
        .hub_port = port_num
      }
    };
    hcd_event_handler(&remove_event, false);
    return;
  }
  p_hub->reset_failures = 0;
  p_hub->pending_reset_port = 0;
  p_hub->recovery_backoff_ms = 0;

  // submit attach event
  hcd_event_t event =
  {
    .rhport     = usbh_get_rhport(daddr),
    .event_id   = HCD_EVENT_DEVICE_ATTACH,
    .connection =
    {
      .hub_addr = daddr,
      .hub_port = port_num
    }
  };

  hcd_event_handler(&event, false);
}

#endif
