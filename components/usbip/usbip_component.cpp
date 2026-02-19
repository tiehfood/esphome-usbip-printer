#include "usbip_component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// ESP-IDF USB Host includes
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_intr_alloc.h"
#include "esp_task_wdt.h"
#include "hal/usb_dwc_hal.h"
#include "hal/usb_dwc_ll.h"
#include "soc/usb_dwc_struct.h"

namespace esphome {
namespace usbip {

// ============================================================================
// Direct USB DWC2 Hardware Access - Scatter/Gather DMA Mode
// This bypasses ESP-IDF's USB Host driver for bulk OUT transfers
// which don't work through the normal ESP-IDF API (suspected ESP-IDF bug).
// Uses the same DMA mechanism as ESP-IDF but with our own channel (ch 7).
// ============================================================================

#include "esp_heap_caps.h"
#include "esp_cache.h"

// ============================================================================
// BULK OUT MODE SELECTION
// Set to 0 to use standard ESP-IDF API (usb_host_transfer_submit)
// Set to 1 to use direct Buffer DMA mode (bypasses ESP-IDF)
// Try mode 0 first - it works for other people with printers on ESP32-S3
// ============================================================================
#define USE_DIRECT_BULK_OUT 0  // Try standard ESP-IDF API - others report success with printers

#define DIRECT_CHANNEL 7  // Use channel 7 (highest) to avoid conflict with ESP-IDF channels
#define BULK_MPS 64       // Full-Speed bulk max packet size
#define USB_PID_DATA0 0
#define USB_PID_DATA1 2
#define DMA_BUFFER_SIZE 1024

// Result codes for direct USB transfers
enum class DirectUSBResult {
  OK = 0,
  ERROR_NO_CHANNEL = -1,
  ERROR_ALLOC = -2,
  ERROR_TIMEOUT = -3,
  ERROR_STALL = -4,
  ERROR_NAK = -5,
  ERROR_XACT = -6,
  ERROR_BABBLE = -7,
  ERROR_HALTED = -8,
};

// Direct USB state
static bool direct_usb_initialized = false;
static uint8_t current_data_pid = USB_PID_DATA1;  // Printer expects DATA1 after ESP-IDF init

// DMA resources - QTD list must be 512-byte aligned
static usb_dwc_ll_dma_qtd_t *direct_qtd_list = nullptr;
static uint8_t *dma_data_buffer = nullptr;

static const char *const TAG_DIRECT = "usb_direct";

// Initialize direct USB access and allocate DMA resources
static void direct_usb_init() {
  if (direct_usb_initialized) {
    return;
  }

  ESP_LOGD(TAG_DIRECT, "Initializing direct USB DMA access");

  usb_dwc_dev_t *hw = &USB_DWC;
  uint32_t snpsid = hw->gsnpsid_reg.val;
  ESP_LOGD(TAG_DIRECT, "USB DWC Core ID: 0x%08lX", (unsigned long)snpsid);

  uint32_t num_channels = hw->ghwcfg2_reg.numhstchnl + 1;
  ESP_LOGD(TAG_DIRECT, "Available channels: %lu, using channel %d", (unsigned long)num_channels, DIRECT_CHANNEL);

  if (DIRECT_CHANNEL >= (int)num_channels) {
    ESP_LOGE(TAG_DIRECT, "Channel %d not available", DIRECT_CHANNEL);
    return;
  }

  // Allocate 512-byte aligned QTD list (need at least 2 QTDs: 1 data + safety)
  direct_qtd_list = (usb_dwc_ll_dma_qtd_t *)heap_caps_aligned_alloc(
      512, sizeof(usb_dwc_ll_dma_qtd_t) * 4, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!direct_qtd_list) {
    ESP_LOGE(TAG_DIRECT, "Failed to allocate QTD list");
    return;
  }
  memset(direct_qtd_list, 0, sizeof(usb_dwc_ll_dma_qtd_t) * 4);
  ESP_LOGD(TAG_DIRECT, "QTD list at %p (aligned=%d)", direct_qtd_list,
           ((uintptr_t)direct_qtd_list % 512) == 0);

  // Allocate DMA-capable data buffer
  dma_data_buffer = (uint8_t *)heap_caps_aligned_alloc(
      64, DMA_BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!dma_data_buffer) {
    ESP_LOGE(TAG_DIRECT, "Failed to allocate DMA buffer");
    heap_caps_free(direct_qtd_list);
    direct_qtd_list = nullptr;
    return;
  }
  ESP_LOGD(TAG_DIRECT, "DMA buffer at %p", dma_data_buffer);

  direct_usb_initialized = true;
  ESP_LOGI(TAG_DIRECT, "Direct USB DMA access initialized");
}

// Perform bulk OUT using Buffer DMA mode on channel 7
// SG-DMA mode (descdma=1) freezes for all OUT eptypes on ESP32-S3.
// Buffer DMA mode (descdma=0) uses HCTSIZ xfersize/pktcnt directly and
// HCDMA points to data buffer, not QTD list. This is a completely different
// hardware path that may bypass the SG-DMA freeze bug.
static DirectUSBResult direct_bulk_out(uint8_t dev_addr, uint8_t ep_num,
                                        const uint8_t *data, uint32_t length,
                                        uint32_t timeout_ms, uint32_t *actual_sent) {
  if (!direct_usb_initialized) {
    direct_usb_init();
  }
  if (!direct_usb_initialized) {
    return DirectUSBResult::ERROR_ALLOC;
  }

  usb_dwc_dev_t *hw = &USB_DWC;
  volatile usb_dwc_host_chan_regs_t *chan = &hw->host_chans[DIRECT_CHANNEL];

  *actual_sent = 0;

  // Save all state we'll modify
  uint32_t saved_hcfg = hw->hcfg_reg.val;
  uint32_t saved_haintmsk = hw->haintmsk_reg.val;
  uint32_t saved_hcchar = chan->hcchar_reg.val;
  uint32_t saved_hctsiz = chan->hctsiz_reg.val;
  uint32_t saved_hcdma = chan->hcdma_reg.val;
  uint32_t saved_hcintmsk = chan->hcintmsk_reg.val;

  // === CRITICAL: Switch ALL channels to Buffer DMA mode ===
  // First, disable global interrupts to prevent ESP-IDF ISR from running
  // while we're in buffer DMA mode
  hw->gahbcfg_reg.glbllntrmsk = 0;

  // Halt all active channels before switching DMA mode
  for (int ch = 0; ch < 8; ch++) {
    volatile usb_dwc_host_chan_regs_t *c = &hw->host_chans[ch];
    if (c->hcchar_reg.chena) {
      c->hcchar_reg.chdis = 1;
      c->hcchar_reg.chena = 1;
      uint32_t t = 1000;
      while (c->hcchar_reg.chena && t--) esp_rom_delay_us(1);
      c->hcint_reg.val = 0xFFFFFFFF;
    }
  }

  // Switch to Buffer DMA mode (descdma=0)
  hw->hcfg_reg.descdma = 0;

  // Configure our channel
  chan->hcchar_reg.val = 0;
  chan->hcint_reg.val = 0xFFFFFFFF;
  chan->hcintmsk_reg.val = 0x000007FF;  // All channel interrupts

  // Enable channel in HAINTMSK (for HAINT register visibility, even with glbllntrmsk=0)
  hw->haintmsk_reg.val |= (1 << DIRECT_CHANNEL);
  // Keep global interrupts DISABLED during entire Buffer DMA transfer.
  // ESP-IDF ISR expects SG-DMA mode and will crash if it runs in Buffer DMA mode.
  // We poll HCINT directly so interrupts aren't needed.

  uint32_t bytes_sent = 0;
  DirectUSBResult result = DirectUSBResult::OK;
  uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

  // Force all-DATA1 mode: This printer NAKs all DATA0 packets regardless of
  // SET_INTERFACE or CLEAR_FEATURE toggle resets. This is a confirmed hardware
  // incompatibility between the DWC2 Buffer DMA mode and this printer.
  // All-DATA1 transfers complete with 0 NAKs.
  current_data_pid = USB_PID_DATA1;

  while (bytes_sent < length) {
    uint32_t packet_size = length - bytes_sent;
    if (packet_size > BULK_MPS) {
      packet_size = BULK_MPS;
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - start_time) >= timeout_ms) {
      ESP_LOGW(TAG_DIRECT, "Timeout after %lu bytes", (unsigned long)bytes_sent);
      result = DirectUSBResult::ERROR_TIMEOUT;
      break;
    }

    // Copy data to DMA buffer and flush cache
    memcpy(dma_data_buffer, data + bytes_sent, packet_size);
    esp_cache_msync(dma_data_buffer, DMA_BUFFER_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    // Configure HCCHAR for BULK OUT
    usb_dwc_hcchar_reg_t hcchar;
    hcchar.val = 0;
    hcchar.mps = BULK_MPS;
    hcchar.epnum = ep_num;
    hcchar.epdir = 0;  // OUT
    hcchar.lspddev = 0;
    hcchar.eptype = 2;  // BULK
    hcchar.devaddr = dev_addr;
    chan->hcchar_reg.val = hcchar.val;

    // Single-packet transfer: xfersize=packet_size, pktcnt=1, pid=DATA1
    // ESP32-S3 DWC2 freezes on multi-packet OUT in both SG-DMA and Buffer DMA modes
    usb_dwc_hctsiz_reg_t hctsiz;
    hctsiz.val = 0;
    hctsiz.xfersize = packet_size;
    hctsiz.pktcnt = 1;
    hctsiz.pid = current_data_pid;
    hctsiz.dopng = 0;
    chan->hctsiz_reg.val = hctsiz.val;

    chan->hcdma_reg.val = (uint32_t)dma_data_buffer;

    chan->hcint_reg.val = 0xFFFFFFFF;
    chan->hcchar_reg.chena = 1;
    chan->hcchar_reg.chdis = 0;

    bool transfer_done = false;
    uint32_t wait_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t nak_count = 0;

    while (!transfer_done) {
      uint32_t hcint = chan->hcint_reg.val;

      if (hcint & (1 << 0)) {  // XFERCOMPL
        transfer_done = true;
        bytes_sent += packet_size;
        // No toggle - all-DATA1 mode (printer NAKs DATA0)
        chan->hcint_reg.val = 0xFFFFFFFF;
      } else if (hcint & (1 << 3)) {  // STALL
        chan->hcint_reg.val = 0xFFFFFFFF;
        ESP_LOGW(TAG_DIRECT, "STALL");
        result = DirectUSBResult::ERROR_STALL;
        transfer_done = true;
      } else if (hcint & (1 << 7)) {  // XACTERR
        chan->hcint_reg.val = 0xFFFFFFFF;
        ESP_LOGW(TAG_DIRECT, "XACTERR");
        result = DirectUSBResult::ERROR_XACT;
        transfer_done = true;
      } else if (hcint & (1 << 4)) {  // NAK
        nak_count++;
        chan->hcint_reg.val = hcint;

        if (hcint & (1 << 1)) {
          chan->hctsiz_reg.val = hctsiz.val;
          chan->hcdma_reg.val = (uint32_t)dma_data_buffer;
          chan->hcint_reg.val = 0xFFFFFFFF;
          chan->hcchar_reg.chena = 1;
          chan->hcchar_reg.chdis = 0;
        }
      } else if (hcint & (1 << 5)) {  // ACK only
        chan->hcint_reg.val = hcint;
      } else if (hcint & (1 << 1)) {  // CHHLTD unknown
        chan->hcint_reg.val = 0xFFFFFFFF;
        ESP_LOGW(TAG_DIRECT, "CHHLTD unknown: HCINT=0x%08lX", (unsigned long)hcint);
        result = DirectUSBResult::ERROR_HALTED;
        transfer_done = true;
      } else if (hcint != 0) {
        chan->hcint_reg.val = hcint;
      }

      uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - wait_start;

      if (!transfer_done && elapsed >= 2000) {
        ESP_LOGW(TAG_DIRECT, "Timeout %lums NAKs=%lu",
                 (unsigned long)elapsed, (unsigned long)nak_count);

        chan->hcchar_reg.chdis = 1;
        chan->hcchar_reg.chena = 1;
        vTaskDelay(pdMS_TO_TICKS(5));
        chan->hcint_reg.val = 0xFFFFFFFF;
        result = DirectUSBResult::ERROR_NAK;
        transfer_done = true;
      }

      if (!transfer_done) {
        esp_task_wdt_reset();
        if (nak_count > 0 && (nak_count % 10) == 0) {
          vTaskDelay(1);
        }
      }
    }

    if (result != DirectUSBResult::OK) break;
    esp_task_wdt_reset();
  }

  // === Restore: switch back to SG-DMA mode ===
  hw->gahbcfg_reg.glbllntrmsk = 0;  // Disable interrupts during mode switch

  // Halt our channel
  if (chan->hcchar_reg.chena) {
    chan->hcchar_reg.chdis = 1;
    chan->hcchar_reg.chena = 1;
    uint32_t t = 1000;
    while (chan->hcchar_reg.chena && t--) esp_rom_delay_us(1);
  }

  // Restore SG-DMA mode
  hw->hcfg_reg.val = saved_hcfg;

  // Restore channel state
  chan->hcchar_reg.val = saved_hcchar;
  chan->hctsiz_reg.val = saved_hctsiz;
  chan->hcdma_reg.val = saved_hcdma;
  chan->hcint_reg.val = 0xFFFFFFFF;
  chan->hcintmsk_reg.val = saved_hcintmsk;
  hw->haintmsk_reg.val = saved_haintmsk;

  // Re-enable global interrupts
  hw->gahbcfg_reg.glbllntrmsk = 1;

  *actual_sent = bytes_sent;
  if (result != DirectUSBResult::OK) {
    ESP_LOGW(TAG_DIRECT, "Bulk OUT failed: sent=%lu/%lu result=%d",
             (unsigned long)bytes_sent, (unsigned long)length, (int)result);
  }

  return result;
}

// ============================================================================
// End Direct USB DWC2 Hardware Access
// ============================================================================

static const char *const TAG = "usbip";

// USB standard requests
static constexpr uint8_t USB_REQ_GET_STATUS = 0x00;
static constexpr uint8_t USB_REQ_CLEAR_FEATURE = 0x01;
static constexpr uint8_t USB_REQ_SET_FEATURE = 0x03;
static constexpr uint8_t USB_REQ_SET_ADDRESS = 0x05;
static constexpr uint8_t USB_REQ_GET_DESCRIPTOR = 0x06;
static constexpr uint8_t USB_REQ_SET_DESCRIPTOR = 0x07;
static constexpr uint8_t USB_REQ_GET_CONFIGURATION = 0x08;
static constexpr uint8_t USB_REQ_SET_CONFIGURATION = 0x09;
static constexpr uint8_t USB_REQ_GET_INTERFACE = 0x0A;
static constexpr uint8_t USB_REQ_SET_INTERFACE = 0x0B;

// USB descriptor types
static constexpr uint8_t USB_DESC_DEVICE = 0x01;
static constexpr uint8_t USB_DESC_CONFIGURATION = 0x02;
static constexpr uint8_t USB_DESC_STRING = 0x03;
static constexpr uint8_t USB_DESC_INTERFACE = 0x04;
static constexpr uint8_t USB_DESC_ENDPOINT = 0x05;

// USB Printer Class requests (class code 0x07)
static constexpr uint8_t USB_PRINTER_CLASS = 0x07;
static constexpr uint8_t PRINTER_REQ_GET_DEVICE_ID = 0x00;
static constexpr uint8_t PRINTER_REQ_GET_PORT_STATUS = 0x01;
static constexpr uint8_t PRINTER_REQ_SOFT_RESET = 0x02;

// HP device identifiers
static constexpr uint16_t HP_VID = 0x03F0;

#ifdef HAS_PRINTER_FIRMWARE
// Printer firmware data (generated by ESPHome from YAML config)
extern const uint8_t printer_fw_data[];
extern const size_t printer_fw_data_size;
#endif

// Global component reference for callbacks
USBIPComponent *g_usbip_component = nullptr;

// USB Host library client handle
static usb_host_client_handle_t usb_client_hdl = nullptr;
static usb_device_handle_t usb_device_hdl = nullptr;
static SemaphoreHandle_t usb_event_sem = nullptr;

// Transfer completion handling
static SemaphoreHandle_t transfer_done_sem = nullptr;
static volatile bool transfer_completed = false;
static volatile usb_transfer_status_t transfer_status = USB_TRANSFER_STATUS_ERROR;
static volatile int transfer_actual_bytes = 0;
static volatile uint32_t transfer_seq_expected = 0;  // Expected sequence number
static volatile uint32_t transfer_seq_completed = 0; // Completed sequence number
static uint32_t transfer_seq_counter = 1;            // Global sequence counter

// Printer state tracking
static bool printer_initialized = false;
static int8_t printer_interface_num = -1;
static bool interface_selected = false;  // Track if SET_INTERFACE was sent
static bool toggle_reset_for_job = false;  // Track if toggle was reset for current print job

// USB Host daemon task handle
static TaskHandle_t usb_daemon_task_hdl = nullptr;
static volatile bool usb_daemon_running = false;

// Enumeration filter callback - required when CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK is enabled
// Returns true to allow enumeration of the device, false to reject it
// This callback fires BEFORE the USB_HOST_CLIENT_EVENT_NEW_DEV callback
static bool enum_filter_callback(const usb_device_desc_t *dev_desc, uint8_t *bConfigurationValue) {
  ESP_LOGD("usbip", "Enum filter: VID=%04X PID=%04X Class=%02X",
           dev_desc->idVendor, dev_desc->idProduct, dev_desc->bDeviceClass);
  // Use configuration value 1 (most common)
  *bConfigurationValue = 1;
  return true;  // Allow all devices
}

// Pending device actions (set by callback, processed by main loop)
static volatile uint8_t pending_dev_addr = 0;
static volatile bool pending_dev_connect = false;
static volatile bool pending_dev_disconnect = false;

// Device discovery request (set by daemon task, processed by main loop)
// This avoids calling usb_host_device_open() from within the daemon task
static volatile bool request_device_discovery = false;

// Port power state - start unpowered, power on after client is registered
static volatile bool port_power_requested = false;
static volatile bool port_powered = false;

// Helper function to send SET_INTERFACE(interface, alt_setting) control transfer
// USB Spec 9.1.1.5: "This includes setting the data toggle of any endpoint
// using data toggles to the value DATA0."
static bool send_set_interface(uint8_t interface_num, uint8_t alt_setting) {
  if (!usb_device_hdl || !usb_client_hdl) {
    ESP_LOGW("usb_direct", "send_set_interface: device not ready");
    return false;
  }

  ESP_LOGD("usb_direct", "Sending SET_INTERFACE(%d, %d) to reset DATA toggle to DATA0",
           interface_num, alt_setting);

  usb_transfer_t *transfer = nullptr;
  // Must allocate at least 8 bytes for setup packet in data_buffer
  esp_err_t err = usb_host_transfer_alloc(64, 0, &transfer);
  if (err != ESP_OK || !transfer) {
    ESP_LOGE("usb_direct", "Failed to allocate control transfer: %s", esp_err_to_name(err));
    return false;
  }

  // SET_INTERFACE: bmRequestType=0x01 (Host-to-Device, Standard, Interface)
  //                bRequest=0x0B (SET_INTERFACE)
  //                wValue=alt_setting, wIndex=interface
  transfer->device_handle = usb_device_hdl;
  transfer->bEndpointAddress = 0;  // Control endpoint
  transfer->callback = [](usb_transfer_t *t) {
    // Signal completion via semaphore
    SemaphoreHandle_t sem = (SemaphoreHandle_t)t->context;
    if (sem) xSemaphoreGive(sem);
  };

  SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
  transfer->context = done_sem;
  transfer->num_bytes = 8;  // Setup packet only, no data phase

  // Build setup packet
  usb_setup_packet_t *setup = (usb_setup_packet_t *)transfer->data_buffer;
  setup->bmRequestType = 0x01;  // OUT, Standard, Interface
  setup->bRequest = 0x0B;       // SET_INTERFACE
  setup->wValue = alt_setting;
  setup->wIndex = interface_num;
  setup->wLength = 0;

  err = usb_host_transfer_submit_control(usb_client_hdl, transfer);
  if (err != ESP_OK) {
    ESP_LOGW("usb_direct", "SET_INTERFACE submit failed: %s", esp_err_to_name(err));
    vSemaphoreDelete(done_sem);
    usb_host_transfer_free(transfer);
    return false;
  }

  // Wait for completion
  bool success = false;
  if (xSemaphoreTake(done_sem, pdMS_TO_TICKS(1000)) == pdTRUE) {
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
      ESP_LOGD("usb_direct", "SET_INTERFACE completed successfully - toggle should be DATA0");
      success = true;
      // Reset our software toggle to DATA0 per USB spec
      current_data_pid = USB_PID_DATA0;
    } else {
      ESP_LOGW("usb_direct", "SET_INTERFACE status: %d", transfer->status);
    }
  } else {
    ESP_LOGW("usb_direct", "SET_INTERFACE timeout");
  }

  vSemaphoreDelete(done_sem);
  usb_host_transfer_free(transfer);

  // Process any pending events
  if (usb_client_hdl) {
    usb_host_client_handle_events(usb_client_hdl, 10);
  }

  return success;
}

// USB Host daemon task - continuously processes USB events in background
static void usb_host_daemon_task(void *arg) {
  ESP_LOGD("usbip", "USB Host daemon task started");
  usb_daemon_running = true;

  static uint32_t startup_time = 0;
  static bool initial_discovery_done = false;

  // Record startup time for enumeration wait
  startup_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

  while (usb_daemon_running) {
    // Process library-level events (device connect/disconnect)
    uint32_t event_flags = 0;
    esp_err_t err = usb_host_lib_handle_events(100, &event_flags);

    // Log library events
    if (event_flags != 0) {
      ESP_LOGD("usbip", "USB lib event flags: 0x%lX", (unsigned long)event_flags);
    }
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGE("usbip", "usb_host_lib_handle_events error: %s", esp_err_to_name(err));
    }

    // Process client-level events (transfer completions, device events)
    if (usb_client_hdl) {
      err = usb_host_client_handle_events(usb_client_hdl, 100);
      if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        ESP_LOGE("usbip", "usb_host_client_handle_events error: %s", esp_err_to_name(err));
      }
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Power on the port 500ms after startup (once client is registered)
    // This ensures enumeration happens AFTER our client is ready to receive NEW_DEV callback
    if (port_power_requested && !port_powered && (now - startup_time) > 500) {
      ESP_LOGI("usbip", "Powering on USB root port...");
      err = usb_host_lib_set_root_port_power(true);
      if (err == ESP_OK) {
        ESP_LOGD("usbip", "USB root port powered ON - waiting for device enumeration");
        port_powered = true;
      } else {
        ESP_LOGE("usbip", "Failed to power on root port: %s", esp_err_to_name(err));
      }
    }

    // Wait 5 seconds after port power-on before checking for manual discovery
    // This gives time for device enumeration via the normal callback path
    if (port_powered && !initial_discovery_done && (now - startup_time) > 5500) {
      initial_discovery_done = true;

      usb_host_lib_info_t lib_info;
      if (usb_host_lib_info(&lib_info) == ESP_OK) {
        ESP_LOGD("usbip", "Enumeration check: %d devices in library, handle=%p",
                 lib_info.num_devices, usb_device_hdl);

        // Only request manual discovery if device in library but callback never fired
        if (lib_info.num_devices > 0 && usb_device_hdl == nullptr && !pending_dev_connect) {
          ESP_LOGW("usbip", "Device in library but NEW_DEV callback never fired - requesting manual discovery");
          request_device_discovery = true;
        }
      }
    }

    // Small delay to prevent CPU hogging and feed watchdog
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_LOGD("usbip", "USB Host daemon task stopped");
  vTaskDelete(NULL);
}

// USB transfer completion callback (called from USB Host task context, not ISR)
static void usb_transfer_callback(usb_transfer_t *transfer) {
  uint32_t seq = (uint32_t)(uintptr_t)transfer->context;
  ESP_LOGD("usbip", "Transfer callback: seq=%lu status=%d actual=%d ep=0x%02X",
           (unsigned long)seq, transfer->status, transfer->actual_num_bytes, transfer->bEndpointAddress);

  // Only update if this is the transfer we're waiting for
  if (seq == transfer_seq_expected) {
    transfer_status = transfer->status;
    transfer_actual_bytes = transfer->actual_num_bytes;
    transfer_seq_completed = seq;
    transfer_completed = true;
    if (transfer_done_sem) {
      xSemaphoreGive(transfer_done_sem);
    }
  } else {
    ESP_LOGD("usbip", "Ignoring stale callback: expected seq=%lu got seq=%lu",
             (unsigned long)transfer_seq_expected, (unsigned long)seq);
  }
}

// Debug function to dump USB DWC2 hardware state
static void dump_usb_hw_state(const char *context) {
  // Access the USB DWC2 registers directly
  usb_dwc_dev_t *usb_dwc = &USB_DWC;

  // Read key registers
  uint32_t gintsts = usb_dwc->gintsts_reg.val;
  uint32_t hprt = usb_dwc->hprt_reg.val;

  // Check port status
  bool port_connected = (hprt >> 0) & 1;
  bool port_enabled = (hprt >> 2) & 1;
  uint32_t port_speed = (hprt >> 17) & 3;
  const char *speed_str = (port_speed == 0) ? "HS" : (port_speed == 1) ? "FS" : "LS";

  ESP_LOGD("usb_hw", "%s: HPRT=0x%08lX conn=%d en=%d speed=%s GINTSTS=0x%08lX",
           context, (unsigned long)hprt, port_connected, port_enabled,
           speed_str, (unsigned long)gintsts);
}

// Prepare a transfer with a unique sequence number
static uint32_t prepare_transfer(usb_transfer_t *transfer) {
  uint32_t seq = transfer_seq_counter++;
  transfer->context = (void *)(uintptr_t)seq;
  transfer_seq_expected = seq;
  transfer_completed = false;
  transfer_status = USB_TRANSFER_STATUS_ERROR;
  transfer_actual_bytes = 0;
  if (transfer_done_sem) {
    xSemaphoreTake(transfer_done_sem, 0);  // Clear any pending signals
  }
  return seq;
}

// Wait for transfer completion - actively process USB events while waiting
static bool wait_for_transfer(uint32_t timeout_ms, bool dump_hw_state = false) {
  uint32_t start = millis();
  uint32_t last_dump = 0;

  // Actively process USB events while waiting
  while (!transfer_completed && (millis() - start) < timeout_ms) {
    // Process USB Host events directly - don't rely only on daemon task
    uint32_t event_flags;
    usb_host_lib_handle_events(1, &event_flags);
    if (usb_client_hdl) {
      usb_host_client_handle_events(usb_client_hdl, 1);
    }
    esp_task_wdt_reset();

    // Periodically dump hardware state if requested (every 500ms)
    uint32_t elapsed = millis() - start;
    if (dump_hw_state && (elapsed - last_dump) >= 500) {
      char buf[32];
      snprintf(buf, sizeof(buf), "wait @%lums", (unsigned long)elapsed);
      dump_usb_hw_state(buf);
      last_dump = elapsed;
    }
  }

  ESP_LOGD("usbip", "Wait ended: completed=%d, elapsed=%lums",
           transfer_completed, (unsigned long)(millis() - start));
  return transfer_completed;
}

// USB Host event callback - called from usb_host_client_handle_events context
// IMPORTANT: Do not block or do heavy work here - just set flags for daemon task
static void usb_host_client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg) {
  switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
      ESP_LOGD(TAG, "USB_HOST_CLIENT_EVENT_NEW_DEV: address=%d", event_msg->new_dev.address);
      pending_dev_addr = event_msg->new_dev.address;
      pending_dev_connect = true;
      break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
      ESP_LOGD(TAG, "USB_HOST_CLIENT_EVENT_DEV_GONE");
      pending_dev_disconnect = true;
      break;
    default:
      ESP_LOGW(TAG, "Unknown USB client event: %d", event_msg->event);
      break;
  }
  if (usb_event_sem) {
    xSemaphoreGive(usb_event_sem);
  }
}

USBIPComponent::USBIPComponent() {
  g_usbip_component = this;
}

USBIPComponent::~USBIPComponent() {
  // Stop USB daemon task first
  if (usb_daemon_task_hdl) {
    usb_daemon_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));  // Give task time to exit
    usb_daemon_task_hdl = nullptr;
  }

  if (this->client_fd_ >= 0) {
    close(this->client_fd_);
  }
  if (this->server_fd_ >= 0) {
    close(this->server_fd_);
  }
  if (usb_client_hdl) {
    usb_host_client_deregister(usb_client_hdl);
    usb_client_hdl = nullptr;
  }
  if (usb_event_sem) {
    vSemaphoreDelete(usb_event_sem);
    usb_event_sem = nullptr;
  }
  g_usbip_component = nullptr;
}

float USBIPComponent::get_setup_priority() const {
  return setup_priority::AFTER_WIFI;
}

void USBIPComponent::setup() {
  // Initialize USB Host
  this->usb_host_init_();

  // Start TCP server
  this->start_server_();
}

void USBIPComponent::usb_host_init_() {

  // Create semaphore for USB events
  usb_event_sem = xSemaphoreCreateBinary();
  if (!usb_event_sem) {
    ESP_LOGE(TAG, "Failed to create USB event semaphore");
    return;
  }

  // Create semaphore for transfer completion
  transfer_done_sem = xSemaphoreCreateBinary();
  if (!transfer_done_sem) {
    ESP_LOGE(TAG, "Failed to create transfer semaphore");
    return;
  }

  // Install USB Host library
  // IMPORTANT: Start with port unpowered so enumeration happens AFTER client registration
  // This ensures the NEW_DEV callback fires to our client, enabling proper interface claiming
  // CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK is enabled, so we provide a callback
  usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .root_port_unpowered = true,  // Start unpowered, power on after client is registered
      .intr_flags = ESP_INTR_FLAG_LOWMED,
      .enum_filter_cb = enum_filter_callback,  // Required when ENABLE_ENUM_FILTER_CALLBACK is set
      .fifo_settings_custom = {0, 0, 0},       // Use Kconfig defaults
      .peripheral_map = 0,                     // Use default peripheral
  };

  esp_err_t err = usb_host_install(&host_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install USB Host library: %s", esp_err_to_name(err));
    return;
  }

  ESP_LOGD(TAG, "USB Host library installed");

  // Register USB Host client
  usb_host_client_config_t client_config = {
      .is_synchronous = false,
      .max_num_event_msg = 5,
      .async = {
          .client_event_callback = usb_host_client_event_callback,
          .callback_arg = nullptr,
      },
  };

  err = usb_host_client_register(&client_config, &usb_client_hdl);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register USB Host client: %s", esp_err_to_name(err));
    usb_host_uninstall();
    return;
  }

  // Start USB Host daemon task for continuous event processing
  // Stack size 8192 needed for device enumeration with logging
  BaseType_t task_created = xTaskCreatePinnedToCore(
      usb_host_daemon_task,
      "usb_daemon",
      8192,
      nullptr,
      5,  // Higher priority than main loop
      &usb_daemon_task_hdl,
      0   // Pin to core 0
  );
  if (task_created != pdTRUE) {
    ESP_LOGE(TAG, "Failed to create USB daemon task");
  }

  this->usb_host_client_hdl_ = usb_client_hdl;
  this->usb_host_initialized_ = true;

  // Initialize direct USB access for bulk OUT workaround
  direct_usb_init();

  // Request port power-on (daemon task will handle it after a delay)
  // This ensures enumeration happens after our client is registered
  port_power_requested = true;
  ESP_LOGI(TAG, "USB Host initialized");
}

void USBIPComponent::start_server_() {

  this->server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (this->server_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to create socket: %d", errno);
    return;
  }

  // Set socket options
  int opt = 1;
  setsockopt(this->server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  // Set non-blocking
  int flags = fcntl(this->server_fd_, F_GETFL, 0);
  fcntl(this->server_fd_, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(this->port_);

  if (bind(this->server_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind socket: %d", errno);
    close(this->server_fd_);
    this->server_fd_ = -1;
    return;
  }

  if (listen(this->server_fd_, 1) < 0) {
    ESP_LOGE(TAG, "Failed to listen: %d", errno);
    close(this->server_fd_);
    this->server_fd_ = -1;
    return;
  }

  ESP_LOGI(TAG, "USB/IP server listening on port %d", this->port_);
}

void USBIPComponent::loop() {
  // Handle device discovery request from daemon task
  // IMPORTANT: usb_host_device_open() must be called from main loop, not daemon task
  // ESP-IDF 5.5.x changed internal state management which causes crashes if called from daemon
  if (request_device_discovery && usb_client_hdl && usb_device_hdl == nullptr) {
    request_device_discovery = false;
    ESP_LOGD(TAG, "Processing device discovery request from main loop...");

    for (uint8_t addr = 1; addr <= 10; addr++) {
      usb_device_handle_t dev_hdl = nullptr;
      esp_err_t open_err = usb_host_device_open(usb_client_hdl, addr, &dev_hdl);
      if (open_err == ESP_OK && dev_hdl != nullptr) {
        ESP_LOGD(TAG, "Opened device at address %d, handle=%p", addr, dev_hdl);
        usb_device_hdl = dev_hdl;
        pending_dev_addr = addr;
        pending_dev_connect = true;
        break;
      } else if (open_err != ESP_ERR_NOT_FOUND) {
        ESP_LOGD(TAG, "Address %d: %s", addr, esp_err_to_name(open_err));
      }
    }
  }

  // Handle pending device connect/disconnect (set by callback or discovery)
  // Process disconnect FIRST to clean up old device before opening new one
  if (pending_dev_disconnect) {
    pending_dev_disconnect = false;
    ESP_LOGD(TAG, "Processing device disconnect");
    this->on_device_disconnected(0);
  }
  if (pending_dev_connect) {
    uint8_t addr = pending_dev_addr;
    pending_dev_connect = false;
    ESP_LOGD(TAG, "Processing device connect for address %d", addr);
    this->on_device_connected(addr);
  }

  // Handle TCP client connections
  if (this->server_fd_ >= 0) {
    // Check for new clients
    if (this->client_fd_ < 0) {
      struct sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      int new_client = accept(this->server_fd_, (struct sockaddr *)&client_addr, &client_len);
      if (new_client >= 0) {
        ESP_LOGI(TAG, "New USB/IP client connected");
        this->client_fd_ = new_client;
        this->client_state_ = CLIENT_STATE_IDLE;

        // Enable TCP_NODELAY for immediate packet transmission
        int opt = 1;
        setsockopt(this->client_fd_, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

        // Set client socket to non-blocking
        int flags = fcntl(this->client_fd_, F_GETFL, 0);
        fcntl(this->client_fd_, F_SETFL, flags | O_NONBLOCK);
      }
    }

    // Handle existing client
    if (this->client_fd_ >= 0) {
      this->handle_client_();
    }
  }
}

void USBIPComponent::usb_host_task_() {
  // Handle USB Host library events
  usb_host_lib_handle_events(0, nullptr);

  // Handle client events
  if (usb_client_hdl) {
    usb_host_client_handle_events(usb_client_hdl, 0);
  }
}

void USBIPComponent::on_device_connected(uint8_t dev_addr) {
  ESP_LOGD(TAG, "on_device_connected: addr=%d, usb_device_hdl=%p", dev_addr, usb_device_hdl);

  // Open the device if not already open
  if (usb_device_hdl == nullptr) {
    esp_err_t err = usb_host_device_open(usb_client_hdl, dev_addr, &usb_device_hdl);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to open USB device: %s", esp_err_to_name(err));
      return;
    }
    ESP_LOGD(TAG, "Device opened, handle=%p", usb_device_hdl);
  }

  this->usb_device_.connected = true;
  this->usb_device_.dev_addr = dev_addr;

  // Enumerate the device
  if (!this->enumerate_device_(dev_addr)) {
    ESP_LOGW(TAG, "Failed to enumerate device");
    return;
  }

  this->usb_device_.enumerated = true;
  ESP_LOGI(TAG, "Device enumerated: VID=%04X PID=%04X Class=%02X",
           this->usb_device_.vid, this->usb_device_.pid, this->usb_device_.device_class);

#ifdef HAS_PRINTER_FIRMWARE
  // HP Smart Install firmware upload flow
  if (this->usb_device_.vid == HP_VID) {
    // Phase 1: Smart Install mode (mass storage) → send SCSI 0xD0 to switch to printer
    if (this->has_mass_storage_interface_() && this->fw_state_ == FW_STATE_IDLE) {
      ESP_LOGI(TAG, "HP Smart Install detected — switching to printer mode");
      this->handle_hp_smart_install_();
      return;
    }

    // Phase 2: Printer mode without firmware → upload firmware
    if (this->has_printer_interface_() && this->fw_state_ == FW_STATE_AWAITING_PRINTER) {
      this->upload_hp_firmware_();
      return;
    }

    // Phase 3: Final re-enumeration after firmware upload → normal operation
    if (this->fw_state_ == FW_STATE_AWAITING_READY) {
      ESP_LOGD(TAG, "HP device ready after firmware upload");
      this->fw_state_ = FW_STATE_DONE;
    }
  }
#endif

  // Initialize printer device if it's a printer class
  this->init_printer_device_();
}

void USBIPComponent::on_device_disconnected(uint8_t dev_addr) {
  ESP_LOGI(TAG, "USB device disconnected");
#ifdef HAS_PRINTER_FIRMWARE
  bool in_fw_flow = (this->fw_state_ == FW_STATE_AWAITING_PRINTER ||
                     this->fw_state_ == FW_STATE_AWAITING_READY);
  ESP_LOGD(TAG, "fw_state=%d%s", this->fw_state_,
           in_fw_flow ? ", firmware flow in progress" : "");
#endif

  if (usb_device_hdl) {
    // Release all claimed interfaces
    for (uint8_t i = 0; i < this->usb_device_.num_interfaces && i < 4; i++) {
      uint8_t iface_num = this->usb_device_.interfaces[i].interface_number;
      usb_host_interface_release(usb_client_hdl, usb_device_hdl, iface_num);
    }
    usb_host_device_close(usb_client_hdl, usb_device_hdl);
    usb_device_hdl = nullptr;
  }

  // Reset printer/interface state
  printer_initialized = false;
  printer_interface_num = -1;
  interface_selected = false;
  toggle_reset_for_job = false;
  current_data_pid = USB_PID_DATA1;  // Reset to default for next connection

  this->usb_device_.connected = false;
  this->usb_device_.enumerated = false;
  this->usb_device_.dev_addr = 0;
  this->usb_device_.device_descriptor.clear();
  this->usb_device_.config_descriptor.clear();
  this->usb_device_.num_interfaces = 0;
  this->usb_device_.num_endpoints = 0;

#ifdef HAS_PRINTER_FIRMWARE
  // During firmware upload flow, don't reset fw_state_ or disconnect client
  // The device will re-enumerate and we need to continue the flow
  if (in_fw_flow) {
    ESP_LOGD(TAG, "Disconnect during firmware flow - preserving state for re-enumeration");
    return;
  }

  // Reset so a replug triggers firmware upload again (HP P1102 firmware is volatile)
  this->fw_state_ = FW_STATE_IDLE;
#endif

  // Close TCP connection so the Linux USB/IP client detects the disconnect
  if (this->client_state_ == CLIENT_STATE_ATTACHED) {
    this->disconnect_client_();
  }
}

bool USBIPComponent::enumerate_device_(uint8_t dev_addr) {
  if (!usb_device_hdl) {
    return false;
  }

  // Feed watchdog before potentially long operations
  esp_task_wdt_reset();

  // Give USB stack time to fully initialize device after open
  // ESP-IDF 5.5.x requires more settling time
  vTaskDelay(pdMS_TO_TICKS(100));
  esp_task_wdt_reset();

  // Get device descriptor
  const usb_device_desc_t *dev_desc;
  esp_err_t err = usb_host_get_device_descriptor(usb_device_hdl, &dev_desc);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get device descriptor: %s", esp_err_to_name(err));
    return false;
  }

  esp_task_wdt_reset();

  // Store device info
  this->usb_device_.vid = dev_desc->idVendor;
  this->usb_device_.pid = dev_desc->idProduct;
  this->usb_device_.bcd_device = dev_desc->bcdDevice;
  this->usb_device_.device_class = dev_desc->bDeviceClass;
  this->usb_device_.device_subclass = dev_desc->bDeviceSubClass;
  this->usb_device_.device_protocol = dev_desc->bDeviceProtocol;
  this->usb_device_.max_packet_size0 = dev_desc->bMaxPacketSize0;
  this->usb_device_.num_configurations = dev_desc->bNumConfigurations;

  // Store raw device descriptor
  this->usb_device_.device_descriptor.assign(
      reinterpret_cast<const uint8_t *>(dev_desc),
      reinterpret_cast<const uint8_t *>(dev_desc) + sizeof(usb_device_desc_t));

  ESP_LOGD(TAG, "Device: VID=%04X PID=%04X Class=%02X",
           this->usb_device_.vid, this->usb_device_.pid, this->usb_device_.device_class);

  // Allow USB events to be processed before getting config descriptor
  vTaskDelay(pdMS_TO_TICKS(50));
  esp_task_wdt_reset();

  // Get configuration descriptor
  // In ESP-IDF 5.4.0+, multiconfiguration support was added
  // usb_host_get_active_config_descriptor() may return nullptr if no config is active yet
  const usb_config_desc_t *config_desc = nullptr;
  err = usb_host_get_active_config_descriptor(usb_device_hdl, &config_desc);

  if (err == ESP_OK && config_desc == nullptr) {
    // No active configuration yet - try to get config descriptor by configuration value
    // Most devices use bConfigurationValue = 1 for first configuration
    ESP_LOGW(TAG, "No active config, trying to get config descriptor for config value 1");
    err = usb_host_get_config_desc(usb_client_hdl, usb_device_hdl, 1, &config_desc);
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get config descriptor: %s", esp_err_to_name(err));
    return false;
  }

  if (config_desc == nullptr) {
    ESP_LOGE(TAG, "Config descriptor is NULL even after fallback");
    return false;
  }

  esp_task_wdt_reset();

  // Validate config descriptor
  uint16_t total_length = config_desc->wTotalLength;
  if (total_length < sizeof(usb_config_desc_t) || total_length > 4096) {
    ESP_LOGE(TAG, "Invalid config descriptor length: %d", total_length);
    return false;
  }

  // Store raw config descriptor
  this->usb_device_.config_descriptor.assign(
      reinterpret_cast<const uint8_t *>(config_desc),
      reinterpret_cast<const uint8_t *>(config_desc) + total_length);

  this->usb_device_.num_interfaces = config_desc->bNumInterfaces;
  this->usb_device_.current_configuration = config_desc->bConfigurationValue;

  ESP_LOGD(TAG, "Config: %d interfaces, %d bytes",
           this->usb_device_.num_interfaces, total_length);

  esp_task_wdt_reset();

  // Parse interfaces and endpoints from config descriptor
  const uint8_t *desc_ptr = reinterpret_cast<const uint8_t *>(config_desc);
  size_t offset = config_desc->bLength;
  uint8_t interface_idx = 0;
  uint8_t endpoint_idx = 0;

  while (offset < total_length && interface_idx < 4 && endpoint_idx < 16) {
    uint8_t desc_len = desc_ptr[offset];
    uint8_t desc_type = desc_ptr[offset + 1];

    if (desc_len < 2) break;

    if (desc_type == USB_DESC_INTERFACE && desc_len >= 9) {
      this->usb_device_.interfaces[interface_idx].interface_number = desc_ptr[offset + 2];
      this->usb_device_.interfaces[interface_idx].interface_class = desc_ptr[offset + 5];
      this->usb_device_.interfaces[interface_idx].interface_subclass = desc_ptr[offset + 6];
      this->usb_device_.interfaces[interface_idx].interface_protocol = desc_ptr[offset + 7];

      ESP_LOGD(TAG, "  Interface %d: Class=%02X SubClass=%02X Protocol=%02X",
               desc_ptr[offset + 2],
               this->usb_device_.interfaces[interface_idx].interface_class,
               this->usb_device_.interfaces[interface_idx].interface_subclass,
               this->usb_device_.interfaces[interface_idx].interface_protocol);

      interface_idx++;
    } else if (desc_type == USB_DESC_ENDPOINT && desc_len >= 7) {
      this->usb_device_.endpoints[endpoint_idx].address = desc_ptr[offset + 2];
      this->usb_device_.endpoints[endpoint_idx].attributes = desc_ptr[offset + 3];
      this->usb_device_.endpoints[endpoint_idx].max_packet_size =
          desc_ptr[offset + 4] | (desc_ptr[offset + 5] << 8);
      this->usb_device_.endpoints[endpoint_idx].interval = desc_ptr[offset + 6];

      ESP_LOGD(TAG, "  Endpoint %02X: Attr=%02X MaxPacket=%d",
               this->usb_device_.endpoints[endpoint_idx].address,
               this->usb_device_.endpoints[endpoint_idx].attributes,
               this->usb_device_.endpoints[endpoint_idx].max_packet_size);

      endpoint_idx++;
    }

    offset += desc_len;
  }

  this->usb_device_.num_endpoints = endpoint_idx;

  // Determine device speed (convert ESP-IDF speed to Linux USB/IP speed values)
  usb_device_info_t dev_info;
  err = usb_host_device_info(usb_device_hdl, &dev_info);
  if (err == ESP_OK) {
    // ESP-IDF uses: USB_SPEED_LOW=0, USB_SPEED_FULL=1
    // Linux USB/IP uses: USB_SPEED_LOW=1, USB_SPEED_FULL=2, USB_SPEED_HIGH=3
    if (dev_info.speed == 0) {  // ESP-IDF USB_SPEED_LOW
      this->usb_device_.speed = USBIP_SPEED_LOW;
    } else {  // ESP-IDF USB_SPEED_FULL or higher
      this->usb_device_.speed = USBIP_SPEED_FULL;
    }
    ESP_LOGD(TAG, "Device speed: ESP-IDF=%d -> USBIP=%lu",
             dev_info.speed, (unsigned long)this->usb_device_.speed);
  }

  esp_task_wdt_reset();

  // If device has no active configuration, we need to set it first
  // This is required in ESP-IDF 5.4.0+ when using usb_host_get_config_desc()
  const usb_config_desc_t *active_config = nullptr;
  err = usb_host_get_active_config_descriptor(usb_device_hdl, &active_config);
  if (err == ESP_OK && active_config == nullptr) {
    ESP_LOGD(TAG, "Setting configuration %d...", this->usb_device_.current_configuration);

    // Send SET_CONFIGURATION control transfer
    usb_transfer_t *ctrl_transfer;
    err = usb_host_transfer_alloc(64, 0, &ctrl_transfer);
    if (err == ESP_OK) {
      ctrl_transfer->device_handle = usb_device_hdl;
      ctrl_transfer->bEndpointAddress = 0;
      ctrl_transfer->callback = usb_transfer_callback;
      ctrl_transfer->context = nullptr;
      ctrl_transfer->num_bytes = 8;  // Just setup packet, no data stage

      // SET_CONFIGURATION setup packet
      ctrl_transfer->data_buffer[0] = 0x00;  // bmRequestType: host-to-device, standard, device
      ctrl_transfer->data_buffer[1] = 0x09;  // bRequest: SET_CONFIGURATION
      ctrl_transfer->data_buffer[2] = this->usb_device_.current_configuration;  // wValue low
      ctrl_transfer->data_buffer[3] = 0x00;  // wValue high
      ctrl_transfer->data_buffer[4] = 0x00;  // wIndex low
      ctrl_transfer->data_buffer[5] = 0x00;  // wIndex high
      ctrl_transfer->data_buffer[6] = 0x00;  // wLength low
      ctrl_transfer->data_buffer[7] = 0x00;  // wLength high

      prepare_transfer(ctrl_transfer);
      err = usb_host_transfer_submit_control(usb_client_hdl, ctrl_transfer);
      if (err == ESP_OK) {
        wait_for_transfer(1000);
        if (transfer_status == USB_TRANSFER_STATUS_COMPLETED) {
          ESP_LOGD(TAG, "Configuration set successfully");
        } else {
          ESP_LOGW(TAG, "SET_CONFIGURATION failed: status=%d", transfer_status);
        }
      } else {
        ESP_LOGW(TAG, "SET_CONFIGURATION submit failed: %s", esp_err_to_name(err));
      }
      usb_host_transfer_free(ctrl_transfer);

      // Give device and USB stack time to configure
      // Process USB events actively to update library state
      for (int wait = 0; wait < 10; wait++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        uint32_t event_flags;
        usb_host_lib_handle_events(10, &event_flags);
        if (usb_client_hdl) {
          usb_host_client_handle_events(usb_client_hdl, 10);
        }
        esp_task_wdt_reset();
      }
    }
  }

  // Verify configuration is now active
  active_config = nullptr;
  err = usb_host_get_active_config_descriptor(usb_device_hdl, &active_config);
  if (active_config != nullptr) {
    ESP_LOGD(TAG, "Active configuration confirmed: %d interfaces", active_config->bNumInterfaces);
  } else {
    ESP_LOGW(TAG, "Library reports no active config (will try claiming anyway)");
  }

  // Always try to claim interfaces - we've sent SET_CONFIGURATION so the device should be ready
  // Even if the library doesn't track the active config, claiming might still work
  ESP_LOGD(TAG, "Claiming %d interfaces...", interface_idx);
  bool any_claimed = false;
  for (uint8_t i = 0; i < interface_idx && i < 4; i++) {
    uint8_t iface_num = this->usb_device_.interfaces[i].interface_number;
    err = usb_host_interface_claim(usb_client_hdl, usb_device_hdl, iface_num, 0);
    if (err == ESP_OK) {
      ESP_LOGD(TAG, "Claimed interface %d", iface_num);
      any_claimed = true;
    } else {
      ESP_LOGW(TAG, "Failed to claim interface %d: %s (0x%x)", iface_num, esp_err_to_name(err), err);
    }
    esp_task_wdt_reset();
  }

  if (!any_claimed) {
    ESP_LOGW(TAG, "No interfaces could be claimed - bulk transfers may fail");
  }

  return true;
}

#ifdef HAS_PRINTER_FIRMWARE
bool USBIPComponent::has_mass_storage_interface_() const {
  for (uint8_t i = 0; i < this->usb_device_.num_interfaces && i < 4; i++) {
    if (this->usb_device_.interfaces[i].interface_class == USB_CLASS_MASS_STORAGE) {
      return true;
    }
  }
  return false;
}

bool USBIPComponent::has_printer_interface_() const {
  for (uint8_t i = 0; i < this->usb_device_.num_interfaces && i < 4; i++) {
    if (this->usb_device_.interfaces[i].interface_class == USB_PRINTER_CLASS) {
      return true;
    }
  }
  return false;
}

bool USBIPComponent::handle_hp_smart_install_() {
  // Find bulk OUT and bulk IN endpoints on the mass storage interface
  uint8_t bulk_out_ep = 0;
  uint8_t bulk_in_ep = 0;
  for (uint8_t i = 0; i < this->usb_device_.num_endpoints && i < 16; i++) {
    uint8_t addr = this->usb_device_.endpoints[i].address;
    uint8_t attr = this->usb_device_.endpoints[i].attributes;
    if ((attr & 0x03) == 0x02) {  // Bulk transfer type
      if (addr & 0x80) {
        bulk_in_ep = addr;
      } else {
        bulk_out_ep = addr;
      }
    }
  }

  if (bulk_out_ep == 0 || bulk_in_ep == 0) {
    ESP_LOGE(TAG, "Mass storage endpoints not found (OUT=0x%02X IN=0x%02X)", bulk_out_ep, bulk_in_ep);
    return false;
  }
  ESP_LOGD(TAG, "Mass storage endpoints: OUT=0x%02X IN=0x%02X", bulk_out_ep, bulk_in_ep);

  // Allocate transfer buffer (64 bytes, enough for CBW=31 and CSW=13)
  usb_transfer_t *transfer = nullptr;
  esp_err_t err = usb_host_transfer_alloc(BULK_MPS, 0, &transfer);
  if (err != ESP_OK || !transfer) {
    ESP_LOGE(TAG, "Failed to allocate transfer: %s", esp_err_to_name(err));
    return false;
  }

  transfer->device_handle = usb_device_hdl;
  transfer->callback = usb_transfer_callback;
  transfer->timeout_ms = 5000;

  // Build USB Mass Storage CBW (Command Block Wrapper) - 31 bytes
  // SCSI vendor command 0xD0 triggers mode switch from Smart Install to printer
  memset(transfer->data_buffer, 0, BULK_MPS);
  transfer->data_buffer[0] = 0x55;  // dCBWSignature "USBC"
  transfer->data_buffer[1] = 0x53;
  transfer->data_buffer[2] = 0x42;
  transfer->data_buffer[3] = 0x43;
  transfer->data_buffer[4] = 0x78;  // dCBWTag (arbitrary)
  transfer->data_buffer[5] = 0x56;
  transfer->data_buffer[6] = 0x34;
  transfer->data_buffer[7] = 0x12;
  // bytes 8-11: dCBWDataTransferLength = 0
  // byte 12: bmCBWFlags = 0 (host-to-device)
  // byte 13: bCBWLUN = 0
  transfer->data_buffer[14] = 6;    // bCBWCBLength = 6
  transfer->data_buffer[15] = 0xD0; // SCSI vendor command

  transfer->num_bytes = 31;
  transfer->bEndpointAddress = bulk_out_ep;

  ESP_LOGD(TAG, "Sending CBW with SCSI command 0xD0...");
  prepare_transfer(transfer);
  err = usb_host_transfer_submit(transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "CBW submit failed: %s", esp_err_to_name(err));
    usb_host_transfer_free(transfer);
    return false;
  }

  if (!wait_for_transfer(5000)) {
    ESP_LOGE(TAG, "CBW transfer timeout");
    usb_host_transfer_free(transfer);
    return false;
  }

  if (transfer_status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "CBW transfer status: %d", transfer_status);
    // Some devices disconnect immediately without ACK - that's OK
    if (transfer_status != USB_TRANSFER_STATUS_NO_DEVICE) {
      usb_host_transfer_free(transfer);
      return false;
    }
  }
  ESP_LOGD(TAG, "CBW sent successfully");

  // Try to read CSW (Command Status Wrapper) - 13 bytes
  // The device may disconnect before responding, which is acceptable
  memset(transfer->data_buffer, 0, BULK_MPS);
  transfer->num_bytes = BULK_MPS;  // Must be multiple of MPS for bulk IN
  transfer->bEndpointAddress = bulk_in_ep;

  prepare_transfer(transfer);
  err = usb_host_transfer_submit(transfer);
  if (err == ESP_OK) {
    if (wait_for_transfer(3000)) {
      if (transfer_status == USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGD(TAG, "CSW received (%d bytes)", transfer_actual_bytes);
      } else {
        ESP_LOGD(TAG, "CSW status: %d (device may have disconnected)", transfer_status);
      }
    } else {
      ESP_LOGD(TAG, "CSW timeout (device may have disconnected already)");
    }
  } else {
    ESP_LOGD(TAG, "CSW submit: %s (device may have disconnected)", esp_err_to_name(err));
  }

  usb_host_transfer_free(transfer);

  this->fw_state_ = FW_STATE_AWAITING_PRINTER;
  ESP_LOGD(TAG, "Mode switch complete, waiting for re-enumeration as printer");
  return true;
}

bool USBIPComponent::upload_hp_firmware_() {
  ESP_LOGI(TAG, "Uploading firmware (%u bytes)...", (unsigned)printer_fw_data_size);

  this->fw_state_ = FW_STATE_UPLOADING;

  // Find bulk OUT endpoint on printer interface
  uint8_t bulk_out_ep = 0;
  for (uint8_t i = 0; i < this->usb_device_.num_endpoints && i < 16; i++) {
    uint8_t addr = this->usb_device_.endpoints[i].address;
    uint8_t attr = this->usb_device_.endpoints[i].attributes;
    if ((attr & 0x03) == 0x02 && !(addr & 0x80)) {  // Bulk OUT
      bulk_out_ep = addr;
      break;
    }
  }

  if (bulk_out_ep == 0) {
    ESP_LOGE(TAG, "No bulk OUT endpoint found on printer interface");
    this->fw_state_ = FW_STATE_IDLE;
    return false;
  }
  ESP_LOGD(TAG, "Using bulk OUT endpoint 0x%02X", bulk_out_ep);

  // Allocate single MPS-sized transfer for chunked sending
  usb_transfer_t *transfer = nullptr;
  esp_err_t err = usb_host_transfer_alloc(BULK_MPS, 0, &transfer);
  if (err != ESP_OK || !transfer) {
    ESP_LOGE(TAG, "Failed to allocate transfer: %s", esp_err_to_name(err));
    this->fw_state_ = FW_STATE_IDLE;
    return false;
  }

  transfer->device_handle = usb_device_hdl;
  transfer->bEndpointAddress = bulk_out_ep;
  transfer->callback = usb_transfer_callback;
  transfer->timeout_ms = 5000;

  size_t offset = 0;
  uint32_t last_progress = 0;
  uint32_t chunk_count = 0;
  uint32_t upload_start = millis();

  while (offset < printer_fw_data_size) {
    size_t chunk = printer_fw_data_size - offset;
    if (chunk > BULK_MPS) {
      chunk = BULK_MPS;
    }

    memcpy(transfer->data_buffer, printer_fw_data + offset, chunk);
    transfer->num_bytes = chunk;

    prepare_transfer(transfer);
    err = usb_host_transfer_submit(transfer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Firmware upload failed at offset %u: %s", (unsigned)offset, esp_err_to_name(err));
      usb_host_transfer_free(transfer);
      this->fw_state_ = FW_STATE_IDLE;
      return false;
    }

    // Tight polling with 0ms timeouts for speed (transfer completes in ~100μs on bus)
    uint32_t chunk_start = millis();
    while (!transfer_completed && (millis() - chunk_start) < 5000) {
      uint32_t event_flags;
      usb_host_lib_handle_events(0, &event_flags);
      if (usb_client_hdl) {
        usb_host_client_handle_events(usb_client_hdl, 0);
      }
    }

    if (!transfer_completed) {
      ESP_LOGE(TAG, "Firmware upload timeout at offset %u", (unsigned)offset);
      usb_host_transfer_free(transfer);
      this->fw_state_ = FW_STATE_IDLE;
      return false;
    }

    if (transfer_status != USB_TRANSFER_STATUS_COMPLETED) {
      // Device may disconnect near the end after receiving all data
      if (transfer_status == USB_TRANSFER_STATUS_NO_DEVICE && offset > printer_fw_data_size * 9 / 10) {
        ESP_LOGI(TAG, "Device disconnected near end of upload (offset %u/%u) - likely success",
                 (unsigned)offset, (unsigned)printer_fw_data_size);
        break;
      }
      ESP_LOGE(TAG, "Firmware upload error at offset %u: status=%d", (unsigned)offset, transfer_status);
      usb_host_transfer_free(transfer);
      this->fw_state_ = FW_STATE_IDLE;
      return false;
    }

    offset += chunk;
    chunk_count++;

    // Log progress every ~10%
    uint32_t progress = (offset * 100) / printer_fw_data_size;
    if (progress / 10 > last_progress / 10) {
      ESP_LOGI(TAG, "Firmware upload: %lu%% (%u/%u bytes)",
               (unsigned long)progress, (unsigned)offset, (unsigned)printer_fw_data_size);
      last_progress = progress;
    }

    // Feed WDT every 500 chunks (~32KB) to avoid tight-loop starve
    if (chunk_count % 500 == 0) {
      esp_task_wdt_reset();
    }
  }

  uint32_t elapsed = millis() - upload_start;
  ESP_LOGI(TAG, "Firmware upload complete (%lums, %lu KB/s)",
           (unsigned long)elapsed,
           elapsed > 0 ? (unsigned long)(offset / elapsed) : 0);

  usb_host_transfer_free(transfer);

  this->fw_state_ = FW_STATE_AWAITING_READY;
  return true;
}
#endif  // HAS_PRINTER_FIRMWARE

void USBIPComponent::init_printer_device_() {
  // Reset state
  printer_initialized = false;
  printer_interface_num = -1;

  // Find printer interface (class 0x07)
  int printer_interface = -1;
  for (uint8_t i = 0; i < this->usb_device_.num_interfaces && i < 4; i++) {
    if (this->usb_device_.interfaces[i].interface_class == USB_PRINTER_CLASS) {
      printer_interface = this->usb_device_.interfaces[i].interface_number;
      ESP_LOGD(TAG, "Found printer interface %d (subclass=%d, protocol=%d)",
               printer_interface,
               this->usb_device_.interfaces[i].interface_subclass,
               this->usb_device_.interfaces[i].interface_protocol);
      break;
    }
  }

  if (printer_interface < 0) {
    ESP_LOGD(TAG, "No printer interface found, skipping printer init");
    return;
  }

  printer_interface_num = printer_interface;

  // Allocate transfer for printer class requests
  usb_transfer_t *transfer;
  esp_err_t err = usb_host_transfer_alloc(64, 0, &transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to allocate transfer for printer init: %s", esp_err_to_name(err));
    return;
  }

  // Skip SOFT_RESET - Linux usblp doesn't send it during normal open.
  // SOFT_RESET can put the printer in a state where it NAKs OUT transfers.

  transfer->device_handle = usb_device_hdl;
  transfer->bEndpointAddress = 0;
  transfer->callback = usb_transfer_callback;
  transfer->context = nullptr;
  transfer->timeout_ms = 2000;

  // Step 1: GET_PORT_STATUS to check printer is ready
  // bmRequestType = 0xA1 (device-to-host, class, interface)
  // bRequest = 0x01 (GET_PORT_STATUS)
  ESP_LOGD(TAG, "Getting printer port status...");

  transfer->num_bytes = 8 + 1;  // setup + 1 byte response
  transfer->data_buffer[0] = 0xA1;  // bmRequestType: class, interface, device-to-host
  transfer->data_buffer[1] = PRINTER_REQ_GET_PORT_STATUS;  // bRequest
  transfer->data_buffer[2] = 0x00;  // wValue low
  transfer->data_buffer[3] = 0x00;  // wValue high
  transfer->data_buffer[4] = printer_interface & 0xFF;  // wIndex low = interface
  transfer->data_buffer[5] = 0x00;  // wIndex high
  transfer->data_buffer[6] = 0x01;  // wLength low = 1 byte
  transfer->data_buffer[7] = 0x00;  // wLength high

  prepare_transfer(transfer);

  err = usb_host_transfer_submit_control(usb_client_hdl, transfer);
  if (err == ESP_OK) {
    wait_for_transfer(1000);
    if (transfer_status == USB_TRANSFER_STATUS_COMPLETED && transfer_actual_bytes > 8) {
      uint8_t port_status = transfer->data_buffer[8];
      // Bit 5: Paper Empty (0=OK), Bit 4: Selected (1=OK), Bit 3: Not Error (1=OK)
      bool paper_ok = !(port_status & 0x20);
      bool selected = (port_status & 0x10) != 0;
      bool no_error = (port_status & 0x08) != 0;
      ESP_LOGI(TAG, "Printer ready (interface %d, paper=%s, error=%s)",
               printer_interface,
               paper_ok ? "OK" : "EMPTY",
               no_error ? "NONE" : "ERROR");
    } else {
      ESP_LOGW(TAG, "GET_PORT_STATUS failed: status=%d actual=%d", transfer_status, transfer_actual_bytes);
    }
  } else {
    ESP_LOGW(TAG, "GET_PORT_STATUS submit failed: %s", esp_err_to_name(err));
  }

  // Step 3: Send SET_INTERFACE(0, 0) to reset data toggles on device
  // USB 2.0 spec 9.1.1.5: SET_INTERFACE resets data toggles to DATA0
  // This is what the Linux usblp driver does when opening the device
  ESP_LOGD(TAG, "Sending SET_INTERFACE(0, 0) to reset data toggles");

  transfer->num_bytes = 8;
  transfer->data_buffer[0] = 0x01;  // bmRequestType: host-to-device, standard, interface
  transfer->data_buffer[1] = USB_REQ_SET_INTERFACE;  // bRequest = 11
  transfer->data_buffer[2] = 0x00;  // wValue low = alt setting 0
  transfer->data_buffer[3] = 0x00;  // wValue high
  transfer->data_buffer[4] = printer_interface & 0xFF;  // wIndex low = interface
  transfer->data_buffer[5] = 0x00;  // wIndex high
  transfer->data_buffer[6] = 0x00;  // wLength low
  transfer->data_buffer[7] = 0x00;  // wLength high

  prepare_transfer(transfer);

  err = usb_host_transfer_submit_control(usb_client_hdl, transfer);
  if (err == ESP_OK) {
    wait_for_transfer(1000);
    if (transfer_status == USB_TRANSFER_STATUS_COMPLETED) {
      ESP_LOGD(TAG, "SET_INTERFACE(0, 0) completed successfully");
    } else {
      ESP_LOGW(TAG, "SET_INTERFACE(0, 0) failed: status=%d", transfer_status);
    }
  } else {
    ESP_LOGW(TAG, "SET_INTERFACE submit failed: %s", esp_err_to_name(err));
  }

  usb_host_transfer_free(transfer);

  // Step 4: Release and re-claim interfaces to reset ESP-IDF's internal
  // data toggle tracking (ensures host-side toggles start at DATA0)
  ESP_LOGD(TAG, "Re-claiming interfaces to reset toggle state");
  for (uint8_t i = 0; i < this->usb_device_.num_interfaces && i < 4; i++) {
    uint8_t iface_num = this->usb_device_.interfaces[i].interface_number;
    err = usb_host_interface_release(usb_client_hdl, usb_device_hdl, iface_num);
    if (err == ESP_OK) {
      err = usb_host_interface_claim(usb_client_hdl, usb_device_hdl, iface_num, 0);
      if (err == ESP_OK) {
        ESP_LOGD(TAG, "Interface %d re-claimed successfully", iface_num);
      } else {
        ESP_LOGW(TAG, "Interface %d re-claim failed: %s", iface_num, esp_err_to_name(err));
      }
    } else {
      ESP_LOGW(TAG, "Interface %d release failed: %s", iface_num, esp_err_to_name(err));
    }
  }

  printer_initialized = true;
}

void USBIPComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "USB/IP Component:");
  ESP_LOGCONFIG(TAG, "  Port: %d", this->port_);
#ifdef HAS_PRINTER_FIRMWARE
  ESP_LOGCONFIG(TAG, "  Printer firmware: %u bytes", (unsigned)printer_fw_data_size);
#endif
  ESP_LOGCONFIG(TAG, "  USB Host initialized: %s", this->usb_host_initialized_ ? "yes" : "no");
  if (this->usb_device_.connected) {
    ESP_LOGCONFIG(TAG, "  USB Device: VID=%04X PID=%04X Class=%02X",
                  this->usb_device_.vid, this->usb_device_.pid, this->usb_device_.device_class);
    ESP_LOGCONFIG(TAG, "  Device enumerated: %s", this->usb_device_.enumerated ? "yes" : "no");
  } else {
    ESP_LOGCONFIG(TAG, "  USB Device: not connected");
  }
}

void USBIPComponent::handle_client_() {
  // Check if data is available
  uint8_t peek;
  int result = recv(this->client_fd_, &peek, 1, MSG_PEEK | MSG_DONTWAIT);
  if (result == 0) {
    // Connection closed
    ESP_LOGI(TAG, "Client disconnected");
    this->disconnect_client_();
    return;
  }
  if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    ESP_LOGW(TAG, "Socket error: %d", errno);
    this->disconnect_client_();
    return;
  }
  if (result < 0) {
    return;  // No data available
  }

  // After IMPORT, the protocol switches to URB format (different header)
  if (this->client_state_ == CLIENT_STATE_ATTACHED) {
    // URB header format: command(4) + seqnum(4) + devid(4) + direction(4) + ep(4) = 20 bytes minimum
    // Full USBIP_CMD_SUBMIT header is 48 bytes
    this->handle_urb_();
    return;
  }

  // Operation command header format: version(2) + command(2) + status(4) = 8 bytes
  USBIPHeader header;
  if (!this->read_bytes_(reinterpret_cast<uint8_t *>(&header), sizeof(header))) {
    ESP_LOGW(TAG, "Failed to read USB/IP header");
    this->disconnect_client_();
    return;
  }

  uint16_t version = ntohs_(header.version);
  uint16_t command = ntohs_(header.command);

  ESP_LOGD(TAG, "USB/IP command: version=0x%04X, command=0x%04X", version, command);

  switch (command) {
    case OP_REQ_DEVLIST:
      ESP_LOGD(TAG, "DEVLIST request");
      this->handle_devlist_request_();
      break;

    case OP_REQ_IMPORT:
      ESP_LOGD(TAG, "IMPORT request");
      this->handle_import_request_();
      break;

    default:
      ESP_LOGW(TAG, "Unknown USB/IP command: 0x%04X", command);
      break;
  }
}

bool USBIPComponent::handle_devlist_request_() {
  // Note: The status field is already part of the header we read, no additional data to read
  this->send_devlist_response_();
  return true;
}

void USBIPComponent::send_devlist_response_() {
  bool has_device = this->usb_device_.connected && this->usb_device_.enumerated;

  ESP_LOGD(TAG, "Preparing DEVLIST response: device=%s", has_device ? "yes" : "no");

  // Send header
  USBIPHeader header;
  header.version = htons_(USBIP_VERSION);
  header.command = htons_(OP_REP_DEVLIST);
  header.status = htonl_(ST_OK);

  if (!this->write_bytes_(reinterpret_cast<uint8_t *>(&header), sizeof(header))) {
    ESP_LOGE(TAG, "Failed to send DEVLIST header");
    return;
  }

  // Send device count
  uint32_t device_count = has_device ? htonl_(1) : htonl_(0);
  if (!this->write_bytes_(reinterpret_cast<uint8_t *>(&device_count), sizeof(device_count))) {
    ESP_LOGE(TAG, "Failed to send device count");
    return;
  }

  if (has_device) {
    // Send device info
    USBIPDeviceInfo dev_info;
    memset(&dev_info, 0, sizeof(dev_info));

    strncpy(dev_info.path, "/sys/devices/pci0000:00/0000:00:01.2/usb1/1-1", sizeof(dev_info.path) - 1);
    strncpy(dev_info.busid, "1-1", sizeof(dev_info.busid) - 1);
    dev_info.busnum = htonl_(1);
    dev_info.devnum = htonl_(this->usb_device_.dev_addr);
    dev_info.speed = htonl_(this->usb_device_.speed);
    dev_info.idVendor = htons_(this->usb_device_.vid);
    dev_info.idProduct = htons_(this->usb_device_.pid);
    dev_info.bcdDevice = htons_(this->usb_device_.bcd_device);
    dev_info.bDeviceClass = this->usb_device_.device_class;
    dev_info.bDeviceSubClass = this->usb_device_.device_subclass;
    dev_info.bDeviceProtocol = this->usb_device_.device_protocol;
    dev_info.bConfigurationValue = this->usb_device_.current_configuration;
    dev_info.bNumConfigurations = this->usb_device_.num_configurations;
    dev_info.bNumInterfaces = this->usb_device_.num_interfaces;

    ESP_LOGD(TAG, "Sending device: VID=%04X PID=%04X interfaces=%d",
             this->usb_device_.vid, this->usb_device_.pid, this->usb_device_.num_interfaces);

    if (!this->write_bytes_(reinterpret_cast<uint8_t *>(&dev_info), sizeof(dev_info))) {
      ESP_LOGE(TAG, "Failed to send device info");
      return;
    }

    // Send interface info for each interface
    for (uint8_t i = 0; i < this->usb_device_.num_interfaces && i < 4; i++) {
      USBIPInterfaceInfo iface_info;
      iface_info.bInterfaceClass = this->usb_device_.interfaces[i].interface_class;
      iface_info.bInterfaceSubClass = this->usb_device_.interfaces[i].interface_subclass;
      iface_info.bInterfaceProtocol = this->usb_device_.interfaces[i].interface_protocol;
      iface_info.padding = 0;

      ESP_LOGD(TAG, "  Interface %d: class=%02X", i, iface_info.bInterfaceClass);

      if (!this->write_bytes_(reinterpret_cast<uint8_t *>(&iface_info), sizeof(iface_info))) {
        ESP_LOGE(TAG, "Failed to send interface info");
        return;
      }
    }
  }

  ESP_LOGD(TAG, "Sent DEVLIST response: %d device(s)", has_device ? 1 : 0);
}

bool USBIPComponent::handle_import_request_() {
  char busid[32];
  memset(busid, 0, sizeof(busid));
  if (!this->read_bytes_(reinterpret_cast<uint8_t *>(busid), sizeof(busid))) {
    ESP_LOGE(TAG, "Failed to read busid");
    return false;
  }

  // Ensure null termination
  busid[31] = '\0';
  ESP_LOGD(TAG, "Import request for bus ID: '%s'", busid);

  bool has_device = this->usb_device_.connected && this->usb_device_.enumerated;
  bool success = has_device && (strcmp(busid, "1-1") == 0);

  ESP_LOGD(TAG, "Import: has_device=%d, busid_match=%d, success=%d",
           has_device, strcmp(busid, "1-1") == 0, success);

  this->send_import_response_(success);

  if (success) {
    this->client_state_ = CLIENT_STATE_ATTACHED;
    ESP_LOGI(TAG, "Device attached to client, waiting for URBs");
  } else {
    ESP_LOGW(TAG, "Import failed: device=%d, busid='%s'", has_device, busid);
  }

  return true;
}

void USBIPComponent::send_import_response_(bool success) {
  ESP_LOGD(TAG, "Sending IMPORT response: success=%d", success);

  USBIPHeader header;
  header.version = htons_(USBIP_VERSION);
  header.command = htons_(OP_REP_IMPORT);
  header.status = htonl_(success ? ST_OK : ST_NA);

  ESP_LOGD(TAG, "IMPORT header: version=0x%04X cmd=0x%04X status=0x%08lX",
           ntohs_(header.version), ntohs_(header.command), (unsigned long)ntohl_(header.status));

  if (!this->write_bytes_(reinterpret_cast<uint8_t *>(&header), sizeof(header))) {
    ESP_LOGE(TAG, "Failed to send IMPORT header");
    return;
  }

  if (success) {
    USBIPDeviceInfo dev_info;
    memset(&dev_info, 0, sizeof(dev_info));

    strncpy(dev_info.path, "/sys/devices/pci0000:00/0000:00:01.2/usb1/1-1", sizeof(dev_info.path) - 1);
    strncpy(dev_info.busid, "1-1", sizeof(dev_info.busid) - 1);
    dev_info.busnum = htonl_(1);
    dev_info.devnum = htonl_(this->usb_device_.dev_addr);
    dev_info.speed = htonl_(this->usb_device_.speed);
    dev_info.idVendor = htons_(this->usb_device_.vid);
    dev_info.idProduct = htons_(this->usb_device_.pid);
    dev_info.bcdDevice = htons_(this->usb_device_.bcd_device);
    dev_info.bDeviceClass = this->usb_device_.device_class;
    dev_info.bDeviceSubClass = this->usb_device_.device_subclass;
    dev_info.bDeviceProtocol = this->usb_device_.device_protocol;
    dev_info.bConfigurationValue = this->usb_device_.current_configuration;
    dev_info.bNumConfigurations = this->usb_device_.num_configurations;
    dev_info.bNumInterfaces = this->usb_device_.num_interfaces;

    ESP_LOGD(TAG, "IMPORT device: VID=%04X PID=%04X speed=%lu devnum=%d config=%d interfaces=%d",
             this->usb_device_.vid, this->usb_device_.pid,
             (unsigned long)this->usb_device_.speed, this->usb_device_.dev_addr,
             this->usb_device_.current_configuration, this->usb_device_.num_interfaces);
    ESP_LOGD(TAG, "IMPORT device info size: %zu bytes", sizeof(dev_info));

    if (!this->write_bytes_(reinterpret_cast<uint8_t *>(&dev_info), sizeof(dev_info))) {
      ESP_LOGE(TAG, "Failed to send IMPORT device info");
      return;
    }

    ESP_LOGD(TAG, "IMPORT response sent (%zu + %zu bytes)",
             sizeof(header), sizeof(dev_info));
  } else {
    ESP_LOGD(TAG, "IMPORT response sent (failure)");
  }
}

bool USBIPComponent::handle_urb_() {
  // URB header format (48 bytes total):
  // command(4) + seqnum(4) + devid(4) + direction(4) + ep(4) +
  // transfer_flags(4) + transfer_buffer_length(4) + start_frame(4) +
  // number_of_packets(4) + interval(4) + setup[8]
  uint8_t header[48];
  if (!this->read_bytes_(header, sizeof(header))) {
    ESP_LOGW(TAG, "Failed to read URB header");
    return false;
  }

  uint32_t command = ntohl_(*reinterpret_cast<uint32_t *>(&header[0]));
  uint32_t seqnum = ntohl_(*reinterpret_cast<uint32_t *>(&header[4]));
  uint32_t devid = ntohl_(*reinterpret_cast<uint32_t *>(&header[8]));
  uint32_t direction = ntohl_(*reinterpret_cast<uint32_t *>(&header[12]));
  uint32_t ep = ntohl_(*reinterpret_cast<uint32_t *>(&header[16]));
  uint32_t transfer_flags = ntohl_(*reinterpret_cast<uint32_t *>(&header[20]));
  uint32_t transfer_buffer_length = ntohl_(*reinterpret_cast<uint32_t *>(&header[24]));
  uint8_t *setup = &header[40];

  const char *dir_str = direction ? "IN" : "OUT";
  const char *ep_type = (ep == 0) ? "ctrl" : "bulk";
  ESP_LOGD(TAG, "URB: seq=%lu ep%lu %s %s len=%lu",
           (unsigned long)seqnum, (unsigned long)ep, ep_type, dir_str,
           (unsigned long)transfer_buffer_length);

  // Handle UNLINK command
  if (command == USBIP_CMD_UNLINK) {
    ESP_LOGD(TAG, "UNLINK request for seqnum %lu", (unsigned long)seqnum);
    USBIPRetUnlink response;
    memset(&response, 0, sizeof(response));
    response.command = htonl_(USBIP_RET_UNLINK);
    response.seqnum = htonl_(seqnum);
    response.status = htonl_(0);  // -ECONNRESET
    this->write_bytes_(reinterpret_cast<uint8_t *>(&response), sizeof(response));
    return true;
  }

  if (command != USBIP_CMD_SUBMIT) {
    ESP_LOGW(TAG, "Unknown URB command: 0x%04lX", (unsigned long)command);
    return false;
  }

  (void)devid;
  (void)transfer_flags;

  // Read transfer buffer if OUT transfer
  std::vector<uint8_t> buffer;
  if (direction == 0 && transfer_buffer_length > 0) {
    buffer.resize(transfer_buffer_length);
    if (!this->read_bytes_(buffer.data(), transfer_buffer_length)) {
      ESP_LOGW(TAG, "Failed to read transfer buffer");
      return false;
    }
  }

  if (ep == 0) {
    return this->handle_control_transfer_(seqnum, setup, transfer_buffer_length, buffer.data());
  } else {
    return this->handle_bulk_transfer_(seqnum, ep, direction, buffer.data(), transfer_buffer_length);
  }
}

bool USBIPComponent::handle_control_transfer_(uint32_t seqnum, const uint8_t *setup_data,
                                               uint32_t data_length, const uint8_t *out_data) {
  uint8_t bmRequestType = setup_data[0];
  uint8_t bRequest = setup_data[1];
  uint16_t wValue = setup_data[2] | (setup_data[3] << 8);
  uint16_t wIndex = setup_data[4] | (setup_data[5] << 8);
  uint16_t wLength = setup_data[6] | (setup_data[7] << 8);

  // Human-readable control transfer logging
  const char *req_name = "UNKNOWN";
  if (!(bmRequestType & 0x60)) {
    // Standard requests
    switch (bRequest) {
      case USB_REQ_GET_STATUS: req_name = "GET_STATUS"; break;
      case USB_REQ_CLEAR_FEATURE: req_name = "CLEAR_FEATURE"; break;
      case USB_REQ_SET_FEATURE: req_name = "SET_FEATURE"; break;
      case USB_REQ_SET_ADDRESS: req_name = "SET_ADDRESS"; break;
      case USB_REQ_GET_DESCRIPTOR: {
        uint8_t dt = wValue >> 8;
        if (dt == USB_DESC_DEVICE) req_name = "GET_DESCRIPTOR(DEVICE)";
        else if (dt == USB_DESC_CONFIGURATION) req_name = "GET_DESCRIPTOR(CONFIG)";
        else if (dt == USB_DESC_STRING) req_name = "GET_DESCRIPTOR(STRING)";
        else req_name = "GET_DESCRIPTOR(OTHER)";
        break;
      }
      case USB_REQ_SET_DESCRIPTOR: req_name = "SET_DESCRIPTOR"; break;
      case USB_REQ_GET_CONFIGURATION: req_name = "GET_CONFIGURATION"; break;
      case USB_REQ_SET_CONFIGURATION: req_name = "SET_CONFIGURATION"; break;
      case USB_REQ_GET_INTERFACE: req_name = "GET_INTERFACE"; break;
      case USB_REQ_SET_INTERFACE: req_name = "SET_INTERFACE"; break;
    }
  } else if ((bmRequestType & 0x60) == 0x20) {
    // Class requests
    if ((bmRequestType & 0x1F) == 0x01) {
      // Interface class request
      switch (bRequest) {
        case PRINTER_REQ_GET_DEVICE_ID: req_name = "PRINTER:GET_DEVICE_ID"; break;
        case PRINTER_REQ_GET_PORT_STATUS: req_name = "PRINTER:GET_PORT_STATUS"; break;
        case PRINTER_REQ_SOFT_RESET: req_name = "PRINTER:SOFT_RESET"; break;
        default: req_name = "CLASS_REQUEST"; break;
      }
    } else {
      req_name = "CLASS_REQUEST";
    }
  } else if ((bmRequestType & 0x60) == 0x40) {
    req_name = "VENDOR_REQUEST";
  }
  ESP_LOGD(TAG, "Control: %s bmReqType=0x%02X bReq=0x%02X wVal=0x%04X wIdx=0x%04X wLen=%d",
           req_name, bmRequestType, bRequest, wValue, wIndex, wLength);

  std::vector<uint8_t> response_data;
  int32_t status = 0;

  if (!this->usb_device_.connected || !this->usb_device_.enumerated) {
    this->send_urb_response_(seqnum, -19, nullptr, 0);  // -ENODEV
    return false;
  }

  // For GET_DESCRIPTOR requests, try to use cached descriptors first
  bool use_cached = false;
  if ((bmRequestType & 0x80) && bRequest == USB_REQ_GET_DESCRIPTOR) {
    uint8_t desc_type = wValue >> 8;

    if (desc_type == USB_DESC_DEVICE && !this->usb_device_.device_descriptor.empty()) {
      response_data = this->usb_device_.device_descriptor;
      if (response_data.size() > wLength) {
        response_data.resize(wLength);
      }
      use_cached = true;
      ESP_LOGD(TAG, "Using cached device descriptor (%zu bytes)", response_data.size());
    } else if (desc_type == USB_DESC_CONFIGURATION && !this->usb_device_.config_descriptor.empty()) {
      response_data = this->usb_device_.config_descriptor;
      if (response_data.size() > wLength) {
        response_data.resize(wLength);
      }
      use_cached = true;
      ESP_LOGD(TAG, "Using cached config descriptor (%zu bytes)", response_data.size());
    }
  }

  // Intercept SET_CONFIGURATION - ESP-IDF already configured the device
  // Forwarding it again can cause issues with endpoint state
  if (bRequest == USB_REQ_SET_CONFIGURATION && !(bmRequestType & 0x80)) {
    uint8_t config_val = wValue & 0xFF;
    ESP_LOGD(TAG, "Intercepted SET_CONFIGURATION(%d) - already configured", config_val);
    // Return success without actually sending to device
    this->send_urb_response_(seqnum, 0, nullptr, 0);
    return true;
  }

  // Intercept CLEAR_FEATURE(ENDPOINT_HALT) - also clear ESP-IDF's internal endpoint state
  // Without this, ESP-IDF rejects subsequent transfers with ESP_ERR_INVALID_STATE
  if (bRequest == USB_REQ_CLEAR_FEATURE && !(bmRequestType & 0x80) &&
      (bmRequestType & 0x1F) == 0x02 && wValue == 0x0000) {
    uint8_t target_ep = wIndex & 0xFF;
    ESP_LOGD(TAG, "CLEAR_FEATURE(ENDPOINT_HALT) on ep 0x%02X", target_ep);
    // Clear ESP-IDF's internal endpoint state
    usb_host_endpoint_flush(usb_device_hdl, target_ep);
    usb_host_client_handle_events(usb_client_hdl, 50);
    usb_host_endpoint_clear(usb_device_hdl, target_ep);
    // Still forward to device below
  }

  if (!use_cached && usb_device_hdl) {
    // Perform actual USB control transfer
    usb_transfer_t *transfer;
    size_t alloc_size = wLength > 0 ? wLength + 8 : 64;
    esp_err_t err = usb_host_transfer_alloc(alloc_size, 0, &transfer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to allocate transfer: %s", esp_err_to_name(err));
      this->send_urb_response_(seqnum, -12, nullptr, 0);  // -ENOMEM
      return false;
    }

    // Build setup packet
    transfer->device_handle = usb_device_hdl;
    transfer->bEndpointAddress = 0;
    transfer->callback = usb_transfer_callback;
    transfer->context = nullptr;
    transfer->num_bytes = wLength + 8;
    transfer->timeout_ms = 1000;

    // Copy setup packet
    memcpy(transfer->data_buffer, setup_data, 8);

    // Copy OUT data if present
    if (!(bmRequestType & 0x80) && out_data && wLength > 0) {
      memcpy(transfer->data_buffer + 8, out_data, wLength);
    }

    // Prepare transfer with unique sequence number
    uint32_t seq = prepare_transfer(transfer);
    ESP_LOGV(TAG, "Control transfer seq=%lu", (unsigned long)seq);

    // Submit control transfer
    err = usb_host_transfer_submit_control(usb_client_hdl, transfer);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Control transfer submit failed: %s", esp_err_to_name(err));
      usb_host_transfer_free(transfer);
      status = -32;  // -EPIPE
    } else {
      // Wait for transfer completion while processing USB events
      if (wait_for_transfer(1000)) {
        if (transfer_status == USB_TRANSFER_STATUS_COMPLETED) {
          if (bmRequestType & 0x80) {
            // IN transfer - copy response data
            size_t data_len = transfer_actual_bytes > 8 ? transfer_actual_bytes - 8 : 0;
            if (data_len > 0) {
              response_data.assign(transfer->data_buffer + 8,
                                   transfer->data_buffer + 8 + data_len);
            }
          }
          ESP_LOGD(TAG, "Control transfer completed, %d bytes", transfer_actual_bytes);

          // If this was SET_CONFIGURATION, re-claim interfaces
          // SET_CONFIGURATION resets device state, so we need to re-claim
          if (bRequest == USB_REQ_SET_CONFIGURATION) {
            ESP_LOGD(TAG, "SET_CONFIGURATION completed, re-claiming interfaces");
            vTaskDelay(pdMS_TO_TICKS(50));

            // Re-claim all interfaces
            for (uint8_t i = 0; i < this->usb_device_.num_interfaces && i < 4; i++) {
              uint8_t iface_num = this->usb_device_.interfaces[i].interface_number;
              // First try to release (ignore errors)
              usb_host_interface_release(usb_client_hdl, usb_device_hdl, iface_num);
              // Then re-claim
              esp_err_t claim_err = usb_host_interface_claim(usb_client_hdl, usb_device_hdl, iface_num, 0);
              if (claim_err == ESP_OK) {
                ESP_LOGD(TAG, "Re-claimed interface %d", iface_num);
              } else {
                ESP_LOGW(TAG, "Failed to re-claim interface %d: %s", iface_num, esp_err_to_name(claim_err));
              }
            }
          }
        } else {
          ESP_LOGW(TAG, "Control transfer status: %d", transfer_status);
          status = -32;  // -EPIPE
        }
      } else {
        ESP_LOGW(TAG, "Control transfer timeout");
        status = -110;  // -ETIMEDOUT
      }
      usb_host_transfer_free(transfer);
    }
  }

  // Log control transfer result
  if (status != 0) {
    ESP_LOGW(TAG, "Control %s failed: status=%ld", req_name, (long)status);
  } else if (!response_data.empty()) {
    // Log response data preview for class requests
    if ((bmRequestType & 0x60) == 0x20 && response_data.size() > 0) {
      char hex[96];
      int hex_len = 0;
      for (size_t i = 0; i < response_data.size() && i < 32 && hex_len < 90; i++) {
        hex_len += snprintf(hex + hex_len, sizeof(hex) - hex_len, "%02X ", response_data[i]);
      }
      ESP_LOGD(TAG, "Control %s response (%zu bytes): %s", req_name, response_data.size(), hex);
    }
    // Log GET_PORT_STATUS specifically
    if (bRequest == PRINTER_REQ_GET_PORT_STATUS && (bmRequestType & 0x60) == 0x20 && response_data.size() >= 1) {
      uint8_t ps = response_data[0];
      ESP_LOGI(TAG, "Printer port status: 0x%02X paper=%s selected=%s error=%s",
               ps, (ps & 0x20) ? "EMPTY" : "OK", (ps & 0x10) ? "YES" : "NO", (ps & 0x08) ? "NONE" : "ERROR");
    }
  }

  this->send_urb_response_(seqnum, status, response_data.data(), response_data.size());
  return true;
}

bool USBIPComponent::handle_bulk_transfer_(uint32_t seqnum, uint32_t ep, uint32_t direction,
                                            const uint8_t *data, uint32_t length) {
  if (!this->usb_device_.connected || !this->usb_device_.enumerated || !usb_device_hdl) {
    this->send_urb_response_(seqnum, -19, nullptr, 0);  // -ENODEV
    return false;
  }

  // Print job progress tracking
  static uint32_t job_bytes_out = 0;
  static uint32_t job_bytes_in = 0;
  static uint32_t job_transfers_out = 0;
  static uint32_t job_transfers_in = 0;
  static uint32_t job_start_time = 0;
  static uint32_t last_progress_log = 0;
  static bool job_active = false;

  static uint32_t last_transfer_time = 0;

  // Detect job idle (>5s gap between transfers) and reset
  if (job_active && (millis() - last_transfer_time) > 5000) {
    uint32_t elapsed_s = (last_transfer_time - job_start_time) / 1000;
    ESP_LOGI(TAG, "Print job ended (idle): %lu bytes OUT, %lu bytes IN, %lu OUT transfers, %lu IN transfers, %lus",
             (unsigned long)job_bytes_out, (unsigned long)job_bytes_in,
             (unsigned long)job_transfers_out, (unsigned long)job_transfers_in,
             (unsigned long)elapsed_s);
    job_active = false;
    toggle_reset_for_job = false;  // Allow toggle reset for next job
  }

  // Detect start of new print job (first bulk OUT after idle)
  if (direction == 0 && !job_active) {
    job_active = true;
    job_bytes_out = 0;
    job_bytes_in = 0;
    job_transfers_out = 0;
    job_transfers_in = 0;
    job_start_time = millis();
    last_progress_log = 0;
    ESP_LOGI(TAG, "Print job started: first bulk OUT %lu bytes to ep%lu", (unsigned long)length, (unsigned long)ep);

    // Log data format identifier (first 32 bytes of first transfer)
    if (length > 0) {
      char preview[128];
      int plen = 0;
      // Show as ASCII where printable, hex otherwise
      for (uint32_t i = 0; i < length && i < 48 && plen < 120; i++) {
        if (data[i] >= 0x20 && data[i] < 0x7F) {
          preview[plen++] = data[i];
        } else {
          plen += snprintf(preview + plen, sizeof(preview) - plen, "\\x%02X", data[i]);
        }
      }
      preview[plen] = 0;
      ESP_LOGI(TAG, "Job data start: %s", preview);
    }
  }

  last_transfer_time = millis();

  std::vector<uint8_t> response_data;
  int32_t status = 0;

  uint8_t ep_addr = (ep & 0x0F) | (direction ? 0x80 : 0x00);

  if (direction == 0) {
#if USE_DIRECT_BULK_OUT
    // Bulk OUT - use Buffer DMA mode workaround (ESP32-S3 SG-DMA OUT freeze)
    uint32_t actual_sent = 0;
    DirectUSBResult result = direct_bulk_out(
        this->usb_device_.dev_addr, ep & 0x0F, data, length, 5000, &actual_sent);

    if (result == DirectUSBResult::OK) {
      job_bytes_out += actual_sent;
      job_transfers_out++;
      status = 0;

      // Log first transfer data preview (helps identify data format)
      if (job_transfers_out == 1 && length > 0) {
        char hex[64];
        int hex_len = 0;
        for (uint32_t i = 0; i < length && i < 16 && hex_len < 60; i++) {
          hex_len += snprintf(hex + hex_len, sizeof(hex) - hex_len, "%02X ", data[i]);
        }
        ESP_LOGD(TAG, "First OUT data (%lu bytes): %s", (unsigned long)length, hex);
      }
    } else {
      ESP_LOGW(TAG, "Bulk OUT failed: result=%d sent=%lu/%lu",
               (int)result, (unsigned long)actual_sent, (unsigned long)length);
      status = (result == DirectUSBResult::ERROR_STALL) ? -32 : -110;
    }

    // Log progress periodically (every 2 seconds)
    uint32_t now = millis();
    if (now - last_progress_log >= 2000) {
      last_progress_log = now;
      uint32_t elapsed_s = (now - job_start_time) / 1000;
      uint32_t rate = elapsed_s > 0 ? (job_bytes_out / elapsed_s) : 0;
      ESP_LOGI(TAG, "Print progress: %lu bytes sent (%lu transfers, %lu B/s, %lus)",
               (unsigned long)job_bytes_out, (unsigned long)job_transfers_out,
               (unsigned long)rate, (unsigned long)elapsed_s);
    }
#else
    // Bulk OUT - chunked to BULK_MPS to avoid SG-DMA freeze on ESP32-S3.
    // Large transfers require multiple scatter-gather DMA descriptors which hang
    // the DWC2 controller. Sending max-packet-size chunks uses single DMA
    // descriptors, and ESP-IDF HAL handles DATA toggle properly.
    usb_transfer_t *transfer;
    esp_err_t err = usb_host_transfer_alloc(BULK_MPS, 0, &transfer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Bulk OUT alloc failed: %s", esp_err_to_name(err));
      this->send_urb_response_(seqnum, -12, nullptr, 0);  // -ENOMEM
      return false;
    }

    transfer->device_handle = usb_device_hdl;
    transfer->bEndpointAddress = ep_addr;
    transfer->callback = usb_transfer_callback;
    transfer->context = nullptr;

    // Log first transfer data preview
    if (job_transfers_out == 0 && length > 0) {
      char hex[64];
      int hex_len = 0;
      for (uint32_t i = 0; i < length && i < 16 && hex_len < 60; i++) {
        hex_len += snprintf(hex + hex_len, sizeof(hex) - hex_len, "%02X ", data[i]);
      }
      ESP_LOGD(TAG, "First OUT data (%lu bytes): %s", (unsigned long)length, hex);
    }

    uint32_t total_sent = 0;
    bool chunk_failed = false;

    ESP_LOGD(TAG, "Bulk OUT: %lu bytes to ep 0x%02X (chunked %d-byte)",
             (unsigned long)length, ep_addr, BULK_MPS);

    while (total_sent < length && !chunk_failed) {
      uint32_t chunk = length - total_sent;
      if (chunk > BULK_MPS) chunk = BULK_MPS;

      memcpy(transfer->data_buffer, data + total_sent, chunk);
      transfer->num_bytes = chunk;

      prepare_transfer(transfer);

      err = usb_host_transfer_submit(transfer);
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Bulk OUT chunk failed at %lu/%lu: %s",
                 (unsigned long)total_sent, (unsigned long)length, esp_err_to_name(err));
        status = -32;  // -EPIPE
        chunk_failed = true;
        break;
      }

      if (wait_for_transfer(5000, false)) {
        if (transfer_status == USB_TRANSFER_STATUS_COMPLETED) {
          total_sent += chunk;
        } else {
          ESP_LOGW(TAG, "Bulk OUT chunk error: status=%d at %lu/%lu",
                   transfer_status, (unsigned long)total_sent, (unsigned long)length);
          status = -32;  // -EPIPE
          chunk_failed = true;
        }
      } else {
        ESP_LOGW(TAG, "Bulk OUT chunk timeout at %lu/%lu",
                 (unsigned long)total_sent, (unsigned long)length);
        esp_err_t halt_err = usb_host_endpoint_halt(usb_device_hdl, ep_addr);
        if (halt_err == ESP_OK) {
          usb_host_endpoint_flush(usb_device_hdl, ep_addr);
          usb_host_client_handle_events(usb_client_hdl, 50);
          usb_host_endpoint_clear(usb_device_hdl, ep_addr);
        }
        status = -110;  // -ETIMEDOUT
        chunk_failed = true;
      }
    }

    if (!chunk_failed) {
      job_bytes_out += total_sent;
      job_transfers_out++;
      status = 0;
      ESP_LOGD(TAG, "Bulk OUT complete: %lu bytes", (unsigned long)total_sent);
    }

    usb_host_transfer_free(transfer);

    // Log progress periodically
    uint32_t now = millis();
    if (now - last_progress_log >= 2000) {
      last_progress_log = now;
      uint32_t elapsed_s = (now - job_start_time) / 1000;
      uint32_t rate = elapsed_s > 0 ? (job_bytes_out / elapsed_s) : 0;
      ESP_LOGI(TAG, "Print progress: %lu bytes sent (%lu transfers, %lu B/s, %lus)",
               (unsigned long)job_bytes_out, (unsigned long)job_transfers_out,
               (unsigned long)rate, (unsigned long)elapsed_s);
    }
#endif
  } else {
    // Bulk IN - use ESP-IDF driver (works in SG-DMA mode)
    // Before a print job is active, the printer has no data to send.
    // Returning immediately avoids halt/flush/clear cycles on the IN endpoint
    // which can corrupt DWC2 state and prevent subsequent OUT transfers.
    if (!job_active) {
      // Return 0 bytes immediately - printer has no data before job starts
      this->send_urb_response_(seqnum, 0, nullptr, 0);
      return true;
    }

    // ESP-IDF requires IN transfer size to be a multiple of MPS
    usb_transfer_t *transfer;
    size_t req_size = length > 0 ? length : BULK_MPS;
    size_t alloc_size = ((req_size + BULK_MPS - 1) / BULK_MPS) * BULK_MPS;
    esp_err_t err = usb_host_transfer_alloc(alloc_size, 0, &transfer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to allocate transfer: %s", esp_err_to_name(err));
      this->send_urb_response_(seqnum, -12, nullptr, 0);  // -ENOMEM
      return false;
    }

    transfer->device_handle = usb_device_hdl;
    transfer->bEndpointAddress = ep_addr;
    transfer->callback = usb_transfer_callback;
    transfer->context = nullptr;
    transfer->num_bytes = alloc_size;

    uint32_t seq = prepare_transfer(transfer);

    err = usb_host_transfer_submit(transfer);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Bulk IN submit failed: %s", esp_err_to_name(err));
      usb_host_transfer_free(transfer);
      status = -32;  // -EPIPE
    } else {
      if (wait_for_transfer(100, false)) {
        if (transfer_status == USB_TRANSFER_STATUS_COMPLETED) {
          response_data.assign(transfer->data_buffer,
                               transfer->data_buffer + transfer_actual_bytes);
          job_bytes_in += transfer_actual_bytes;
          job_transfers_in++;
          if (transfer_actual_bytes > 0) {
            ESP_LOGI(TAG, "Bulk IN: %d bytes from printer", transfer_actual_bytes);
          }
        } else if (transfer_status == USB_TRANSFER_STATUS_STALL) {
          ESP_LOGW(TAG, "Bulk IN STALL on ep 0x%02X", ep_addr);
          // Clear ESP-IDF internal endpoint state after STALL
          usb_host_endpoint_flush(usb_device_hdl, ep_addr);
          usb_host_client_handle_events(usb_client_hdl, 50);
          usb_host_endpoint_clear(usb_device_hdl, ep_addr);
          status = -32;  // -EPIPE (Linux will send CLEAR_FEATURE)
        } else {
          ESP_LOGW(TAG, "Bulk IN status: %d", transfer_status);
          // Clear endpoint state for any error
          usb_host_endpoint_halt(usb_device_hdl, ep_addr);
          usb_host_endpoint_flush(usb_device_hdl, ep_addr);
          usb_host_client_handle_events(usb_client_hdl, 50);
          usb_host_endpoint_clear(usb_device_hdl, ep_addr);
          status = -32;  // -EPIPE
        }
      } else {
        // IN transfer timeout - device has no data, halt/flush/clear endpoint
        esp_err_t halt_err = usb_host_endpoint_halt(usb_device_hdl, ep_addr);
        if (halt_err == ESP_OK) {
          usb_host_endpoint_flush(usb_device_hdl, ep_addr);
          usb_host_client_handle_events(usb_client_hdl, 50);
          usb_host_endpoint_clear(usb_device_hdl, ep_addr);
        } else {
          usb_host_client_handle_events(usb_client_hdl, 10);
        }
        status = 0;  // Success with 0 bytes
      }
      usb_host_transfer_free(transfer);
    }
  }

  // For bulk OUT, the actual_length in response should be the number of bytes accepted
  // Linux USBIP driver checks this to confirm data was delivered
  if (direction == 0 && status == 0) {
    this->send_urb_response_(seqnum, status, nullptr, length);
  } else {
    this->send_urb_response_(seqnum, status, response_data.data(), response_data.size());
  }
  return true;
}

void USBIPComponent::send_urb_response_(uint32_t seqnum, int32_t status,
                                         const uint8_t *data, uint32_t length) {
  if (status != 0) {
    ESP_LOGW(TAG, "URB response: seq=%lu status=%ld len=%lu",
             (unsigned long)seqnum, (long)status, (unsigned long)length);
  } else {
    ESP_LOGD(TAG, "URB response: seq=%lu status=OK len=%lu",
             (unsigned long)seqnum, (unsigned long)length);
  }

  USBIPRetSubmit response;
  memset(&response, 0, sizeof(response));

  response.command = htonl_(USBIP_RET_SUBMIT);
  response.seqnum = htonl_(seqnum);
  response.devid = 0;
  response.direction = 0;
  response.ep = 0;
  response.status = htonl_(static_cast<uint32_t>(status));
  response.actual_length = htonl_(length);
  response.start_frame = 0;
  response.number_of_packets = 0;
  response.error_count = 0;

  this->write_bytes_(reinterpret_cast<uint8_t *>(&response), sizeof(response));

  if (data && length > 0) {
    this->write_bytes_(data, length);
  }
}

void USBIPComponent::disconnect_client_() {
  if (this->client_fd_ >= 0) {
    close(this->client_fd_);
    this->client_fd_ = -1;
  }
  this->client_state_ = CLIENT_STATE_IDLE;
  ESP_LOGI(TAG, "Client disconnected");
}

bool USBIPComponent::read_bytes_(uint8_t *buffer, size_t length, uint32_t timeout_ms) {
  if (this->client_fd_ < 0) {
    return false;
  }

  size_t total_read = 0;
  uint32_t start = millis();

  while (total_read < length && (millis() - start) < timeout_ms) {
    ssize_t n = recv(this->client_fd_, buffer + total_read, length - total_read, 0);
    if (n > 0) {
      total_read += n;
    } else if (n == 0) {
      // Connection closed
      return false;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
      return false;
    } else {
      // Feed watchdog and yield while waiting for data
      esp_task_wdt_reset();
      vTaskDelay(1);
    }
  }

  return total_read == length;
}

bool USBIPComponent::write_bytes_(const uint8_t *buffer, size_t length) {
  if (this->client_fd_ < 0) {
    return false;
  }

  size_t total_written = 0;
  while (total_written < length) {
    ssize_t n = send(this->client_fd_, buffer + total_written, length - total_written, 0);
    if (n > 0) {
      total_written += n;
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      return false;
    } else {
      // Feed watchdog and yield while waiting
      esp_task_wdt_reset();
      vTaskDelay(1);
    }
  }

  return total_written == length;
}

uint16_t USBIPComponent::htons_(uint16_t value) {
  return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
}

uint32_t USBIPComponent::htonl_(uint32_t value) {
  return ((value & 0xFF) << 24) |
         ((value & 0xFF00) << 8) |
         ((value >> 8) & 0xFF00) |
         ((value >> 24) & 0xFF);
}

uint16_t USBIPComponent::ntohs_(uint16_t value) {
  return htons_(value);
}

uint32_t USBIPComponent::ntohl_(uint32_t value) {
  return htonl_(value);
}

}  // namespace usbip
}  // namespace esphome
