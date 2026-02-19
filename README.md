# ESPHome USB/IP Server for Printer

A USB/IP server that runs on a Waveshare ESP32-S3-ETH board as an [ESPHome](https://esphome.io/) component. It bridges a locally connected USB device to a remote Linux machine over Ethernet, using the standard [USB/IP](https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html) protocol.

```
 Linux Client                  ESP32-S3-ETH               USB Device
 ──────────────                ────────────               ──────────
 usbip attach ──► TCP:3240 ──► USB/IP server ──► USB ──► Printer
 CUPS / lp    ◄── USB/IP   ◄── component     ◄── USB ◄── Keyboard, ...
```

The Linux host sees the USB device as if it were directly attached. All USB operations (control transfers, bulk IN/OUT, descriptors) are forwarded transparently.

Implemented with help of AI. This is a POC and works for my HP LaserJet P1102w printer.

## Features

- Standard USB/IP v1.1.1 protocol — works with the `usbip` tools shipped with the Linux kernel
- USB device hot-plug detection (connect and disconnect at runtime)
- Automatic HP Smart Install firmware upload for HP LaserJet P1102w printer
- Workaround for ESP32-S3 DWC2 bulk OUT DMA silicon bug

## Prerequisites

- **Hardware:** [Waveshare ESP32-S3-ETH](https://www.waveshare.com/wiki/ESP32-S3-ETH) (or any ESP32-S3 board with Ethernet and exposed USB host pins)
- **Framework:** [ESPHome](https://esphome.io/) 2025.12+ with ESP-IDF backend
- **Linux client:** `usbip` utilities (package `linux-tools-common` on Ubuntu/Debian)

## Getting Started

### 1. Set up the Python virtual environment

```bash
python3 -m venv virtenv
source virtenv/bin/activate
pip install esphome
```

### 2. Build and flash

```bash
source virtenv/bin/activate
esphome compile usbip_config.yaml
esphome upload usbip_config.yaml
```

### 3. Attach from Linux

```bash
# Discover
usbip list -r <esp32-ip>

# Attach
sudo usbip attach -r <esp32-ip> -b 1-1

# Verify
lsusb

# Detach
sudo usbip detach -p 0
```

Once attached, the device appears as a native USB device. Use it with CUPS, `lp`, or any standard driver.

## Configuration

### Component parameters

```yaml
usbip:
  port: 3240                        # optional, default 3240
  printer_firmware: sihpP1102.dl    # optional, HP P1102w only
```

| Parameter | Required | Default | Description |
|-----------|----------|---------|-------------|
| `port` | No | `3240` | TCP port for the USB/IP server. 3240 is the standard Linux USB/IP port. |
| `printer_firmware` | No | — | Path to an HP printer firmware blob (relative to config file). When set, the firmware is embedded into flash at compile time and automatically uploaded whenever the printer is plugged in. |

### Required ESP-IDF sdkconfig options

These must be set under `esp32.framework.sdkconfig_options`:

| Option | Value | Reason |
|--------|-------|--------|
| `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE` | `"1024"` | Printers return large descriptors and Device ID strings (up to ~1 KB). The ESP-IDF default of 256 bytes causes truncated responses. |
| `CONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED` | `y` | Balances the DWC2 internal FIFO between IN and OUT endpoints. The default bias toward periodic endpoints starves bulk transfers. |
| `CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK` | `y` | Lets the component accept all USB devices and control which configuration is selected during enumeration. |
| `CONFIG_SPIRAM` | `n` | The DWC2 DMA engine requires internal SRAM. PSRAM-backed buffers cause silent DMA failures or bus contention. |

### Required build flags

```yaml
esphome:
  platformio_options:
    build_flags:
      - -DCONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=1024
      - -DCONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED=1
```

These mirror the sdkconfig values as preprocessor defines. Some ESP-IDF headers check these via `#ifdef`, so both the sdkconfig entry and the build flag are needed.

### Logger baud rate

```yaml
logger:
  baud_rate: 0
```

**Mandatory.** The ESP32-S3 USB pins (GPIO19/20) are shared between JTAG/serial and USB Host mode. `baud_rate: 0` releases them for USB Host. Log output is available via the ESPHome web server or Home Assistant API instead.

### Full example

See [`usbip_config.yaml`](usbip_config.yaml) for a complete working configuration including Ethernet, web server, OTA, and diagnostics.


## Technical Details

### USB/IP Protocol

The component implements USB/IP protocol version 0x0111 over a single TCP connection:

| Message | Direction | Purpose |
|---------|-----------|---------|
| `OP_REQ_DEVLIST` / `OP_REP_DEVLIST` | Client ← → Server | List connected USB devices with descriptors |
| `OP_REQ_IMPORT` / `OP_REP_IMPORT` | Client ← → Server | Attach a device; connection switches to URB mode |
| `USBIP_CMD_SUBMIT` / `USBIP_RET_SUBMIT` | Client ← → Server | Forward USB Request Blocks (control + bulk transfers) |
| `USBIP_CMD_UNLINK` / `USBIP_RET_UNLINK` | Client ← → Server | Cancel pending transfers |

### ESP32-S3 DWC2 Bulk OUT Bug

The ESP32-S3 uses a Synopsys DWC2 USB controller in Scatter-Gather DMA mode. This mode has a silicon bug: **bulk OUT transfers requiring multiple DMA descriptors freeze the controller**. Any OUT transfer larger than one max-packet-size (64 bytes for Full-Speed) triggers the freeze.

**Workaround:** Bulk OUT data is split into individual 64-byte MPS chunks, each submitted as a separate transfer via the ESP-IDF API. Single-descriptor transfers avoid the bug. ESP-IDF handles DATA0/DATA1 toggle tracking internally.

A direct Buffer DMA mode fallback (`USE_DIRECT_BULK_OUT=1`) that bypasses ESP-IDF is included in the source but disabled by default.

### Bulk IN Alignment

ESP-IDF rejects bulk IN transfers whose size is not a multiple of the endpoint max packet size (`ESP_ERR_INVALID_ARG`). The component rounds up all IN buffer allocations to the next 64-byte boundary.

### Device Hot-Plug

1. **Connect** — a background daemon task detects USB port events and sets flags. The main ESPHome loop opens the device, reads descriptors, claims interfaces, and caches configuration data.
2. **Disconnect** — interfaces are released, the device handle is closed, and the USB/IP TCP connection is terminated so the Linux client immediately detects the removal.
3. **Fallback discovery** — if the ESP-IDF `NEW_DEV` callback doesn't fire (a known edge case), the daemon scans addresses 1–10 after 5.5 seconds.

### HP P1102w Firmware Upload

The HP LaserJet P1102w powers up as a USB Mass Storage device ("Smart Install"). It must receive a firmware blob before switching to Printer class mode. When `printer_firmware` is configured, this happens automatically:

1. **Detect** — mass storage interface (class 0x08) on an HP VID triggers the flow
2. **Mode switch** — SCSI vendor command 0xD0 sent via CBW; device disconnects and re-enumerates
3. **Upload** — firmware sent in 64-byte chunks; progress logged every 10%
4. **Ready** — device re-enumerates with firmware loaded (`FWVER:` appears in Device ID)

The firmware is volatile (lost on every power cycle), so the upload repeats on every replug.

### Print Job Tracking

Bulk OUT activity is tracked to provide print job visibility in the logs:

- **Start** — first bulk OUT after 5 seconds of idle
- **Progress** — bytes sent, transfer count, and throughput logged every 2 seconds
- **Data preview** — first 48 bytes shown in ASCII/hex to identify the data format (PJL, ZJS, PCL, etc.)
- **End** — 5 seconds of inactivity after the last transfer

### Transfer Synchronization

USB transfers in ESP-IDF are asynchronous. The component uses a sequence-numbered semaphore pattern to match completions to requests:

1. `prepare_transfer()` assigns a unique sequence number and clears completion state
2. The transfer callback only accepts completions matching the expected sequence number
3. `wait_for_transfer()` actively pumps USB host events while waiting, preventing deadlocks between the ESPHome main loop and the USB daemon task


## Limitations

- Only one USB device is supported at a time (single-port USB host)
- Only one USB/IP client can be connected at a time
- Isochronous and interrupt transfers are not implemented (bulk and control only)
- Full-Speed devices only (12 Mbit/s) — the ESP32-S3 USB host does not support High-Speed
- PSRAM must be disabled due to DMA constraints

## Troubleshooting

| Symptom | Cause | Fix                                                                |
|---------|-------|--------------------------------------------------------------------|
| `usbip list` shows no devices | Printer not enumerated yet | Wait 10 seconds after boot; check web log for "Device enumerated"  |
| `usbip attach` hangs | Firewall blocking port 3240 | Open TCP port 3240 on the network                                  |
| Bulk OUT transfers fail / timeout | SG-DMA freeze with large transfers | Already mitigated by MPS chunking; check logs for STALL/NAK errors |
| `baud_rate` error on boot | Logger using USB pins | Set `logger: baud_rate: 0`                                         |
| Printer stuck in mass storage mode | Firmware not configured or upload failed | Add `printer_firmware: sihpP1102.dl` to config; verify file exists |
| Linux sees device but printing fails | Wrong driver or data format | Use `foo2zjs` for HP P1102w; verify with `lpstat -t`               |
