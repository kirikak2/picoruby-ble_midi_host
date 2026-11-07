/*
 * PicoRuby BLE-MIDI Host
 *
 * The board as a BLE-MIDI central: it scans for peripherals advertising the
 * MIDI service (BLE-MIDI keyboards, WIDI / MD-BT01 adapters, ...), connects,
 * subscribes to the MIDI I/O characteristic and writes to it.
 *
 * Several peripherals at once, each in a slot of its own (like the USB-MIDI
 * host on a hub): a dynamic bit in picoruby-midi's transport registry per
 * slot, taken the first time the slot is used and kept; the same peripheral
 * comes back to its slot. Peripherals connected once are remembered (NVS)
 * and connected again automatically while the host is enabled.
 *
 * Thread model:
 *   NimBLE host task: GAP / GATT callbacks -> decode -> the slot's RX ring
 *   any task       : BLE_MIDI_HOST_dev_send_packet -> the slot's TX ring
 *   ble_midi_host task (same core as the NimBLE host): TX batching, scans,
 *                  connects, the reconnect timer, enable / disable
 *
 * Build-time knobs: ble_midi_host_config.h.
 */

#ifndef BLE_MIDI_HOST_DEFINED_H_
#define BLE_MIDI_HOST_DEFINED_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "midi_transport.h"   /* picoruby-midi: midi_tmask_t */
#include "ble_midi_host_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t addr[6];          /* most significant byte first */
    uint8_t addr_type;        /* BLE_ADDR_PUBLIC / RANDOM / ... */
    char    name[32];
    int8_t  rssi;             /* scan results only */
    bool    known;            /* remembered (connected before) */
    bool    connected;        /* in a slot now */
} ble_midi_peer_t;

typedef enum {
    BLE_MIDI_HOST_SLOT_FREE = 0,      /* never used, or the peripheral is gone */
    BLE_MIDI_HOST_SLOT_CONNECTING,
    BLE_MIDI_HOST_SLOT_DISCOVERING,   /* MTU / service / characteristic */
    BLE_MIDI_HOST_SLOT_SUBSCRIBING,   /* CCCD write (pairing if asked) */
    BLE_MIDI_HOST_SLOT_CONNECTED,     /* MIDI flows */
} ble_midi_host_slot_status_t;

typedef struct {
    ble_midi_host_slot_status_t status;
    bool     encrypted;
    bool     bonded;
    uint16_t mtu;
    uint16_t interval_1250us;
    uint32_t rx_packets;
    uint32_t rx_messages;
    uint32_t rx_dropped;
    uint32_t decode_errors;
    uint32_t tx_messages;
    uint32_t tx_packets;
    uint32_t tx_dropped;
    uint32_t tx_retries;
    uint32_t connections;
    int      last_error;      /* NimBLE status of the last failed step (0: none) */
} ble_midi_host_stats_t;

/* Allocate the slots; BLE is not started. Idempotent. -1 without BLE. */
int  BLE_MIDI_HOST_init(void);
bool BLE_MIDI_HOST_available(void);

/* Hold the BLE host and reconnect the remembered peripherals (on), or drop
 * every link (off). Asynchronous. */
int  BLE_MIDI_HOST_enable(bool on);
bool BLE_MIDI_HOST_is_enabled(void);
/* Whether the BLE host is up for us (enabled and started) */
bool BLE_MIDI_HOST_is_ready(void);

/* Automatic reconnection of remembered peripherals (default on) */
void BLE_MIDI_HOST_set_auto_connect(bool on);
bool BLE_MIDI_HOST_auto_connect(void);

/* Scan for BLE-MIDI peripherals for `duration_ms`. Enables the host if it
 * is not. Results accumulate until the next scan starts. */
int  BLE_MIDI_HOST_scan_start(uint32_t duration_ms);
void BLE_MIDI_HOST_scan_stop(void);
bool BLE_MIDI_HOST_scanning(void);
int  BLE_MIDI_HOST_scan_results(ble_midi_peer_t *out, int max);

/* Connect to a peripheral (asynchronous). Returns the slot it goes to, or
 * -1 (no free slot / not available). The slot's status says how far it got. */
int  BLE_MIDI_HOST_connect(const uint8_t addr[6], uint8_t addr_type);
void BLE_MIDI_HOST_disconnect(int slot);

/* Remembered peripherals */
int  BLE_MIDI_HOST_known(ble_midi_peer_t *out, int max);
/* Forget a peripheral (disconnects it, removes its bond). 0 / -1 */
int  BLE_MIDI_HOST_forget(const uint8_t addr[6]);

/* --- slots --- */
int          BLE_MIDI_HOST_dev_status(int slot);
midi_tmask_t BLE_MIDI_HOST_dev_bit(int slot);
bool         BLE_MIDI_HOST_dev_connected(int slot);
int          BLE_MIDI_HOST_dev_info(int slot, ble_midi_peer_t *out);   /* 0 / -1 */
void         BLE_MIDI_HOST_dev_stats(int slot, ble_midi_host_stats_t *out);
int  BLE_MIDI_HOST_dev_send_packet(int slot, uint8_t cable, uint8_t cin,
                                   uint8_t b1, uint8_t b2, uint8_t b3);
int  BLE_MIDI_HOST_dev_bytes_available(int slot);
int  BLE_MIDI_HOST_dev_read_packet(int slot, uint8_t *buf, size_t maxlen);
/* Slot of a connected peripheral whose name contains `name` (-1: none) */
int  BLE_MIDI_HOST_find(const char *name);

/* "AA:BB:CC:DD:EE:FF" <-> addr[6]; parse returns 0 / -1 */
void BLE_MIDI_HOST_format_addr(const uint8_t addr[6], char out[18]);
int  BLE_MIDI_HOST_parse_addr(const char *s, uint8_t addr[6]);

#ifdef __cplusplus
}
#endif

#endif /* BLE_MIDI_HOST_DEFINED_H_ */
