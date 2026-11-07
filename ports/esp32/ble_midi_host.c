/*
 * PicoRuby BLE-MIDI Host - ESP32 port (NimBLE central)
 *
 * One client of picoruby-ble's shared NimBLE owner (nimble_owner.h). It adds
 * no GATT services, so acquiring a running host never restarts it.
 *
 * Connect sequence per slot (all on the NimBLE host task, chained callbacks):
 *   ble_gap_connect -> CONNECT -> exchange MTU -> MIDI service -> MIDI I/O
 *   characteristic -> its descriptors (CCCD) -> write 0x0001 to the CCCD
 *   (pairing first if the peripheral asks for it) -> CONNECTED
 */

#include "../../include/ble_midi_host.h"

#if BLE_MIDI_HOST_ENABLED

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_store.h"

#include "nimble_owner.h"       /* picoruby-ble's ESP32 port */
#include "ble_midi_codec.h"
#include "midi_timed_ring.h"     /* picoruby-midi */
#include "midi.h"
#include "midi_route.h"

void ble_store_config_init(void);

static const char *TAG = "BLE_MIDI_HOST";

#define NB_TX          0x01
#define NB_ENABLE      0x02
#define NB_DISABLE     0x04
#define NB_SCAN        0x08
#define NB_SCAN_STOP   0x10
#define NB_CONNECT     0x20

#define TX_MASK (BLE_MIDI_HOST_TX_PACKETS - 1)
#define TX_BUF_MAX 512
#define CCCD_UUID 0x2902

#define NVS_NS  "ble_midi"
#define NVS_KEY "known"

typedef struct {
    uint8_t  p[4];
    uint16_t ts;
} tx_entry_t;

typedef struct {
    volatile ble_midi_host_slot_status_t status;
    volatile uint16_t conn;
    ble_addr_t addr;                  /* NimBLE order (val[0] = LSB) */
    bool       used;                  /* has (had) a peripheral */
    bool       parked;                /* disconnected by the user: no reconnecting */
    char       name[32];
    uint16_t   svc_start, svc_end;
    uint16_t   val_handle, chr_end;
    uint16_t   cccd_handle;
    bool       security_tried;
    volatile uint16_t mtu;

    midi_transport_t transport;
    midi_tmask_t bit;

    midi_timed_ring_t *rx;
    ble_midi_decoder_t dec;
    ble_midi_sync_t sync;
    int64_t rx_now;                   /* arrival of the packet being decoded */
    int32_t rx_budget;

    tx_entry_t *tx;
    volatile uint32_t tx_head, tx_tail;
    portMUX_TYPE tx_mux;
    uint8_t *txbuf;
    size_t   tx_pending;
    uint8_t  tx_done;
    ble_midi_encoder_t enc;
    bool     enc_open;

    ble_midi_host_stats_t stats;
} slot_t;

typedef struct {
    uint8_t type;
    uint8_t val[6];        /* NimBLE order */
    char    name[25];
} known_t;

static bool s_inited;
static slot_t *s_slots;               /* BLE_MIDI_HOST_MAX_DEVICES */
static TaskHandle_t s_task;
static esp_timer_handle_t s_batch_timer;

static volatile bool s_wanted;
static volatile bool s_held;
static volatile bool s_auto = true;

/* scans */
static volatile bool s_scanning;
static volatile bool s_scan_is_user;  /* the user's scan (results shown) vs reconnect */
static volatile uint32_t s_scan_ms;
static ble_midi_peer_t *s_results;    /* BLE_MIDI_HOST_MAX_RESULTS */
static volatile int s_result_count;
static portMUX_TYPE s_results_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_next_reconnect_us;

/* connect request / the one connection being made */
static volatile int s_connect_slot = -1;   /* asked for, made on the task */
static volatile int s_connecting = -1;     /* slot of ble_gap_connect in progress */

static known_t s_known[BLE_MIDI_HOST_MAX_KNOWN];
static int s_known_count;
static bool s_known_loaded;
static portMUX_TYPE s_known_mux = portMUX_INITIALIZER_UNLOCKED;

/* 03B80E5A-EDE8-4B33-A751-6CE34EC4C700 / 7772E5DB-3868-4112-A1A9-F2669D106BF3 */
static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03);
static const ble_uuid128_t s_chr_uuid = BLE_UUID128_INIT(
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77);

static int gap_event(struct ble_gap_event *event, void *arg);
static void subscribe(slot_t *s);

static inline uint16_t now_ts(void)
{
    return (uint16_t)((esp_timer_get_time() / 1000) & BLE_MIDI_TS_MASK);
}

static void wake(uint32_t bits)
{
    if (s_task) xTaskNotify(s_task, bits, eSetBits);
}

static slot_t *slot_at(int i)
{
    if (s_slots == NULL || i < 0 || i >= BLE_MIDI_HOST_MAX_DEVICES) return NULL;
    return &s_slots[i];
}

static slot_t *slot_by_conn(uint16_t conn)
{
    for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        if (s_slots[i].conn == conn && s_slots[i].status != BLE_MIDI_HOST_SLOT_FREE) return &s_slots[i];
    }
    return NULL;
}

static int slot_index(const slot_t *s) { return (int)(s - s_slots); }

static bool addr_eq(const ble_addr_t *a, const uint8_t val[6])
{
    return memcmp(a->val, val, 6) == 0;
}

static void to_msb(const uint8_t val[6], uint8_t out[6])
{
    for (int i = 0; i < 6; i++) out[i] = val[5 - i];
}

void BLE_MIDI_HOST_format_addr(const uint8_t a[6], char out[18])
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", a[0], a[1], a[2], a[3], a[4], a[5]);
}

int BLE_MIDI_HOST_parse_addr(const char *s, uint8_t a[6])
{
    if (s == NULL) return -1;
    for (int i = 0; i < 6; i++) {
        int hi, lo;
        if (!isxdigit((unsigned char)s[0]) || !isxdigit((unsigned char)s[1])) return -1;
        hi = isdigit((unsigned char)s[0]) ? s[0] - '0' : (toupper((unsigned char)s[0]) - 'A' + 10);
        lo = isdigit((unsigned char)s[1]) ? s[1] - '0' : (toupper((unsigned char)s[1]) - 'A' + 10);
        a[i] = (uint8_t)(hi << 4 | lo);
        s += 2;
        if (i < 5) {
            if (*s != ':' && *s != '-') return -1;
            s++;
        }
    }
    return *s == '\0' ? 0 : -1;
}

/* ---------------------------------------------------------------------
 * Remembered peripherals (NVS)
 * ------------------------------------------------------------------- */

static void known_load(void)
{
    if (s_known_loaded) return;
    s_known_loaded = true;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof(s_known);
    if (nvs_get_blob(h, NVS_KEY, s_known, &len) == ESP_OK) {
        s_known_count = (int)(len / sizeof(known_t));
        for (int i = 0; i < s_known_count; i++) s_known[i].name[sizeof(s_known[i].name) - 1] = '\0';
    }
    nvs_close(h);
}

static void known_save(void)
{
    known_t copy[BLE_MIDI_HOST_MAX_KNOWN];
    int n;
    portENTER_CRITICAL(&s_known_mux);
    n = s_known_count;
    memcpy(copy, s_known, sizeof(known_t) * (size_t)n);
    portEXIT_CRITICAL(&s_known_mux);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (n > 0) nvs_set_blob(h, NVS_KEY, copy, sizeof(known_t) * (size_t)n);
    else nvs_erase_key(h, NVS_KEY);
    nvs_commit(h);
    nvs_close(h);
}

static int known_find(const uint8_t val[6])
{
    for (int i = 0; i < s_known_count; i++) {
        if (memcmp(s_known[i].val, val, 6) == 0) return i;
    }
    return -1;
}

/* Called from the host task: only marks; the save runs on the gem's task */
static volatile bool s_known_dirty;

static void known_remember(const slot_t *s)
{
    portENTER_CRITICAL(&s_known_mux);
    int i = known_find(s->addr.val);
    if (i < 0) {
        if (s_known_count == BLE_MIDI_HOST_MAX_KNOWN) {   /* drop the oldest */
            memmove(&s_known[0], &s_known[1], sizeof(known_t) * (BLE_MIDI_HOST_MAX_KNOWN - 1));
            s_known_count--;
        }
        i = s_known_count++;
    } else if (i != s_known_count - 1) {
        /* most recent last */
        known_t k = s_known[i];
        memmove(&s_known[i], &s_known[i + 1], sizeof(known_t) * (size_t)(s_known_count - 1 - i));
        s_known[s_known_count - 1] = k;
        i = s_known_count - 1;
    }
    s_known[i].type = s->addr.type;
    memcpy(s_known[i].val, s->addr.val, 6);
    strncpy(s_known[i].name, s->name, sizeof(s_known[i].name) - 1);
    s_known[i].name[sizeof(s_known[i].name) - 1] = '\0';
    portEXIT_CRITICAL(&s_known_mux);
    s_known_dirty = true;
}

/* ---------------------------------------------------------------------
 * RX / TX
 * ------------------------------------------------------------------- */

static volatile int32_t s_rx_latency = BLE_MIDI_HOST_RX_LATENCY_US;

static int32_t rx_budget(const slot_t *s)
{
    int32_t l = s_rx_latency;
    if (l < 0) return ble_midi_sync_auto_budget(s->stats.interval_1250us);
    return l;
}

static void rx_push(void *arg, const uint8_t pkt[4], uint16_t ts)
{
    slot_t *s = (slot_t *)arg;
    int64_t at = ble_midi_sync_release(&s->sync, ts, s->rx_now, s->rx_budget);
    if (!midi_timed_ring_push(s->rx, pkt, at)) {
        s->stats.rx_dropped++;
        return;
    }
    s->stats.rx_messages++;
}

static void tx_clear(slot_t *s)
{
    portENTER_CRITICAL(&s->tx_mux);
    s->tx_tail = s->tx_head;
    portEXIT_CRITICAL(&s->tx_mux);
    s->tx_pending = 0;
    s->tx_done = 0;
    s->enc_open = false;
    s->enc.sysex_open = false;
}

static void batch_timer_cb(void *arg)
{
    (void)arg;
    wake(NB_TX);
}

static bool write_out(slot_t *s, const uint8_t *buf, size_t len)
{
    uint16_t conn = s->conn;
    if (s->status != BLE_MIDI_HOST_SLOT_CONNECTED) return true;
    int rc = ble_gattc_write_no_rsp_flat(conn, s->val_handle, buf, (uint16_t)len);
    if (rc == 0) {
        s->stats.tx_packets++;
        return true;
    }
    if (rc == BLE_HS_ENOMEM || rc == BLE_HS_EBUSY) {
        s->stats.tx_retries++;
        return false;
    }
    return true;
}

static bool tx_flush(slot_t *s)
{
    if (s->status != BLE_MIDI_HOST_SLOT_CONNECTED) {
        if (s->tx_head != s->tx_tail || s->tx_pending) tx_clear(s);
        return false;
    }
    if (s->tx_pending) {
        if (!write_out(s, s->txbuf, s->tx_pending)) return true;
        s->tx_pending = 0;
        s->enc_open = false;
    }
    size_t cap = (size_t)s->mtu - 3;
    if (cap > TX_BUF_MAX) cap = TX_BUF_MAX;
    if (cap < 5) cap = 5;

    for (;;) {
        tx_entry_t ent;
        portENTER_CRITICAL(&s->tx_mux);
        bool empty = (s->tx_tail == s->tx_head);
        if (!empty) ent = s->tx[s->tx_tail];
        portEXIT_CRITICAL(&s->tx_mux);
        if (empty) break;
        if (!s->enc_open) {
            ble_midi_encoder_begin(&s->enc, s->txbuf, cap, ent.ts);
            s->enc_open = true;
        }
        uint8_t done = s->tx_done;
        if (ble_midi_encode(&s->enc, ent.p, ent.ts, &done)) {
            s->tx_done = 0;
            portENTER_CRITICAL(&s->tx_mux);
            s->tx_tail = (s->tx_tail + 1) & TX_MASK;
            portEXIT_CRITICAL(&s->tx_mux);
            continue;
        }
        s->tx_done = done;
        if (!write_out(s, s->txbuf, s->enc.len)) {
            s->tx_pending = s->enc.len;
            return true;
        }
        s->enc_open = false;
    }
    if (s->enc_open && ble_midi_encoder_pending(&s->enc)) {
        if (!write_out(s, s->txbuf, s->enc.len)) {
            s->tx_pending = s->enc.len;
            return true;
        }
    }
    s->enc_open = false;
    return false;
}

/* ---------------------------------------------------------------------
 * Connect sequence (host task)
 * ------------------------------------------------------------------- */

static void read_conn(slot_t *s)
{
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(s->conn, &desc) != 0) return;
    s->stats.encrypted = desc.sec_state.encrypted;
    s->stats.bonded = desc.sec_state.bonded;
    s->stats.interval_1250us = desc.conn_itvl;
}

static void fail(slot_t *s, int status, const char *step)
{
    ESP_LOGW(TAG, "slot %d: %s failed (%d)", slot_index(s), step, status);
    s->stats.last_error = status;
    ble_gap_terminate(s->conn, BLE_ERR_REM_USER_CONN_TERM);
}

static int on_cccd_written(uint16_t conn, const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    slot_t *s = (slot_t *)arg;
    if (s->conn != conn) return 0;
    if (error->status == 0) {
        s->status = BLE_MIDI_HOST_SLOT_CONNECTED;
        read_conn(s);
        known_remember(s);
        ESP_LOGI(TAG, "slot %d: \"%s\" connected (MTU %u)", slot_index(s), s->name, (unsigned)s->mtu);
        if (MIDI_Input_was_started() && !MIDI_Input_is_running()) MIDI_Input_start();
        return 0;
    }
    if (!s->security_tried
        && (error->status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_AUTHEN)
            || error->status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_ENC))) {
        s->security_tried = true;
        int rc = ble_gap_security_initiate(conn);
        if (rc != 0) fail(s, rc, "pairing");
        return 0;   /* ENC_CHANGE subscribes again */
    }
    fail(s, error->status, "subscribing");
    return 0;
}

static void subscribe(slot_t *s)
{
    s->status = BLE_MIDI_HOST_SLOT_SUBSCRIBING;
    if (s->cccd_handle == 0) {
        fail(s, BLE_HS_ENOENT, "finding the CCCD");
        return;
    }
    static const uint8_t on[2] = { 0x01, 0x00 };
    int rc = ble_gattc_write_flat(s->conn, s->cccd_handle, on, sizeof(on), on_cccd_written, s);
    if (rc != 0) fail(s, rc, "subscribing");
}

static int on_dsc(uint16_t conn, const struct ble_gatt_error *error,
                  uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle;
    slot_t *s = (slot_t *)arg;
    if (s->conn != conn) return 0;
    if (error->status == 0 && dsc) {
        if (ble_uuid_cmp(&dsc->uuid.u, BLE_UUID16_DECLARE(CCCD_UUID)) == 0) s->cccd_handle = dsc->handle;
        return 0;
    }
    if (error->status == BLE_HS_EDONE) subscribe(s);
    else fail(s, error->status, "descriptor discovery");
    return 0;
}

static int on_chr(uint16_t conn, const struct ble_gatt_error *error,
                  const struct ble_gatt_chr *chr, void *arg)
{
    slot_t *s = (slot_t *)arg;
    if (s->conn != conn) return 0;
    if (error->status == 0 && chr) {
        if (s->val_handle == 0) s->val_handle = chr->val_handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        fail(s, error->status, "characteristic discovery");
        return 0;
    }
    if (s->val_handle == 0) {
        fail(s, BLE_HS_ENOENT, "finding the MIDI characteristic");
        return 0;
    }
    /* its descriptors sit between the value and the end of the service
     * (the characteristic is normally the only one) */
    int rc = ble_gattc_disc_all_dscs(conn, s->val_handle, s->svc_end, on_dsc, s);
    if (rc != 0) fail(s, rc, "descriptor discovery");
    return 0;
}

static int on_svc(uint16_t conn, const struct ble_gatt_error *error,
                  const struct ble_gatt_svc *svc, void *arg)
{
    slot_t *s = (slot_t *)arg;
    if (s->conn != conn) return 0;
    if (error->status == 0 && svc) {
        if (s->svc_start == 0) {
            s->svc_start = svc->start_handle;
            s->svc_end = svc->end_handle;
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        fail(s, error->status, "service discovery");
        return 0;
    }
    if (s->svc_start == 0) {
        fail(s, BLE_HS_ENOENT, "finding the MIDI service");
        return 0;
    }
    int rc = ble_gattc_disc_chrs_by_uuid(conn, s->svc_start, s->svc_end, &s_chr_uuid.u, on_chr, s);
    if (rc != 0) fail(s, rc, "characteristic discovery");
    return 0;
}

static int on_mtu(uint16_t conn, const struct ble_gatt_error *error, uint16_t mtu, void *arg)
{
    slot_t *s = (slot_t *)arg;
    if (s->conn != conn) return 0;
    if (error->status == 0) s->mtu = mtu;
    /* go on whatever the MTU exchange gave */
    int rc = ble_gattc_disc_svc_by_uuid(conn, &s_svc_uuid.u, on_svc, s);
    if (rc != 0) fail(s, rc, "service discovery");
    return 0;
}

static void slot_link_down(slot_t *s)
{
    s->status = BLE_MIDI_HOST_SLOT_FREE;
    s->conn = BLE_HS_CONN_HANDLE_NONE;
    s->mtu = 23;
    tx_clear(s);
    ble_midi_decoder_reset(&s->dec);
    ble_midi_sync_reset(&s->sync);
    if (s->rx) midi_timed_ring_clear(s->rx);
}

static void results_add(const struct ble_gap_disc_desc *d)
{
    struct ble_hs_adv_fields f;
    if (ble_hs_adv_parse_fields(&f, d->data, d->length_data) != 0) return;
    bool midi = false;
    for (int i = 0; i < f.num_uuids128; i++) {
        if (ble_uuid_cmp(&f.uuids128[i].u, &s_svc_uuid.u) == 0) midi = true;
    }

    portENTER_CRITICAL(&s_results_mux);
    int idx = -1;
    for (int i = 0; i < s_result_count; i++) {
        uint8_t val[6];
        to_msb(d->addr.val, val);
        if (memcmp(s_results[i].addr, val, 6) == 0) { idx = i; break; }
    }
    if (idx < 0 && midi && s_result_count < BLE_MIDI_HOST_MAX_RESULTS) {
        idx = s_result_count++;
        memset(&s_results[idx], 0, sizeof(s_results[idx]));
        to_msb(d->addr.val, s_results[idx].addr);
        s_results[idx].addr_type = d->addr.type;
    }
    if (idx >= 0) {
        s_results[idx].rssi = d->rssi;
        if (f.name && f.name_len > 0) {
            size_t n = f.name_len < sizeof(s_results[idx].name) - 1 ? f.name_len : sizeof(s_results[idx].name) - 1;
            memcpy(s_results[idx].name, f.name, n);
            s_results[idx].name[n] = '\0';
        }
    }
    portEXIT_CRITICAL(&s_results_mux);
}

/* A remembered peripheral showed up during a reconnect scan */
static void maybe_reconnect(const struct ble_gap_disc_desc *d)
{
    if (!s_auto || s_connecting >= 0 || s_connect_slot >= 0) return;
    portENTER_CRITICAL(&s_known_mux);
    int k = known_find(d->addr.val);
    portEXIT_CRITICAL(&s_known_mux);
    if (k < 0) return;
    for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        const slot_t *s = &s_slots[i];
        if ((s->status != BLE_MIDI_HOST_SLOT_FREE || s->parked) && s->used && addr_eq(&s->addr, d->addr.val)) return;
    }
    uint8_t msb[6];
    to_msb(d->addr.val, msb);
    BLE_MIDI_HOST_connect(msb, d->addr.type);
}

/* Scans and connection setup are claimed from the shared host
 * (nimble_owner.h); a script's BLE class takes them over */
static picoruby_nimble_client_t s_client;
static volatile int s_preempted = -1;      /* slot whose connect the BLE class cancelled */

static void central_done(void)
{
    if (!s_scanning && s_connecting < 0) {
        picoruby_nimble_unclaim(&s_client, PICORUBY_NIMBLE_PROC_CENTRAL);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        if (s_scan_is_user) results_add(&event->disc);
        else maybe_reconnect(&event->disc);
        break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = false;
        central_done();
        wake(NB_CONNECT);   /* a connect may wait for the scan to end */
        break;

    case BLE_GAP_EVENT_CONNECT: {
        slot_t *s = (slot_t *)arg;
        s_connecting = -1;
        central_done();
        if (s == NULL) break;
        if (event->connect.status != 0 && slot_index(s) == s_preempted) {
            /* the BLE class needed the central: try again when it is free */
            s_preempted = -1;
            s_connect_slot = slot_index(s);
            break;
        }
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "slot %d: connect failed (%d)", slot_index(s), event->connect.status);
            s->stats.last_error = event->connect.status;
            slot_link_down(s);
            break;
        }
        s->conn = event->connect.conn_handle;
        s->status = BLE_MIDI_HOST_SLOT_DISCOVERING;
        s->stats.connections++;
        s->svc_start = s->svc_end = s->val_handle = s->cccd_handle = 0;
        s->security_tried = false;
        read_conn(s);
        if (!s_wanted) {
            ble_gap_terminate(s->conn, BLE_ERR_REM_USER_CONN_TERM);
            break;
        }
        int rc = ble_gattc_exchange_mtu(s->conn, on_mtu, s);
        if (rc != 0) on_mtu(s->conn, &(struct ble_gatt_error){ .status = (uint16_t)rc }, 23, s);
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        slot_t *s = slot_by_conn(event->disconnect.conn.conn_handle);
        if (s) {
            ESP_LOGI(TAG, "slot %d: \"%s\" disconnected (reason 0x%03X)",
                     slot_index(s), s->name, event->disconnect.reason);
            slot_link_down(s);
            s_next_reconnect_us = esp_timer_get_time() + 1000000;   /* look again soon */
        }
        break;
    }

    case BLE_GAP_EVENT_CONN_UPDATE: {
        slot_t *s = slot_by_conn(event->conn_update.conn_handle);
        if (s) read_conn(s);
        break;
    }

    case BLE_GAP_EVENT_ENC_CHANGE: {
        slot_t *s = slot_by_conn(event->enc_change.conn_handle);
        if (s) {
            read_conn(s);
            if (s->status == BLE_MIDI_HOST_SLOT_SUBSCRIBING) {
                if (event->enc_change.status == 0) subscribe(s);
                else fail(s, event->enc_change.status, "pairing");
            }
        }
        break;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        slot_t *s = slot_by_conn(event->notify_rx.conn_handle);
        if (s == NULL || event->notify_rx.attr_handle != s->val_handle) break;
        static uint8_t buf[TX_BUF_MAX];   /* host task only */
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > sizeof(buf)) len = sizeof(buf);
        if (os_mbuf_copydata(event->notify_rx.om, 0, len, buf) != 0) break;
        s->stats.rx_packets++;
        s->rx_now = esp_timer_get_time();
        s->rx_budget = rx_budget(s);
        ble_midi_decode(&s->dec, buf, len, rx_push, s);
        midi_timed_ring_commit(s->rx);
        s->stats.decode_errors = s->dec.errors;
        break;
    }

    case BLE_GAP_EVENT_MTU: {
        slot_t *s = slot_by_conn(event->mtu.conn_handle);
        if (s) s->mtu = event->mtu.value;
        break;
    }

    default:
        break;
    }
    return 0;
}

/* ---------------------------------------------------------------------
 * The gem's task: scans, connects, TX
 * ------------------------------------------------------------------- */

static void start_scan(bool user, uint32_t ms)
{
    if (!s_held || !picoruby_nimble_synced()) {
        s_scanning = false;
        return;
    }
    if (!picoruby_nimble_claim(&s_client, PICORUBY_NIMBLE_PROC_CENTRAL)) {
        if (user) ESP_LOGW(TAG, "scan: the BLE class is scanning or connecting");
        s_scanning = false;
        return;
    }
    if (s_scanning) ble_gap_disc_cancel();
    struct ble_gap_disc_params p;
    memset(&p, 0, sizeof(p));
    p.passive = user ? 0 : 1;          /* names come in scan responses */
    p.filter_duplicates = 0;
    if (user) {
        p.itvl = 0x0050;               /* 50 ms */
        p.window = 0x0030;             /* 30 ms */
    } else {
        p.itvl = 0x00A0;               /* 100 ms */
        p.window = 0x0030;             /* 30 ms */
    }
    s_scan_is_user = user;
    int rc = ble_gap_disc(picoruby_nimble_own_addr_type(), (int32_t)ms, &p, gap_event, NULL);
    s_scanning = (rc == 0);
    if (rc != 0) {
        ESP_LOGW(TAG, "scan: %d", rc);
        central_done();
    }
}

static void do_connect(void)
{
    int i = s_connect_slot;
    if (i < 0 || s_connecting >= 0) return;
    slot_t *s = slot_at(i);
    if (s == NULL || !s_held || !picoruby_nimble_synced()) {
        s_connect_slot = -1;
        if (s) slot_link_down(s);
        return;
    }
    if (!picoruby_nimble_claim(&s_client, PICORUBY_NIMBLE_PROC_CENTRAL)) {
        return;     /* kept in s_connect_slot: proc_free wakes us */
    }
    if (s_scanning) {
        ble_gap_disc_cancel();
        s_scanning = false;
    }
    struct ble_gap_conn_params cp = {
        .scan_itvl = 0x0010,
        .scan_window = 0x0010,
        .itvl_min = BLE_MIDI_HOST_CONN_ITVL_MIN,
        .itvl_max = BLE_MIDI_HOST_CONN_ITVL_MAX,
        .latency = 0,
        .supervision_timeout = 200,
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    s_connecting = i;
    s_connect_slot = -1;
    int rc = ble_gap_connect(picoruby_nimble_own_addr_type(), &s->addr, 10000, &cp, gap_event, s);
    if (rc != 0) {
        ESP_LOGW(TAG, "slot %d: connect: %d", i, rc);
        s->stats.last_error = rc;
        s_connecting = -1;
        central_done();
        slot_link_down(s);
    }
}

static bool known_missing(void)
{
    if (!s_auto) return false;
    bool missing = false;
    bool free_slot = false;
    for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        if (s_slots[i].status == BLE_MIDI_HOST_SLOT_FREE) free_slot = true;
    }
    if (!free_slot) return false;
    portENTER_CRITICAL(&s_known_mux);
    for (int k = 0; k < s_known_count && !missing; k++) {
        bool here = false;
        for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
            const slot_t *s = &s_slots[i];
            if ((s->status != BLE_MIDI_HOST_SLOT_FREE || s->parked) && s->used
                && addr_eq(&s->addr, s_known[k].val)) here = true;
        }
        if (!here) missing = true;
    }
    portEXIT_CRITICAL(&s_known_mux);
    return missing;
}

static void client_configure(void)
{
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    /* The bond store is set up again on every host start (the host frees it
     * when it stops); joining a running host leaves it as it is */
    if (!picoruby_nimble_started()) ble_store_config_init();
}

static void client_on_reset(int reason)
{
    (void)reason;
    s_scanning = false;
    s_connecting = -1;
    s_preempted = -1;
    for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        if (s_slots[i].status != BLE_MIDI_HOST_SLOT_FREE) slot_link_down(&s_slots[i]);
    }
}

/* The BLE class took the central (its scan or connect) */
static void client_yield(int proc)
{
    if (proc != PICORUBY_NIMBLE_PROC_CENTRAL) return;
    if (s_scanning) {
        ble_gap_disc_cancel();
        s_scanning = false;
    }
    int c = s_connecting;
    if (c >= 0) {
        s_preempted = c;
        ble_gap_conn_cancel();      /* CONNECT (cancelled) puts it back in s_connect_slot */
    }
}

static void client_proc_free(int proc)
{
    if (proc == PICORUBY_NIMBLE_PROC_CENTRAL) wake(NB_CONNECT);
}

static picoruby_nimble_client_t s_client = {
    .name = "BLE-MIDI host",
    .configure = client_configure,
    .on_reset = client_on_reset,
    .yield = client_yield,
    .proc_free = client_proc_free,
};

static void do_enable(void)
{
    if (!s_wanted || s_held) return;
    int rc = picoruby_nimble_acquire(&s_client);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE host did not start");
        s_wanted = false;
        return;
    }
    s_held = true;
    s_next_reconnect_us = esp_timer_get_time();
    ESP_LOGI(TAG, "BLE-MIDI host ready (%d remembered)", s_known_count);
}

static void do_disable(void)
{
    if (s_wanted || !s_held) return;
    if (s_scanning) ble_gap_disc_cancel();
    s_scanning = false;
    if (s_connecting >= 0) ble_gap_conn_cancel();
    s_connecting = -1;
    central_done();
    for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        slot_t *s = &s_slots[i];
        if (s->status != BLE_MIDI_HOST_SLOT_FREE && s->status != BLE_MIDI_HOST_SLOT_CONNECTING) {
            ble_gap_terminate(s->conn, BLE_ERR_REM_USER_CONN_TERM);
        }
        slot_link_down(s);
    }
    s_held = false;
    picoruby_nimble_release(&s_client);
}

static void host_task(void *arg)
{
    (void)arg;
    bool again = false;
    for (;;) {
        uint32_t bits = 0;
        TickType_t wait = again ? 1 : pdMS_TO_TICKS(500);
        if (wait == 0) wait = 1;
        xTaskNotifyWait(0, UINT32_MAX, &bits, wait);

        if (bits & NB_ENABLE) do_enable();
        if (bits & NB_DISABLE) do_disable();
        if (bits & NB_SCAN_STOP) {
            if (s_scanning) ble_gap_disc_cancel();
            s_scanning = false;
            central_done();
        }
        if (bits & NB_SCAN) start_scan(true, s_scan_ms);
        if (s_connect_slot >= 0) do_connect();

        if (s_known_dirty) {
            s_known_dirty = false;
            known_save();
        }

        /* reconnect the remembered peripherals */
        if (s_held && !s_scanning && s_connecting < 0 && s_connect_slot < 0
            && esp_timer_get_time() >= s_next_reconnect_us && known_missing()) {
            start_scan(false, BLE_MIDI_HOST_RECONNECT_SCAN_MS);
            s_next_reconnect_us = esp_timer_get_time()
                + (int64_t)BLE_MIDI_HOST_RECONNECT_PERIOD_MS * 1000;
        }

        again = false;
        for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
            if (tx_flush(&s_slots[i])) again = true;
        }
    }
}

/* ---------------------------------------------------------------------
 * Transports
 * ------------------------------------------------------------------- */

static int t_send(void *ctx, uint8_t cable, uint8_t cin, uint8_t b1, uint8_t b2, uint8_t b3)
{
    return BLE_MIDI_HOST_dev_send_packet((int)(intptr_t)ctx, cable, cin, b1, b2, b3);
}
static int t_read(void *ctx, uint8_t *buf, size_t maxlen)
{
    return BLE_MIDI_HOST_dev_read_packet((int)(intptr_t)ctx, buf, maxlen);
}
static int t_avail(void *ctx) { return BLE_MIDI_HOST_dev_bytes_available((int)(intptr_t)ctx); }
static bool t_connected(void *ctx) { return BLE_MIDI_HOST_dev_connected((int)(intptr_t)ctx); }

static const midi_transport_ops_t s_ops = {
    .send_packet     = t_send,
    .read_bytes      = t_read,
    .bytes_available = t_avail,
    .is_connected    = t_connected,
    .transport_id    = MIDI_TRANSPORT_ID_BLE,
};

static void register_slot(int i)
{
    slot_t *s = &s_slots[i];
    if (s->bit != 0) return;
    s->transport.ops = &s_ops;
    s->transport.ctx = (void *)(intptr_t)i;
    s->bit = MIDI_transport_register(&s->transport);
    if (s->bit == 0) {
        ESP_LOGW(TAG, "MIDI transport registry full: BLE-MIDI slot %d is no transport", i);
        return;
    }
    MIDI_Input_add_packet_source(s->bit);
    MIDI_Input_track_clock(s->bit);   /* external BPM (Tempo's BLE source) */
    ESP_LOGI(TAG, "BLE-MIDI slot %d is transport 0x%04X", i, (unsigned)s->bit);
}

/* ---------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------- */

int BLE_MIDI_HOST_init(void)
{
    if (s_inited) return 0;
    s_slots = heap_caps_calloc(BLE_MIDI_HOST_MAX_DEVICES, sizeof(slot_t), MALLOC_CAP_8BIT);
    s_results = heap_caps_calloc(BLE_MIDI_HOST_MAX_RESULTS, sizeof(ble_midi_peer_t), MALLOC_CAP_8BIT);
    if (!s_slots || !s_results) goto fail;
    for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        slot_t *s = &s_slots[i];
        if (s->rx == NULL) s->rx = midi_timed_ring_new(BLE_MIDI_HOST_RX_PACKETS, "ble_midi_host_rx");
        s->tx = heap_caps_calloc(BLE_MIDI_HOST_TX_PACKETS, sizeof(tx_entry_t), MALLOC_CAP_8BIT);
        s->txbuf = heap_caps_malloc(TX_BUF_MAX, MALLOC_CAP_8BIT);
        if (!s->rx || !s->tx || !s->txbuf) goto fail;
        s->tx_mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
        s->conn = BLE_HS_CONN_HANDLE_NONE;
        s->mtu = 23;
        ble_midi_decoder_reset(&s->dec);
    }
    const esp_timer_create_args_t targs = { .callback = batch_timer_cb, .name = "ble_midi_host" };
    if (esp_timer_create(&targs, &s_batch_timer) != ESP_OK) goto fail;
    if (xTaskCreatePinnedToCore(host_task, "ble_midi_host", BLE_MIDI_HOST_TASK_STACK, NULL,
                                BLE_MIDI_HOST_TASK_PRIORITY, &s_task,
                                BLE_MIDI_HOST_TASK_CORE) != pdPASS) {
        esp_timer_delete(s_batch_timer);
        s_batch_timer = NULL;
        goto fail;
    }
    known_load();
    s_inited = true;
    return 0;

fail:
    ESP_LOGE(TAG, "out of memory");
    if (s_slots) {
        for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
            midi_timed_ring_free(s_slots[i].rx);
            heap_caps_free(s_slots[i].tx);
            heap_caps_free(s_slots[i].txbuf);
        }
    }
    heap_caps_free(s_slots); s_slots = NULL;
    heap_caps_free(s_results); s_results = NULL;
    return -1;
}

bool BLE_MIDI_HOST_available(void) { return true; }

int BLE_MIDI_HOST_enable(bool on)
{
    if (BLE_MIDI_HOST_init() != 0) return -1;
    s_wanted = on;
    wake(on ? NB_ENABLE : NB_DISABLE);
    return 0;
}

bool BLE_MIDI_HOST_is_enabled(void) { return s_wanted; }
bool BLE_MIDI_HOST_is_ready(void) { return s_held && picoruby_nimble_synced(); }

void BLE_MIDI_HOST_set_auto_connect(bool on)
{
    s_auto = on;
    if (on) s_next_reconnect_us = 0;
}
bool BLE_MIDI_HOST_auto_connect(void) { return s_auto; }

int BLE_MIDI_HOST_scan_start(uint32_t duration_ms)
{
    if (BLE_MIDI_HOST_init() != 0) return -1;
    if (!s_wanted) BLE_MIDI_HOST_enable(true);
    portENTER_CRITICAL(&s_results_mux);
    s_result_count = 0;
    portEXIT_CRITICAL(&s_results_mux);
    s_scan_ms = duration_ms ? duration_ms : 5000;
    s_scanning = true;      /* until the task has started it (or it ends) */
    s_scan_is_user = true;
    wake(NB_SCAN);
    return 0;
}

void BLE_MIDI_HOST_scan_stop(void) { wake(NB_SCAN_STOP); }
bool BLE_MIDI_HOST_scanning(void) { return s_scanning && s_scan_is_user; }

int BLE_MIDI_HOST_scan_results(ble_midi_peer_t *out, int max)
{
    if (s_results == NULL || out == NULL) return 0;
    int n;
    portENTER_CRITICAL(&s_results_mux);
    n = s_result_count < max ? s_result_count : max;
    memcpy(out, s_results, sizeof(ble_midi_peer_t) * (size_t)n);
    portEXIT_CRITICAL(&s_results_mux);
    portENTER_CRITICAL(&s_known_mux);
    for (int i = 0; i < n; i++) {
        uint8_t val[6];
        for (int k = 0; k < 6; k++) val[k] = out[i].addr[5 - k];
        out[i].known = known_find(val) >= 0;
    }
    portEXIT_CRITICAL(&s_known_mux);
    for (int i = 0; i < n; i++) {
        uint8_t val[6];
        for (int k = 0; k < 6; k++) val[k] = out[i].addr[5 - k];
        for (int j = 0; j < BLE_MIDI_HOST_MAX_DEVICES; j++) {
            if (s_slots[j].status != BLE_MIDI_HOST_SLOT_FREE && addr_eq(&s_slots[j].addr, val)) out[i].connected = true;
        }
    }
    return n;
}

/* Name for a peripheral from the scan results or the remembered list */
static void lookup_name(const uint8_t val[6], char *out, size_t cap)
{
    out[0] = '\0';
    uint8_t msb[6];
    to_msb(val, msb);
    portENTER_CRITICAL(&s_results_mux);
    for (int i = 0; i < s_result_count; i++) {
        if (memcmp(s_results[i].addr, msb, 6) == 0 && s_results[i].name[0]) {
            strncpy(out, s_results[i].name, cap - 1);
            out[cap - 1] = '\0';
        }
    }
    portEXIT_CRITICAL(&s_results_mux);
    if (out[0]) return;
    portENTER_CRITICAL(&s_known_mux);
    int k = known_find(val);
    if (k >= 0) {
        strncpy(out, s_known[k].name, cap - 1);
        out[cap - 1] = '\0';
    }
    portEXIT_CRITICAL(&s_known_mux);
    if (out[0] == '\0') snprintf(out, cap, "BLE %02X%02X", msb[4], msb[5]);
}

int BLE_MIDI_HOST_connect(const uint8_t addr[6], uint8_t addr_type)
{
    if (BLE_MIDI_HOST_init() != 0 || addr == NULL) return -1;
    if (!s_wanted) BLE_MIDI_HOST_enable(true);
    if (s_connect_slot >= 0) return -1;     /* one at a time */
    uint8_t val[6];
    for (int i = 0; i < 6; i++) val[i] = addr[5 - i];

    int pick = -1;
    for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        if (s_slots[i].used && addr_eq(&s_slots[i].addr, val)) {
            if (s_slots[i].status != BLE_MIDI_HOST_SLOT_FREE) return i;   /* already */
            pick = i;
            break;
        }
    }
    /* a slot never used, then one whose peripheral is gone */
    for (int i = 0; pick < 0 && i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        if (!s_slots[i].used) pick = i;
    }
    for (int i = 0; pick < 0 && i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        if (s_slots[i].status == BLE_MIDI_HOST_SLOT_FREE) pick = i;
    }
    if (pick < 0) return -1;

    /* A slot can pass to another peripheral; its routes are then dropped by
     * whoever lists the devices (Midori: ui_midi_devices.cpp), as for the
     * USB-MIDI host */
    slot_t *s = &s_slots[pick];
    s->used = true;
    s->parked = false;
    s->addr.type = addr_type;
    memcpy(s->addr.val, val, 6);
    lookup_name(val, s->name, sizeof(s->name));
    memset(&s->stats, 0, sizeof(s->stats));
    s->status = BLE_MIDI_HOST_SLOT_CONNECTING;
    register_slot(pick);
    s_connect_slot = pick;
    wake(NB_CONNECT);
    return pick;
}

void BLE_MIDI_HOST_disconnect(int slot)
{
    slot_t *s = slot_at(slot);
    if (s == NULL) return;
    s->parked = true;
    if (s->status == BLE_MIDI_HOST_SLOT_FREE) return;
    if (s->status == BLE_MIDI_HOST_SLOT_CONNECTING) {
        if (s_connecting == slot) ble_gap_conn_cancel();
        if (s_connect_slot == slot) s_connect_slot = -1;
        slot_link_down(s);
        return;
    }
    /* the DISCONNECT event frees the slot */
    if (s->conn != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(s->conn, BLE_ERR_REM_USER_CONN_TERM);
}

int BLE_MIDI_HOST_known(ble_midi_peer_t *out, int max)
{
    if (out == NULL) return 0;
    known_load();
    int n;
    portENTER_CRITICAL(&s_known_mux);
    n = s_known_count < max ? s_known_count : max;
    for (int i = 0; i < n; i++) {
        memset(&out[i], 0, sizeof(out[i]));
        to_msb(s_known[i].val, out[i].addr);
        out[i].addr_type = s_known[i].type;
        strncpy(out[i].name, s_known[i].name, sizeof(out[i].name) - 1);
        out[i].known = true;
    }
    portEXIT_CRITICAL(&s_known_mux);
    for (int i = 0; s_slots && i < n; i++) {
        uint8_t val[6];
        for (int k = 0; k < 6; k++) val[k] = out[i].addr[5 - k];
        for (int j = 0; j < BLE_MIDI_HOST_MAX_DEVICES; j++) {
            if (s_slots[j].status != BLE_MIDI_HOST_SLOT_FREE && addr_eq(&s_slots[j].addr, val)) out[i].connected = true;
        }
    }
    return n;
}

int BLE_MIDI_HOST_forget(const uint8_t addr[6])
{
    if (addr == NULL) return -1;
    known_load();
    uint8_t val[6];
    for (int i = 0; i < 6; i++) val[i] = addr[5 - i];
    for (int i = 0; s_slots && i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        if (s_slots[i].status != BLE_MIDI_HOST_SLOT_FREE && addr_eq(&s_slots[i].addr, val)) {
            BLE_MIDI_HOST_disconnect(i);
        }
    }
    int found = -1;
    uint8_t type = 0;
    portENTER_CRITICAL(&s_known_mux);
    int k = known_find(val);
    if (k >= 0) {
        found = k;
        type = s_known[k].type;
        memmove(&s_known[k], &s_known[k + 1], sizeof(known_t) * (size_t)(s_known_count - 1 - k));
        s_known_count--;
    }
    portEXIT_CRITICAL(&s_known_mux);
    if (found < 0) return -1;
    known_save();
    if (s_held) {
        ble_addr_t a = { .type = type };
        memcpy(a.val, val, 6);
        ble_gap_unpair(&a);
    }
    return 0;
}

int BLE_MIDI_HOST_dev_status(int slot)
{
    slot_t *s = slot_at(slot);
    return s ? (int)s->status : BLE_MIDI_HOST_SLOT_FREE;
}

midi_tmask_t BLE_MIDI_HOST_dev_bit(int slot)
{
    slot_t *s = slot_at(slot);
    return s ? s->bit : 0;
}

bool BLE_MIDI_HOST_dev_connected(int slot)
{
    slot_t *s = slot_at(slot);
    return s && s->status == BLE_MIDI_HOST_SLOT_CONNECTED;
}

int BLE_MIDI_HOST_dev_info(int slot, ble_midi_peer_t *out)
{
    slot_t *s = slot_at(slot);
    if (s == NULL || out == NULL || !s->used) return -1;
    memset(out, 0, sizeof(*out));
    to_msb(s->addr.val, out->addr);
    out->addr_type = s->addr.type;
    strncpy(out->name, s->name, sizeof(out->name) - 1);
    out->connected = s->status == BLE_MIDI_HOST_SLOT_CONNECTED;
    portENTER_CRITICAL(&s_known_mux);
    out->known = known_find(s->addr.val) >= 0;
    portEXIT_CRITICAL(&s_known_mux);
    return 0;
}

void BLE_MIDI_HOST_dev_stats(int slot, ble_midi_host_stats_t *out)
{
    if (out == NULL) return;
    slot_t *s = slot_at(slot);
    if (s == NULL) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = s->stats;
    out->status = s->status;
    out->mtu = s->mtu;
    out->rx_late = s->sync.late;
    out->rx_latency_us = rx_budget(s);
}

int BLE_MIDI_HOST_dev_send_packet(int slot, uint8_t cable, uint8_t cin,
                                  uint8_t b1, uint8_t b2, uint8_t b3)
{
    (void)cable;
    slot_t *s = slot_at(slot);
    if (s == NULL) return -1;
    if (s->status != BLE_MIDI_HOST_SLOT_CONNECTED) {
        s->stats.tx_dropped++;
        return -1;
    }
    uint16_t ts = now_ts();
    portENTER_CRITICAL(&s->tx_mux);
    uint32_t head = s->tx_head;
    uint32_t next = (head + 1) & TX_MASK;
    if (next == s->tx_tail) {
        portEXIT_CRITICAL(&s->tx_mux);
        s->stats.tx_dropped++;
        return -1;
    }
    s->tx[head].p[0] = cin & 0x0F;
    s->tx[head].p[1] = b1;
    s->tx[head].p[2] = b2;
    s->tx[head].p[3] = b3;
    s->tx[head].ts = ts;
    s->tx_head = next;
    portEXIT_CRITICAL(&s->tx_mux);
    s->stats.tx_messages++;
    if (!esp_timer_is_active(s_batch_timer)) {
        esp_timer_start_once(s_batch_timer, BLE_MIDI_HOST_TX_BATCH_US);
    }
    return 0;
}

int BLE_MIDI_HOST_dev_bytes_available(int slot)
{
    slot_t *s = slot_at(slot);
    if (s == NULL) return 0;
    return s->rx ? midi_timed_ring_bytes_due(s->rx) : 0;
}

int BLE_MIDI_HOST_dev_read_packet(int slot, uint8_t *buf, size_t maxlen)
{
    slot_t *s = slot_at(slot);
    if (s == NULL || s->rx == NULL) return 0;
    return midi_timed_ring_read(s->rx, buf, maxlen);
}

void BLE_MIDI_HOST_set_rx_latency(int32_t us)
{
    if (us < -1) us = -1;
    if (us > 100000) us = 100000;
    s_rx_latency = us;
}

int32_t BLE_MIDI_HOST_rx_latency_setting(void) { return s_rx_latency; }

static bool contains_ci(const char *hay, const char *needle)
{
    size_t n = strlen(needle);
    if (n == 0) return true;
    for (; *hay; hay++) {
        size_t i = 0;
        while (i < n && hay[i] && tolower((unsigned char)hay[i]) == tolower((unsigned char)needle[i])) i++;
        if (i == n) return true;
    }
    return false;
}

int BLE_MIDI_HOST_find(const char *name)
{
    if (s_slots == NULL || name == NULL) return -1;
    for (int i = 0; i < BLE_MIDI_HOST_MAX_DEVICES; i++) {
        if (s_slots[i].status == BLE_MIDI_HOST_SLOT_CONNECTED && contains_ci(s_slots[i].name, name)) return i;
    }
    return -1;
}

#else  /* !BLE_MIDI_HOST_ENABLED */

#include <string.h>
#include <stdio.h>

int  BLE_MIDI_HOST_init(void) { return -1; }
bool BLE_MIDI_HOST_available(void) { return false; }
int  BLE_MIDI_HOST_enable(bool on) { (void)on; return -1; }
bool BLE_MIDI_HOST_is_enabled(void) { return false; }
bool BLE_MIDI_HOST_is_ready(void) { return false; }
void BLE_MIDI_HOST_set_auto_connect(bool on) { (void)on; }
bool BLE_MIDI_HOST_auto_connect(void) { return false; }
int  BLE_MIDI_HOST_scan_start(uint32_t duration_ms) { (void)duration_ms; return -1; }
void BLE_MIDI_HOST_scan_stop(void) {}
bool BLE_MIDI_HOST_scanning(void) { return false; }
int  BLE_MIDI_HOST_scan_results(ble_midi_peer_t *out, int max) { (void)out; (void)max; return 0; }
int  BLE_MIDI_HOST_connect(const uint8_t addr[6], uint8_t addr_type) { (void)addr; (void)addr_type; return -1; }
void BLE_MIDI_HOST_disconnect(int slot) { (void)slot; }
int  BLE_MIDI_HOST_known(ble_midi_peer_t *out, int max) { (void)out; (void)max; return 0; }
int  BLE_MIDI_HOST_forget(const uint8_t addr[6]) { (void)addr; return -1; }
void BLE_MIDI_HOST_set_rx_latency(int32_t us) { (void)us; }
int32_t BLE_MIDI_HOST_rx_latency_setting(void) { return 0; }
int  BLE_MIDI_HOST_dev_status(int slot) { (void)slot; return BLE_MIDI_HOST_SLOT_FREE; }
midi_tmask_t BLE_MIDI_HOST_dev_bit(int slot) { (void)slot; return 0; }
bool BLE_MIDI_HOST_dev_connected(int slot) { (void)slot; return false; }
int  BLE_MIDI_HOST_dev_info(int slot, ble_midi_peer_t *out) { (void)slot; (void)out; return -1; }
void BLE_MIDI_HOST_dev_stats(int slot, ble_midi_host_stats_t *out) { (void)slot; if (out) memset(out, 0, sizeof(*out)); }
int  BLE_MIDI_HOST_dev_send_packet(int slot, uint8_t cable, uint8_t cin, uint8_t b1, uint8_t b2, uint8_t b3)
{
    (void)slot; (void)cable; (void)cin; (void)b1; (void)b2; (void)b3;
    return -1;
}
int  BLE_MIDI_HOST_dev_bytes_available(int slot) { (void)slot; return 0; }
int  BLE_MIDI_HOST_dev_read_packet(int slot, uint8_t *buf, size_t maxlen) { (void)slot; (void)buf; (void)maxlen; return 0; }
int  BLE_MIDI_HOST_find(const char *name) { (void)name; return -1; }

void BLE_MIDI_HOST_format_addr(const uint8_t a[6], char out[18])
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", a[0], a[1], a[2], a[3], a[4], a[5]);
}

int BLE_MIDI_HOST_parse_addr(const char *s, uint8_t a[6]) { (void)s; (void)a; return -1; }

#endif /* BLE_MIDI_HOST_ENABLED */
