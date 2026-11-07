/*
 * PicoRuby BLE-MIDI Host - mruby/c bindings
 *
 * mruby/c does not tell class methods from instance methods: the class
 * methods of the mruby binding are plain methods here.
 */

#include <stdio.h>
#include <string.h>

#include <mrubyc.h>

#include "../../include/ble_midi_host.h"

static const char *const STATUS_NAMES[] = {
    "free", "connecting", "discovering", "subscribing", "connected",
};

#define RET_BOOL(x) do { if (x) { SET_TRUE_RETURN(); } else { SET_FALSE_RETURN(); } } while (0)
#define ARG_INT(n)  ((argc >= (n) && v[n].tt == MRBC_TT_INTEGER) ? (int)v[n].i : 0)
#define ARG_TRUE(n) (argc >= (n) && v[n].tt == MRBC_TT_TRUE)
#define ARG_STR(n)  ((argc >= (n) && v[n].tt == MRBC_TT_STRING) ? (const char *)mrbc_string_cstr(&v[n]) : NULL)

static void hset(mrbc_value *h, const char *key, mrbc_value val)
{
    mrbc_value k = mrbc_symbol_value(mrbc_str_to_symid(key));
    mrbc_hash_set(h, &k, &val);
}

static mrbc_value peer_hash(mrbc_vm *vm, const ble_midi_peer_t *p, bool with_rssi)
{
    char addr[18];
    BLE_MIDI_HOST_format_addr(p->addr, addr);
    mrbc_value h = mrbc_hash_new(vm, 6);
    hset(&h, "name", mrbc_string_new_cstr(vm, p->name));
    hset(&h, "address", mrbc_string_new_cstr(vm, addr));
    hset(&h, "type", mrbc_integer_value(p->addr_type));
    if (with_rssi) hset(&h, "rssi", mrbc_integer_value(p->rssi));
    hset(&h, "known", p->known ? mrbc_true_value() : mrbc_false_value());
    hset(&h, "connected", p->connected ? mrbc_true_value() : mrbc_false_value());
    return h;
}

static void c_init(mrbc_vm *vm, mrbc_value v[], int argc)        { SET_INT_RETURN(BLE_MIDI_HOST_init()); }
static void c_available(mrbc_vm *vm, mrbc_value v[], int argc)   { RET_BOOL(BLE_MIDI_HOST_available()); }
static void c_enable(mrbc_vm *vm, mrbc_value v[], int argc)      { RET_BOOL(BLE_MIDI_HOST_enable(ARG_TRUE(1)) == 0); }
static void c_enabled(mrbc_vm *vm, mrbc_value v[], int argc)     { RET_BOOL(BLE_MIDI_HOST_is_enabled()); }
static void c_ready(mrbc_vm *vm, mrbc_value v[], int argc)       { RET_BOOL(BLE_MIDI_HOST_is_ready()); }
static void c_set_auto(mrbc_vm *vm, mrbc_value v[], int argc)    { BLE_MIDI_HOST_set_auto_connect(ARG_TRUE(1)); RET_BOOL(ARG_TRUE(1)); }
static void c_rx_latency(mrbc_vm *vm, mrbc_value v[], int argc)  { SET_INT_RETURN(BLE_MIDI_HOST_rx_latency_setting()); }
static void c_set_rx_latency(mrbc_vm *vm, mrbc_value v[], int argc)
{
    if (argc >= 1 && v[1].tt == MRBC_TT_INTEGER) BLE_MIDI_HOST_set_rx_latency((int32_t)v[1].i);
    SET_INT_RETURN(BLE_MIDI_HOST_rx_latency_setting());
}
static void c_auto(mrbc_vm *vm, mrbc_value v[], int argc)        { RET_BOOL(BLE_MIDI_HOST_auto_connect()); }
static void c_scan_start(mrbc_vm *vm, mrbc_value v[], int argc)  { RET_BOOL(BLE_MIDI_HOST_scan_start((uint32_t)ARG_INT(1)) == 0); }
static void c_scan_stop(mrbc_vm *vm, mrbc_value v[], int argc)   { BLE_MIDI_HOST_scan_stop(); SET_NIL_RETURN(); }
static void c_scanning(mrbc_vm *vm, mrbc_value v[], int argc)    { RET_BOOL(BLE_MIDI_HOST_scanning()); }
static void c_max_devices(mrbc_vm *vm, mrbc_value v[], int argc) { SET_INT_RETURN(BLE_MIDI_HOST_MAX_DEVICES); }

static void c_scan_results(mrbc_vm *vm, mrbc_value v[], int argc)
{
    ble_midi_peer_t peers[BLE_MIDI_HOST_MAX_RESULTS];
    int n = BLE_MIDI_HOST_scan_results(peers, BLE_MIDI_HOST_MAX_RESULTS);
    mrbc_value ary = mrbc_array_new(vm, n);
    for (int i = 0; i < n; i++) {
        mrbc_value h = peer_hash(vm, &peers[i], true);
        mrbc_array_push(&ary, &h);
    }
    SET_RETURN(ary);
}

static void c_known(mrbc_vm *vm, mrbc_value v[], int argc)
{
    ble_midi_peer_t peers[BLE_MIDI_HOST_MAX_KNOWN];
    int n = BLE_MIDI_HOST_known(peers, BLE_MIDI_HOST_MAX_KNOWN);
    mrbc_value ary = mrbc_array_new(vm, n);
    for (int i = 0; i < n; i++) {
        mrbc_value h = peer_hash(vm, &peers[i], false);
        mrbc_array_push(&ary, &h);
    }
    SET_RETURN(ary);
}

static void c_connect(mrbc_vm *vm, mrbc_value v[], int argc)
{
    uint8_t addr[6];
    if (BLE_MIDI_HOST_parse_addr(ARG_STR(1), addr) != 0) { SET_INT_RETURN(-1); return; }
    SET_INT_RETURN(BLE_MIDI_HOST_connect(addr, (uint8_t)ARG_INT(2)));
}

static void c_forget(mrbc_vm *vm, mrbc_value v[], int argc)
{
    uint8_t addr[6];
    if (BLE_MIDI_HOST_parse_addr(ARG_STR(1), addr) != 0) { SET_FALSE_RETURN(); return; }
    RET_BOOL(BLE_MIDI_HOST_forget(addr) == 0);
}

static void c_find(mrbc_vm *vm, mrbc_value v[], int argc)
{
    const char *name = ARG_STR(1);
    SET_INT_RETURN(name ? BLE_MIDI_HOST_find(name) : -1);
}

static void c_status(mrbc_vm *vm, mrbc_value v[], int argc)
{
    int st = BLE_MIDI_HOST_dev_status(ARG_INT(1));
    if (st < 0 || st > 4) st = 0;
    mrbc_value sym = mrbc_symbol_value(mrbc_str_to_symid(STATUS_NAMES[st]));
    SET_RETURN(sym);
}

static void c_bit(mrbc_vm *vm, mrbc_value v[], int argc)       { SET_INT_RETURN((mrbc_int_t)BLE_MIDI_HOST_dev_bit(ARG_INT(1))); }
static void c_connected(mrbc_vm *vm, mrbc_value v[], int argc) { RET_BOOL(BLE_MIDI_HOST_dev_connected(ARG_INT(1))); }

static void c_dev_info(mrbc_vm *vm, mrbc_value v[], int argc)
{
    int slot = ARG_INT(1);
    ble_midi_peer_t p;
    if (BLE_MIDI_HOST_dev_info(slot, &p) != 0) { SET_NIL_RETURN(); return; }
    mrbc_value h = peer_hash(vm, &p, false);
    ble_midi_host_stats_t st;
    BLE_MIDI_HOST_dev_stats(slot, &st);
    int s = (int)st.status;
    if (s < 0 || s > 4) s = 0;
    hset(&h, "status", mrbc_symbol_value(mrbc_str_to_symid(STATUS_NAMES[s])));
    hset(&h, "mtu", mrbc_integer_value(st.mtu));
    hset(&h, "rx_messages", mrbc_integer_value(st.rx_messages));
    hset(&h, "rx_late", mrbc_integer_value(st.rx_late));
    hset(&h, "tx_messages", mrbc_integer_value(st.tx_messages));
    hset(&h, "tx_dropped", mrbc_integer_value(st.tx_dropped));
    hset(&h, "last_error", mrbc_integer_value(st.last_error));
    SET_RETURN(h);
}

static void c_send_packet(mrbc_vm *vm, mrbc_value v[], int argc)
{
    if (argc != 6) { SET_INT_RETURN(-1); return; }
    SET_INT_RETURN(BLE_MIDI_HOST_dev_send_packet(ARG_INT(1), (uint8_t)ARG_INT(2), (uint8_t)ARG_INT(3),
                                                 (uint8_t)ARG_INT(4), (uint8_t)ARG_INT(5), (uint8_t)ARG_INT(6)));
}

static void c_bytes_available(mrbc_vm *vm, mrbc_value v[], int argc)
{
    SET_INT_RETURN(BLE_MIDI_HOST_dev_bytes_available(ARG_INT(1)));
}

static void c_read_available(mrbc_vm *vm, mrbc_value v[], int argc)
{
    uint8_t buf[64];
    int n = BLE_MIDI_HOST_dev_read_packet(ARG_INT(1), buf, sizeof(buf));
    if (n <= 0) { SET_NIL_RETURN(); return; }
    mrbc_value s = mrbc_string_new(vm, buf, n);
    SET_RETURN(s);
}

static void c_disconnect(mrbc_vm *vm, mrbc_value v[], int argc)
{
    BLE_MIDI_HOST_disconnect(ARG_INT(1));
    SET_NIL_RETURN();
}

void
mrbc_ble_midi_host_init(mrbc_vm *vm)
{
    mrbc_class *cls = mrbc_define_class(vm, "BLE_MIDI_HOST", mrbc_class_object);
    mrbc_define_method(vm, cls, "_init",            c_init);
    mrbc_define_method(vm, cls, "_available",       c_available);
    mrbc_define_method(vm, cls, "_enable",          c_enable);
    mrbc_define_method(vm, cls, "_enabled",         c_enabled);
    mrbc_define_method(vm, cls, "_ready",           c_ready);
    mrbc_define_method(vm, cls, "_set_auto",        c_set_auto);
    mrbc_define_method(vm, cls, "_rx_latency",      c_rx_latency);
    mrbc_define_method(vm, cls, "_set_rx_latency",  c_set_rx_latency);
    mrbc_define_method(vm, cls, "_auto",            c_auto);
    mrbc_define_method(vm, cls, "_scan_start",      c_scan_start);
    mrbc_define_method(vm, cls, "_scan_stop",       c_scan_stop);
    mrbc_define_method(vm, cls, "_scanning",        c_scanning);
    mrbc_define_method(vm, cls, "_scan_results",    c_scan_results);
    mrbc_define_method(vm, cls, "_known",           c_known);
    mrbc_define_method(vm, cls, "_connect",         c_connect);
    mrbc_define_method(vm, cls, "_forget",          c_forget);
    mrbc_define_method(vm, cls, "_find",            c_find);
    mrbc_define_method(vm, cls, "_max_devices",     c_max_devices);
    mrbc_define_method(vm, cls, "_status",          c_status);
    mrbc_define_method(vm, cls, "_bit",             c_bit);
    mrbc_define_method(vm, cls, "_connected",       c_connected);
    mrbc_define_method(vm, cls, "_dev_info",        c_dev_info);
    mrbc_define_method(vm, cls, "_send_packet",     c_send_packet);
    mrbc_define_method(vm, cls, "_bytes_available", c_bytes_available);
    mrbc_define_method(vm, cls, "_read_available",  c_read_available);
    mrbc_define_method(vm, cls, "_disconnect",      c_disconnect);
}
