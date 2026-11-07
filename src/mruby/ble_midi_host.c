/*
 * PicoRuby BLE-MIDI Host - mruby bindings
 */

#include <string.h>
#include <stdio.h>

#include <mruby.h>
#include <mruby/presym.h>
#include <mruby/class.h>
#include <mruby/string.h>
#include <mruby/hash.h>
#include <mruby/array.h>

#include "../../include/ble_midi_host.h"

static const char *const STATUS_NAMES[] = {
    "free", "connecting", "discovering", "subscribing", "connected",
};

static void
hset(mrb_state *mrb, mrb_value h, const char *key, mrb_value v)
{
    mrb_hash_set(mrb, h, mrb_symbol_value(mrb_intern_cstr(mrb, key)), v);
}

static mrb_value
peer_hash(mrb_state *mrb, const ble_midi_peer_t *p, bool with_rssi)
{
    char addr[18];
    BLE_MIDI_HOST_format_addr(p->addr, addr);
    mrb_value h = mrb_hash_new(mrb);
    hset(mrb, h, "name", mrb_str_new_cstr(mrb, p->name));
    hset(mrb, h, "address", mrb_str_new_cstr(mrb, addr));
    hset(mrb, h, "type", mrb_fixnum_value(p->addr_type));
    if (with_rssi) hset(mrb, h, "rssi", mrb_fixnum_value(p->rssi));
    hset(mrb, h, "known", mrb_bool_value(p->known));
    hset(mrb, h, "connected", mrb_bool_value(p->connected));
    return h;
}

static mrb_value
mrb_bmh_init(mrb_state *mrb, mrb_value self)
{
    return mrb_fixnum_value(BLE_MIDI_HOST_init());
}

static mrb_value
mrb_bmh_available(mrb_state *mrb, mrb_value self)
{
    return mrb_bool_value(BLE_MIDI_HOST_available());
}

static mrb_value
mrb_bmh_enable(mrb_state *mrb, mrb_value self)
{
    mrb_bool on;
    mrb_get_args(mrb, "b", &on);
    return mrb_bool_value(BLE_MIDI_HOST_enable(on) == 0);
}

static mrb_value
mrb_bmh_enabled(mrb_state *mrb, mrb_value self)
{
    return mrb_bool_value(BLE_MIDI_HOST_is_enabled());
}

static mrb_value
mrb_bmh_ready(mrb_state *mrb, mrb_value self)
{
    return mrb_bool_value(BLE_MIDI_HOST_is_ready());
}

static mrb_value
mrb_bmh_set_auto(mrb_state *mrb, mrb_value self)
{
    mrb_bool on;
    mrb_get_args(mrb, "b", &on);
    BLE_MIDI_HOST_set_auto_connect(on);
    return mrb_bool_value(on);
}

static mrb_value
mrb_bmh_rx_latency(mrb_state *mrb, mrb_value self)
{
    return mrb_fixnum_value(BLE_MIDI_HOST_rx_latency_setting());
}

static mrb_value
mrb_bmh_set_rx_latency(mrb_state *mrb, mrb_value self)
{
    mrb_int us;
    mrb_get_args(mrb, "i", &us);
    BLE_MIDI_HOST_set_rx_latency((int32_t)us);
    return mrb_fixnum_value(BLE_MIDI_HOST_rx_latency_setting());
}

static mrb_value
mrb_bmh_auto(mrb_state *mrb, mrb_value self)
{
    return mrb_bool_value(BLE_MIDI_HOST_auto_connect());
}

static mrb_value
mrb_bmh_scan_start(mrb_state *mrb, mrb_value self)
{
    mrb_int ms;
    mrb_get_args(mrb, "i", &ms);
    return mrb_bool_value(BLE_MIDI_HOST_scan_start(ms > 0 ? (uint32_t)ms : 0) == 0);
}

static mrb_value
mrb_bmh_scan_stop(mrb_state *mrb, mrb_value self)
{
    BLE_MIDI_HOST_scan_stop();
    return mrb_nil_value();
}

static mrb_value
mrb_bmh_scanning(mrb_state *mrb, mrb_value self)
{
    return mrb_bool_value(BLE_MIDI_HOST_scanning());
}

static mrb_value
mrb_bmh_scan_results(mrb_state *mrb, mrb_value self)
{
    ble_midi_peer_t peers[BLE_MIDI_HOST_MAX_RESULTS];
    int n = BLE_MIDI_HOST_scan_results(peers, BLE_MIDI_HOST_MAX_RESULTS);
    mrb_value ary = mrb_ary_new_capa(mrb, n);
    for (int i = 0; i < n; i++) {
        int ai = mrb_gc_arena_save(mrb);
        mrb_ary_push(mrb, ary, peer_hash(mrb, &peers[i], true));
        mrb_gc_arena_restore(mrb, ai);
    }
    return ary;
}

static mrb_value
mrb_bmh_known(mrb_state *mrb, mrb_value self)
{
    ble_midi_peer_t peers[BLE_MIDI_HOST_MAX_KNOWN];
    int n = BLE_MIDI_HOST_known(peers, BLE_MIDI_HOST_MAX_KNOWN);
    mrb_value ary = mrb_ary_new_capa(mrb, n);
    for (int i = 0; i < n; i++) {
        int ai = mrb_gc_arena_save(mrb);
        mrb_ary_push(mrb, ary, peer_hash(mrb, &peers[i], false));
        mrb_gc_arena_restore(mrb, ai);
    }
    return ary;
}

/* BLE_MIDI_HOST._connect(address, type) => slot / -1 */
static mrb_value
mrb_bmh_connect(mrb_state *mrb, mrb_value self)
{
    const char *addr_s;
    mrb_int type;
    mrb_get_args(mrb, "zi", &addr_s, &type);
    uint8_t addr[6];
    if (BLE_MIDI_HOST_parse_addr(addr_s, addr) != 0) {
        mrb_raisef(mrb, E_ARGUMENT_ERROR, "bad BLE address: %s", addr_s);
    }
    return mrb_fixnum_value(BLE_MIDI_HOST_connect(addr, (uint8_t)type));
}

static mrb_value
mrb_bmh_forget(mrb_state *mrb, mrb_value self)
{
    const char *addr_s;
    mrb_get_args(mrb, "z", &addr_s);
    uint8_t addr[6];
    if (BLE_MIDI_HOST_parse_addr(addr_s, addr) != 0) return mrb_false_value();
    return mrb_bool_value(BLE_MIDI_HOST_forget(addr) == 0);
}

static mrb_value
mrb_bmh_find(mrb_state *mrb, mrb_value self)
{
    const char *name;
    mrb_get_args(mrb, "z", &name);
    return mrb_fixnum_value(BLE_MIDI_HOST_find(name));
}

static mrb_value
mrb_bmh_max_devices(mrb_state *mrb, mrb_value self)
{
    return mrb_fixnum_value(BLE_MIDI_HOST_MAX_DEVICES);
}

static mrb_value
mrb_bmh_status(mrb_state *mrb, mrb_value self)
{
    mrb_int slot;
    mrb_get_args(mrb, "i", &slot);
    int st = BLE_MIDI_HOST_dev_status((int)slot);
    if (st < 0 || st > 4) st = 0;
    return mrb_symbol_value(mrb_intern_cstr(mrb, STATUS_NAMES[st]));
}

static mrb_value
mrb_bmh_bit(mrb_state *mrb, mrb_value self)
{
    mrb_int slot;
    mrb_get_args(mrb, "i", &slot);
    return mrb_fixnum_value((mrb_int)BLE_MIDI_HOST_dev_bit((int)slot));
}

static mrb_value
mrb_bmh_connected(mrb_state *mrb, mrb_value self)
{
    mrb_int slot;
    mrb_get_args(mrb, "i", &slot);
    return mrb_bool_value(BLE_MIDI_HOST_dev_connected((int)slot));
}

static mrb_value
mrb_bmh_dev_info(mrb_state *mrb, mrb_value self)
{
    mrb_int slot;
    mrb_get_args(mrb, "i", &slot);
    ble_midi_peer_t p;
    if (BLE_MIDI_HOST_dev_info((int)slot, &p) != 0) return mrb_nil_value();
    mrb_value h = peer_hash(mrb, &p, false);
    ble_midi_host_stats_t st;
    BLE_MIDI_HOST_dev_stats((int)slot, &st);
    int s = (int)st.status;
    if (s < 0 || s > 4) s = 0;
    hset(mrb, h, "status", mrb_symbol_value(mrb_intern_cstr(mrb, STATUS_NAMES[s])));
    hset(mrb, h, "encrypted", mrb_bool_value(st.encrypted));
    hset(mrb, h, "bonded", mrb_bool_value(st.bonded));
    hset(mrb, h, "mtu", mrb_fixnum_value(st.mtu));
    hset(mrb, h, "interval_ms", mrb_float_value(mrb, st.interval_1250us * 1.25));
    hset(mrb, h, "rx_packets", mrb_fixnum_value(st.rx_packets));
    hset(mrb, h, "rx_messages", mrb_fixnum_value(st.rx_messages));
    hset(mrb, h, "rx_dropped", mrb_fixnum_value(st.rx_dropped));
    hset(mrb, h, "decode_errors", mrb_fixnum_value(st.decode_errors));
    hset(mrb, h, "rx_late", mrb_fixnum_value(st.rx_late));
    hset(mrb, h, "rx_latency_ms", mrb_float_value(mrb, st.rx_latency_us / 1000.0));
    hset(mrb, h, "tx_messages", mrb_fixnum_value(st.tx_messages));
    hset(mrb, h, "tx_packets", mrb_fixnum_value(st.tx_packets));
    hset(mrb, h, "tx_dropped", mrb_fixnum_value(st.tx_dropped));
    hset(mrb, h, "tx_retries", mrb_fixnum_value(st.tx_retries));
    hset(mrb, h, "connections", mrb_fixnum_value(st.connections));
    hset(mrb, h, "last_error", mrb_fixnum_value(st.last_error));
    return h;
}

static mrb_value
mrb_bmh_send_packet(mrb_state *mrb, mrb_value self)
{
    mrb_int slot, cable, cin, b1, b2, b3;
    mrb_get_args(mrb, "iiiiii", &slot, &cable, &cin, &b1, &b2, &b3);
    return mrb_fixnum_value(BLE_MIDI_HOST_dev_send_packet((int)slot, (uint8_t)cable, (uint8_t)cin,
                                                          (uint8_t)b1, (uint8_t)b2, (uint8_t)b3));
}

static mrb_value
mrb_bmh_bytes_available(mrb_state *mrb, mrb_value self)
{
    mrb_int slot;
    mrb_get_args(mrb, "i", &slot);
    return mrb_fixnum_value(BLE_MIDI_HOST_dev_bytes_available((int)slot));
}

static mrb_value
mrb_bmh_read_available(mrb_state *mrb, mrb_value self)
{
    mrb_int slot;
    mrb_get_args(mrb, "i", &slot);
    uint8_t buf[64];
    int n = BLE_MIDI_HOST_dev_read_packet((int)slot, buf, sizeof(buf));
    if (n <= 0) return mrb_nil_value();
    return mrb_str_new(mrb, (const char *)buf, n);
}

static mrb_value
mrb_bmh_disconnect(mrb_state *mrb, mrb_value self)
{
    mrb_int slot;
    mrb_get_args(mrb, "i", &slot);
    BLE_MIDI_HOST_disconnect((int)slot);
    return mrb_nil_value();
}

void
mrb_picoruby_ble_midi_host_gem_init(mrb_state *mrb)
{
    struct RClass *cls = mrb_define_class_id(mrb, MRB_SYM(BLE_MIDI_HOST), mrb->object_class);

    mrb_define_class_method_id(mrb, cls, MRB_SYM(_init),            mrb_bmh_init,            MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_available),       mrb_bmh_available,       MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_enable),          mrb_bmh_enable,          MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_enabled),         mrb_bmh_enabled,         MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_ready),           mrb_bmh_ready,           MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_set_auto),        mrb_bmh_set_auto,        MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_rx_latency),      mrb_bmh_rx_latency,      MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_set_rx_latency),  mrb_bmh_set_rx_latency,  MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_auto),            mrb_bmh_auto,            MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_scan_start),      mrb_bmh_scan_start,      MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_scan_stop),       mrb_bmh_scan_stop,       MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_scanning),        mrb_bmh_scanning,        MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_scan_results),    mrb_bmh_scan_results,    MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_known),           mrb_bmh_known,           MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_connect),         mrb_bmh_connect,         MRB_ARGS_REQ(2));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_forget),          mrb_bmh_forget,          MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_find),            mrb_bmh_find,            MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_max_devices),     mrb_bmh_max_devices,     MRB_ARGS_NONE());
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_status),          mrb_bmh_status,          MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_bit),             mrb_bmh_bit,             MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_connected),       mrb_bmh_connected,       MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_dev_info),        mrb_bmh_dev_info,        MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_send_packet),     mrb_bmh_send_packet,     MRB_ARGS_REQ(6));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_bytes_available), mrb_bmh_bytes_available, MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_read_available),  mrb_bmh_read_available,  MRB_ARGS_REQ(1));
    mrb_define_class_method_id(mrb, cls, MRB_SYM(_disconnect),      mrb_bmh_disconnect,      MRB_ARGS_REQ(1));
}

void
mrb_picoruby_ble_midi_host_gem_final(mrb_state *mrb)
{
    /* Links stay up across scripts (process-wide C state) */
}
