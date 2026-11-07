/*
 * PicoRuby BLE-MIDI Host - build-time configuration
 *
 * Injected by the host project as compile definitions (Midori:
 * components/midori-runtime/CMakeLists.txt).
 */

#ifndef BLE_MIDI_HOST_CONFIG_DEFINED_H_
#define BLE_MIDI_HOST_CONFIG_DEFINED_H_

/* Built with BLE-MIDI host support (needs NimBLE with the central role and
 * picoruby-ble's ESP32 port). Without it the port compiles to stubs. */
#ifndef BLE_MIDI_HOST_ENABLED
#define BLE_MIDI_HOST_ENABLED 0
#endif

/* Peripherals at once. Each takes a bit of picoruby-midi's transport
 * registry (16 in all) and a NimBLE connection
 * (CONFIG_BT_NIMBLE_MAX_CONNECTIONS). */
#ifndef BLE_MIDI_HOST_MAX_DEVICES
#define BLE_MIDI_HOST_MAX_DEVICES 2
#endif

/* Remembered peripherals (NVS namespace "ble_midi", key "known") */
#ifndef BLE_MIDI_HOST_MAX_KNOWN
#define BLE_MIDI_HOST_MAX_KNOWN 8
#endif

/* Scan results kept */
#ifndef BLE_MIDI_HOST_MAX_RESULTS
#define BLE_MIDI_HOST_MAX_RESULTS 16
#endif

/* Connection interval asked for, 1.25 ms units (BLE-MIDI: 15 ms or less) */
#ifndef BLE_MIDI_HOST_CONN_ITVL_MIN
#define BLE_MIDI_HOST_CONN_ITVL_MIN 6       /* 7.5 ms */
#endif
#ifndef BLE_MIDI_HOST_CONN_ITVL_MAX
#define BLE_MIDI_HOST_CONN_ITVL_MAX 12      /* 15 ms */
#endif

/* ATT MTU asked for */
#ifndef BLE_MIDI_HOST_PREFERRED_MTU
#define BLE_MIDI_HOST_PREFERRED_MTU 247
#endif

/* Reconnection: a scan of RECONNECT_SCAN_MS every RECONNECT_PERIOD_MS while
 * a remembered peripheral is missing */
#ifndef BLE_MIDI_HOST_RECONNECT_SCAN_MS
#define BLE_MIDI_HOST_RECONNECT_SCAN_MS 4000
#endif
#ifndef BLE_MIDI_HOST_RECONNECT_PERIOD_MS
#define BLE_MIDI_HOST_RECONNECT_PERIOD_MS 8000
#endif

/* Queues per slot (power of 2) */
#ifndef BLE_MIDI_HOST_RX_PACKETS
#define BLE_MIDI_HOST_RX_PACKETS 256
#endif
#ifndef BLE_MIDI_HOST_TX_PACKETS
#define BLE_MIDI_HOST_TX_PACKETS 256
#endif

#ifndef BLE_MIDI_HOST_TX_BATCH_US
#define BLE_MIDI_HOST_TX_BATCH_US 1000
#endif

#ifndef BLE_MIDI_HOST_TASK_CORE
#ifdef CONFIG_BT_NIMBLE_PINNED_TO_CORE
#define BLE_MIDI_HOST_TASK_CORE CONFIG_BT_NIMBLE_PINNED_TO_CORE
#else
#define BLE_MIDI_HOST_TASK_CORE 0
#endif
#endif
#ifndef BLE_MIDI_HOST_TASK_PRIORITY
#define BLE_MIDI_HOST_TASK_PRIORITY 5
#endif
#ifndef BLE_MIDI_HOST_TASK_STACK
#define BLE_MIDI_HOST_TASK_STACK 4096
#endif

#endif /* BLE_MIDI_HOST_CONFIG_DEFINED_H_ */
