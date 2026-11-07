# picoruby-ble_midi_host

BLE-MIDI host (central) transport for PicoRuby: connects to BLE-MIDI
peripherals (keyboards, WIDI / MD-BT01 adapters). Each peripheral is a slot
with a [picoruby-midi](../picoruby-midi) transport of its own, like the
USB-MIDI host's devices on a hub.

```ruby
require 'ble_midi_host'

BLE_MIDI_HOST.scan(timeout_ms: 5000)      # => [{name:, address:, type:, rssi:, known:, connected:}]
kb = BLE_MIDI_HOST.connect("WIDI")        # by name (part of it) or address:; waits for the link
MIDI::Device.new(kb)
BLE_MIDI_HOST.find("WIDI")                # a connected one by name
BLE_MIDI_HOST.devices                     # every connected one
BLE_MIDI_HOST.known / forget(address)     # remembered peripherals (NVS)
BLE_MIDI_HOST.rx_latency = 20             # ms (:auto by default, 0 = off)
```

- Peripherals connected once are remembered (NVS `ble_midi` / `known`) and
  connected again automatically while the host is enabled
  (`BLE_MIDI_HOST.enable = true`, `auto_connect =`).
- Pairing (Just Works, bonded) when the peripheral asks for it.
- Incoming messages keep the spacing of the sender's timestamps
  (`rx_latency`).
- Works alongside picoruby-ble's `BLE` class: its scans and connects go
  first; ours (and reconnecting) wait until it is done.
- ESP32 only, on NimBLE through picoruby-ble's shared NimBLE owner. The host
  project compiles `ports/esp32` and defines `BLE_MIDI_HOST_ENABLED=1`.
- Build-time knobs (slots, intervals, reconnect timing):
  `include/ble_midi_host_config.h`.

Design and details: Midori's docs/BLE_MIDI.md.
