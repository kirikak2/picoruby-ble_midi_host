MRuby::Gem::Specification.new('picoruby-ble_midi_host') do |spec|
  spec.license = 'MIT'
  spec.author  = 'Toshio Maki'
  spec.summary = 'BLE-MIDI Host (central) transport layer'
  spec.require_name = 'ble_midi_host'
  # midi_transport.h / ble_midi_codec.h; the ESP32 port is compiled by the
  # host project (it needs NimBLE and picoruby-ble's ESP32 port)
  spec.add_dependency 'picoruby-midi'
  spec.add_dependency 'picoruby-machine'   # sleep_ms
end
