# BLE-MIDI Host (central) transport layer
#
# Connects to BLE-MIDI peripherals (keyboards, adapters). Each connected
# peripheral is a slot with a transport of its own:
#
#   BLE_MIDI_HOST.scan                     # => [{name:, address:, rssi:, ...}]
#   kb = BLE_MIDI_HOST.connect("WIDI")     # by name (or address:)
#   dev = MIDI::Device.new(kb)
#
# Links are C state that lives across scripts; peripherals connected once
# are remembered and connected again automatically while the host is
# enabled (BLE_MIDI_HOST.enable = true).
class BLE_MIDI_HOST

  def self.available?
    _available
  end

  # Hold the BLE host and reconnect the remembered peripherals (true), or
  # drop every link (false; waits until it is done, up to 3 s, so that a
  # script can use the BLE class right after). For this script only: the
  # saved setting comes back when it ends.
  def self.enable=(on)
    _enable(on ? true : false)
    return if on
    started = Machine.board_millis
    while _ready && Machine.board_millis - started < 3000
      sleep_ms 20
    end
  end

  def self.enabled?
    _enabled
  end

  # BLE host up for the BLE-MIDI host
  def self.ready?
    _ready
  end

  # Reconnect remembered peripherals automatically (default true)
  def self.auto_connect=(on)
    _set_auto(on ? true : false)
  end

  def self.auto_connect?
    _auto
  end

  # The object for a slot (slot 0 is the first peripheral connected)
  def self.instance(slot = 0)
    _init
    $__ble_midi_host_instances__ ||= {}
    $__ble_midi_host_instances__[slot] ||= new(slot)
  end

  def self.max_devices
    _max_devices
  end

  # Scan for BLE-MIDI peripherals. Blocks for timeout_ms and returns what
  # was found: [{name:, address:, type:, rssi:, known:, connected:}, ...]
  def self.scan(timeout_ms: 5000)
    _init
    return [] unless _scan_start(timeout_ms)
    started = Machine.board_millis
    while _scanning
      break if Machine.board_millis - started > timeout_ms + 1000
      sleep_ms 50
    end
    _scan_results
  end

  def self.scan_start(timeout_ms = 5000)
    _init
    _scan_start(timeout_ms)
  end

  def self.scanning?
    _scanning
  end

  def self.scan_results
    _scan_results
  end

  def self.scan_stop
    _scan_stop
  end

  # Remembered peripherals: [{name:, address:, type:, known: true, connected:}, ...]
  def self.known
    _known
  end

  # Forget a remembered peripheral (and its pairing); disconnects it
  def self.forget(address)
    _forget(address.to_s)
  end

  # Connect to a peripheral by name (part of it, case-insensitive) or
  # address. A name is looked up in the last scan and the remembered list
  # (scanning first when it is in neither). Waits up to timeout_ms for the
  # link to be ready; returns the slot's object, or nil.
  def self.connect(name = nil, address: nil, type: nil, timeout_ms: 10000)
    _init
    peer = nil
    if address
      address = address.to_s
      peer = (_scan_results + _known).find { |p| p[:address] == address }
      type ||= peer ? peer[:type] : 0
    else
      peer = _match(_scan_results, name) || _match(_known, name)
      peer = _match(scan, name) unless peer
      return nil unless peer
      address = peer[:address]
      type ||= peer[:type]
    end
    slot = _connect(address, type)
    return nil if slot < 0
    started = Machine.board_millis
    while Machine.board_millis - started < timeout_ms
      st = _status(slot)
      return instance(slot) if st == :connected
      break if st == :free
      sleep_ms 50
    end
    nil
  end

  def self._match(peers, name)
    return peers.first if name.nil?
    key = name.to_s.downcase
    peers.find { |p| p[:name].to_s.downcase.include?(key) }
  end

  # A connected peripheral by name (part of it), or nil
  def self.find(name)
    _init
    slot = _find(name.to_s)
    slot < 0 ? nil : instance(slot)
  end

  # Connected peripherals
  def self.devices
    _init
    list = []
    _max_devices.times { |i| list << instance(i) if _connected(i) }
    list
  end

  attr_reader :slot

  def initialize(slot = 0)
    @slot = slot
  end

  # Bit in picoruby-midi's transport registry (0 until the slot is first used)
  def transport_id
    BLE_MIDI_HOST._bit(@slot)
  end

  def connected?
    BLE_MIDI_HOST._connected(@slot)
  end

  # :free / :connecting / :discovering / :subscribing / :connected
  def status
    BLE_MIDI_HOST._status(@slot)
  end

  # name, address, status, MTU, connection interval, counters (nil if the
  # slot was never used)
  def info
    BLE_MIDI_HOST._dev_info(@slot)
  end

  def name
    i = info
    i ? i[:name] : nil
  end

  def address
    i = info
    i ? i[:address] : nil
  end

  def disconnect
    BLE_MIDI_HOST._disconnect(@slot)
  end

  def send_packet(cable, cin, midi1, midi2, midi3)
    BLE_MIDI_HOST._send_packet(@slot, cable, cin, midi1, midi2, midi3)
  end

  def bytes_available
    BLE_MIDI_HOST._bytes_available(@slot)
  end

  def read_available
    BLE_MIDI_HOST._read_available(@slot)
  end
end
