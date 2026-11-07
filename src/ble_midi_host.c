/*
 * PicoRuby BLE-MIDI Host - VM selection wrapper
 */

#include "../include/ble_midi_host.h"

#if defined(PICORB_VM_MRUBY)
  #include "mruby/ble_midi_host.c"
#elif defined(PICORB_VM_MRUBYC)
  #include "mrubyc/ble_midi_host.c"
#endif
