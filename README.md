XIAO RP2040 CRSF half-duplex <-> full-duplex bridge
===================================================
Replaces an external RF module in a RadioMaster TX16S (EdgeTX) JR module
bay with a direct wire to a Betaflight flight controller.

* TX16S module bay                XIAO RP2040             Betaflight FC
* (CRSF, half-duplex,      <-->   PIO UART (1-wire)       UART0   <-->  CRSF serial RX
   inverted, 400000 baud)                                 (2-wire, non-inverted, 420000 baud)

* Radio -> FC : every valid CRSF frame is forwarded unchanged, except that the first (address) byte is rewritten to 0xC8. Betaflight only accepts RC_CHANNELS frames whose address byte is 0xC8 (rx/crsf.c), while EdgeTX sends them addressed to the module
              (0xEE). The address byte is not covered by the CRC.
* FC -> Radio : every valid CRSF telemetry frame from the FC is queued and sent
              to the radio, one frame per reply slot (right after each radio
              frame). The address byte is rewritten to 0xEA (the radio
              address). EdgeTX accepts 0xEA or 0xC8 (pulses/crossfire.cpp).
* Injected    : a LINK_STATISTICS frame (0x14) with 100 % uplink and downlink LQ
              is sent every 100 ms to BOTH the radio and the FC. The radio
              side gets priority over queued FC telemetry. If the queue
              overflows, the oldest frame is dropped.
* Failsafe    : if no valid radio frame arrives for 250 ms, the bridge stops
              forwarding and stops injecting link stats, so Betaflight's own
              failsafe takes over. Normal operation resumes automatically.
* Device ping : forwarded to the FC like any other frame, AND answered by the
              RP2040 with a DEVICE_INFO frame from the module address (0xEE).
              Once EdgeTX sees link stats it replaces the channel frames
              with pings until a module answers, so without this answer
              the sticks stop as soon as the radio reply is enabled.

Half-duplex wire handling ("tri-state")
---------------------------------------
The XIAO only brings out one complete hardware UART (UART0), and an RP2040 hardware UART can't put TX and RX on the same pin, so the radio side runs on a PIO UART (SerialPIO): one PIO state machine receives, another transmits, both on the same pad. Per-pad overrides in the IO bank do the rest:
* listening    : output enable forced OFF (hi-Z), pull-down on (the inverted line idles low), input inverted for the RX state machine.
* transmitting : input forced HIGH (UART idle) so we don't receive our own echo, output enable forced ON, output inverted.

Once the last stop bit has left the shifter, the pin goes back to hi-Z.

Wiring (Seeed Studio XIAO RP2040)
---------------------------------
*  D0 (GP26) <-> TX16S module bay CRSF signal pin (must be 3.3 V logic: the RP2040 is NOT 5 V tolerant)
*  D6 (GP0)   -> FC UART RX     (pad marked TX)
*  D7 (GP1)  <-  FC UART TX     (pad marked RX)
*  GND       <-> TX16S module bay GND and FC GND (common ground is required)
*  GP17/GP16     on-board user RGB LED, red/green (active low)
*  USB           status counters, once a second

Status LED
----------
  solid green   : radio frames arriving, link up
  red flash     : (while green) corrupt data or a UART error on either port: CRC failure, discarded bytes, break, RX buffer overflow (the RP2040 serial drivers drop framing-error bytes silently; they show up here as junk/CRC errors)
  blinking red  : no radio frames (failsafe), nothing forwarded

Betaflight notes
----------------
  - Ports tab: Serial RX on the UART wired to D6/D7. Receiver: Serial, CRSF.
  - serialrx_inverted = OFF, serialrx_halfduplex = OFF (the default for CRSF).
  - If rssi_source / link-quality source is left unset, Betaflight picks the CRSF protocol automatically and shows the injected stats (rx/crsf.c, crsfRxInit).

Build
-----
  Earle Philhower's Arduino-Pico core (tested against 6.3.0), board "Seeed XIAO RP2040". Not the Seeed/mbed "Seeed RP2040 Boards" package.
  No extra libraries.
