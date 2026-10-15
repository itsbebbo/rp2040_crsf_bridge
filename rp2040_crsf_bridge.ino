/*
 * XIAO RP2040 CRSF half-duplex <-> full-duplex bridge
 * ===================================================
 *
 * Replaces an external RF module in a RadioMaster TX16S (EdgeTX) JR module
 * bay with a direct wire to a Betaflight flight controller.
 *
 *   TX16S module bay                XIAO RP2040             Betaflight FC
 *   (CRSF, half-duplex,      <-->   PIO UART (1-wire)       UART0   <-->  CRSF serial RX
 *    inverted, 400000 baud)                                 (2-wire, non-inverted, 420000 baud)
 *
 * Radio -> FC : every valid CRSF frame is forwarded unchanged, except that the
 *               first (address) byte is rewritten to 0xC8. Betaflight only
 *               accepts RC_CHANNELS frames whose address byte is 0xC8
 *               (rx/crsf.c), while EdgeTX sends them addressed to the module
 *               (0xEE). The address byte is not covered by the CRC.
 * FC -> Radio : every valid CRSF telemetry frame from the FC is queued and sent
 *               to the radio, one frame per reply slot (right after each radio
 *               frame). The address byte is rewritten to 0xEA (the radio
 *               address). EdgeTX accepts 0xEA or 0xC8 (pulses/crossfire.cpp).
 * Injected    : a LINK_STATISTICS frame (0x14) with 100 % uplink and downlink LQ
 *               is sent every 100 ms to BOTH the radio and the FC. The radio
 *               side gets priority over queued FC telemetry. If the queue
 *               overflows, the oldest frame is dropped.
 * Failsafe    : if no valid radio frame arrives for 250 ms, the bridge stops
 *               forwarding and stops injecting link stats, so Betaflight's own
 *               failsafe takes over. Normal operation resumes automatically.
 * Device ping : forwarded to the FC like any other frame, AND answered by the
 *               RP2040 with a DEVICE_INFO frame from the module address (0xEE).
 *               Once EdgeTX sees link stats it replaces the channel frames
 *               with pings until a module answers, so without this answer
 *               the sticks stop as soon as the radio reply is enabled.
 *
 * Half-duplex wire handling ("tri-state")
 * ---------------------------------------
 * The XIAO only brings out one complete hardware UART (UART0), and an RP2040
 * hardware UART can't put TX and RX on the same pin, so the radio side runs on
 * a PIO UART (SerialPIO): one PIO state machine receives, another transmits,
 * both on the same pad. Per-pad overrides in the IO bank do the rest:
 *   listening    : output enable forced OFF (hi-Z), pull-down on (the inverted
 *                  line idles low), input inverted for the RX state machine.
 *   transmitting : input forced HIGH (UART idle) so we don't receive our own
 *                  echo, output enable forced ON, output inverted.
 * Once the last stop bit has left the shifter, the pin goes back to hi-Z.
 *
 * Wiring (Seeed Studio XIAO RP2040)
 * ---------------------------------
 *   D0 (GP26) <-> TX16S module bay CRSF signal pin (must be 3.3 V logic: the
 *                 RP2040 is NOT 5 V tolerant)
 *   D6 (GP0)   -> FC UART RX     (pad marked TX)
 *   D7 (GP1)  <-  FC UART TX     (pad marked RX)
 *   GND       <-> TX16S module bay GND and FC GND (common ground is required)
 *   GP17/GP16     on-board user RGB LED, red/green (active low)
 *   USB           status counters, once a second
 *
 * Status LED
 * ----------
 *   solid green   : radio frames arriving, link up
 *   red flash     : (while green) corrupt data or a UART error on either port:
 *                   CRC failure, discarded bytes, break, RX buffer overflow
 *                   (the RP2040 serial drivers drop framing-error bytes
 *                   silently; they show up here as junk/CRC errors)
 *   blinking red  : no radio frames (failsafe), nothing forwarded
 *
 * Betaflight notes
 * ----------------
 *   - Ports tab: Serial RX on the UART wired to D6/D7. Receiver: Serial, CRSF.
 *   - serialrx_inverted = OFF, serialrx_halfduplex = OFF (the default for CRSF).
 *   - If rssi_source / link-quality source is left unset, Betaflight picks the
 *     CRSF protocol automatically and shows the injected stats
 *     (rx/crsf.c, crsfRxInit).
 *
 * Build
 * -----
 *   Earle Philhower's Arduino-Pico core (tested against 6.3.0), board
 *   "Seeed XIAO RP2040". Not the Seeed/mbed "Seeed RP2040 Boards" package.
 *   No extra libraries.
 */

#include <Arduino.h>
#include <SerialPIO.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>

// ---------------------------------------------------------------------------
// User configuration
// ---------------------------------------------------------------------------
#define PIN_RADIO_HALF_DUPLEX   26      // D0: single-wire CRSF to the TX16S module bay (any GPIO)
#define PIN_FC_TX               0       // D6: RP2040 TX -> FC RX (UART0 TX: GP0 or GP28)
#define PIN_FC_RX               1       // D7: RP2040 RX <- FC TX (UART0 RX: GP1 or GP29)
#define PIN_LED_RED             17      // on-board user RGB LED, active low
#define PIN_LED_GREEN           16

#define RADIO_BAUD              400000  // EdgeTX: Model Setup > External RF > Baudrate
#define FC_BAUD                 420000  // Betaflight CRSF default
#define RADIO_INVERTED          true    // module-bay signal idles low
#define ENABLE_RADIO_REPLY      1       // 0 = listen only; 1 = send telemetry to the radio

#define LINK_STATS_INTERVAL_MS  100     // injection period, to radio and to FC
#define RADIO_TIMEOUT_MS        250     // no radio frame for this long -> failsafe
#define FC_TELEM_QUEUE_LEN      32      // FC->radio frames buffered between reply slots
#define REPLY_TURNAROUND_US     0       // optional extra wait before driving the wire

#define ENABLE_USB_STATS        0       // 1 = print counters once a second
#define LED_BRIGHTNESS          20      // 0-255
#define LED_ERROR_FLASH_MS      100     // red flash length on corrupt data / UART errors

// Values for the injected LINK_STATISTICS frame. RSSI fields are "dBm * -1".
#define LS_UPLINK_RSSI_1        30      // -30 dBm
#define LS_UPLINK_RSSI_2        30      // -30 dBm
#define LS_UPLINK_LQ            100     // %
#define LS_UPLINK_SNR           10      // dB
#define LS_ACTIVE_ANTENNA       0
#define LS_RF_MODE              0
#define LS_UPLINK_TX_POWER      0       // index (EdgeTX: 0 = 0 mW)
#define LS_DOWNLINK_RSSI        30      // -30 dBm
#define LS_DOWNLINK_LQ          100     // %
#define LS_DOWNLINK_SNR         10      // dB

// ---------------------------------------------------------------------------
// CRSF protocol constants
// ---------------------------------------------------------------------------
#define CRSF_ADDR_BROADCAST         0x00
#define CRSF_ADDR_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDR_RADIO_TRANSMITTER 0xEA
#define CRSF_ADDR_CRSF_RECEIVER     0xEC
#define CRSF_ADDR_CRSF_TRANSMITTER  0xEE

#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_LINK_STATS_PAYLOAD_LEN    10
#define CRSF_FRAMETYPE_RC_CHANNELS     0x16
#define CRSF_FRAMETYPE_DEVICE_PING     0x28
#define CRSF_FRAMETYPE_DEVICE_INFO     0x29

#define DEVICE_INFO_NAME        "RP2040 CRSF Bridge"

#define CRSF_MAX_FRAME_LEN      64      // address + length + (type + payload + crc)
#define CRSF_MIN_LEN_FIELD      2       // type + crc
#define CRSF_MAX_LEN_FIELD      (CRSF_MAX_FRAME_LEN - 2)

// SerialPIO with TX and RX on the same pin. The subclass only exposes which
// PIO block the TX state machine landed in, so the pad can be handed to it.
class HalfDuplexSerialPIO : public SerialPIO {
public:
  using SerialPIO::SerialPIO;
  gpio_function_t txFunction() const {
    return (gpio_function_t)(GPIO_FUNC_PIO0 + pio_get_index(_txPIO));
  }
};

// PIO UART <-> radio (pad direction switched by hand)
// UART0    <-> flight controller (normal two-pin operation)
static HalfDuplexSerialPIO RadioSerial(PIN_RADIO_HALF_DUPLEX, PIN_RADIO_HALF_DUPLEX, 512);
SerialUART &FcSerial = Serial1;

// ---------------------------------------------------------------------------
// CRC8, polynomial 0xD5 (DVB-S2), over type + payload
// ---------------------------------------------------------------------------
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

static uint8_t crsfCrc(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc = crc8_dvb_s2(crc, *data++);
  }
  return crc;
}

// ---------------------------------------------------------------------------
// Byte-wise CRSF frame parser
// ---------------------------------------------------------------------------
struct CrsfParser {
  uint8_t buf[CRSF_MAX_FRAME_LEN];
  uint8_t pos = 0;
  uint8_t frameLen = 0;
  uint32_t crcErrors = 0;
  uint32_t junkBytes = 0;   // bytes discarded while hunting for a frame start

  static bool isAddress(uint8_t b) {
    return b == CRSF_ADDR_FLIGHT_CONTROLLER || b == CRSF_ADDR_RADIO_TRANSMITTER ||
           b == CRSF_ADDR_CRSF_RECEIVER || b == CRSF_ADDR_CRSF_TRANSMITTER;
  }

  // Returns true when buf[0..frameLen-1] holds a complete frame with a valid CRC.
  bool push(uint8_t b) {
    if (pos == 0) {
      if (isAddress(b)) {
        buf[pos++] = b;
      } else {
        junkBytes++;
      }
      return false;
    }
    if (pos == 1) {
      if (b < CRSF_MIN_LEN_FIELD || b > CRSF_MAX_LEN_FIELD) {
        // Bad length: resync, treating this byte as a possible new start.
        pos = 0;
        junkBytes++;
        if (isAddress(b)) {
          buf[pos++] = b;
        }
        return false;
      }
      buf[pos++] = b;
      frameLen = b + 2;
      return false;
    }
    buf[pos++] = b;
    if (pos < frameLen) {
      return false;
    }
    pos = 0;
    if (crsfCrc(&buf[2], frameLen - 3) != buf[frameLen - 1]) {
      crcErrors++;
      return false;
    }
    return true;
  }
};

static CrsfParser radioParser;
static CrsfParser fcParser;

// ---------------------------------------------------------------------------
// FC -> radio telemetry queue (ring buffer; oldest dropped on overflow)
// ---------------------------------------------------------------------------
struct QueuedFrame {
  uint8_t len;
  uint8_t data[CRSF_MAX_FRAME_LEN];
};

static QueuedFrame telemQueue[FC_TELEM_QUEUE_LEN];
static uint8_t telemHead = 0;   // next to send
static uint8_t telemCount = 0;

static void telemClear() {
  telemHead = 0;
  telemCount = 0;
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------
static uint32_t statRadioFrames = 0;     // valid frames from radio
static uint32_t statFcFrames = 0;        // valid frames from FC
static uint32_t statToRadioTelem = 0;    // FC telemetry frames sent to radio
static uint32_t statToRadioLinkStats = 0;
static uint32_t statToFcLinkStats = 0;
static uint32_t statQueueDrops = 0;
static uint32_t statSkippedSlots = 0;    // reply skipped: radio already sending again
static uint32_t statPings = 0;           // device pings received from radio
static uint32_t statDeviceInfo = 0;      // DEVICE_INFO replies sent to radio
static uint32_t statRcFrames = 0;        // RC_CHANNELS frames since last print
static uint32_t statRcMaxGapUs = 0;      // longest RC_CHANNELS gap since last print
static uint32_t lastRcFrameUs = 0;
static uint32_t statRcChanged = 0;       // RC_CHANNELS frames whose channel data changed
static uint8_t lastRcPayload[22];        // 16 channels x 11 bits
static uint16_t lastCh[4];               // first four channels, raw CRSF units (172-1811)
static uint32_t statLoopMaxUs = 0;       // longest loop() pass since last print
static uint32_t statRadioRxPeak = 0;     // most bytes waiting in the radio RX buffer
static uint32_t statFcRxPeak = 0;        // most bytes waiting in the FC RX buffer

// Serial driver error events. The Arduino-Pico drivers have no error callback,
// only sticky flags (cleared on read), so they are polled once per loop.
enum { UERR_BUF_FULL, UERR_BREAK, UERR_COUNT };
static uint32_t radioUartErr[UERR_COUNT];
static uint32_t fcUartErr[UERR_COUNT];

static void pollUartErrors() {
  if (RadioSerial.overflow()) {
    radioUartErr[UERR_BUF_FULL]++;
  }
  if (FcSerial.overflow()) {
    fcUartErr[UERR_BUF_FULL]++;
  }
  if (FcSerial.getBreakReceived()) {
    fcUartErr[UERR_BREAK]++;
  }
}

static void telemPush(const uint8_t *frame, uint8_t len) {
  if (telemCount == FC_TELEM_QUEUE_LEN) {
    telemHead = (telemHead + 1) % FC_TELEM_QUEUE_LEN;  // drop oldest
    telemCount--;
    statQueueDrops++;
  }
  QueuedFrame &slot = telemQueue[(telemHead + telemCount) % FC_TELEM_QUEUE_LEN];
  memcpy(slot.data, frame, len);
  slot.len = len;
  telemCount++;
}

// ---------------------------------------------------------------------------
// Link state
// ---------------------------------------------------------------------------
static uint32_t lastRadioFrameMs = 0;
static bool radioSeen = false;
static bool linkUp = false;
static uint32_t lastLinkStatsRadioMs = 0;
static uint32_t lastLinkStatsFcMs = 0;
static bool deviceInfoPending = false;   // radio pinged us; answer in next slot

static uint8_t buildLinkStats(uint8_t *out, uint8_t address) {
  out[0] = address;
  out[1] = CRSF_LINK_STATS_PAYLOAD_LEN + 2;       // type + payload + crc
  out[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
  out[3] = LS_UPLINK_RSSI_1;
  out[4] = LS_UPLINK_RSSI_2;
  out[5] = LS_UPLINK_LQ;
  out[6] = (uint8_t)(int8_t)LS_UPLINK_SNR;
  out[7] = LS_ACTIVE_ANTENNA;
  out[8] = LS_RF_MODE;
  out[9] = LS_UPLINK_TX_POWER;
  out[10] = LS_DOWNLINK_RSSI;
  out[11] = LS_DOWNLINK_LQ;
  out[12] = (uint8_t)(int8_t)LS_DOWNLINK_SNR;
  out[13] = crsfCrc(&out[2], CRSF_LINK_STATS_PAYLOAD_LEN + 1);
  return CRSF_LINK_STATS_PAYLOAD_LEN + 4;
}

// Extended frame: dest, origin, name\0, serial[4], hw[4], sw[4], field count,
// parameter version. Zero fields, so EdgeTX's config script has nothing to
// load. The serial is deliberately not "ELRS" (EdgeTX special-cases that).
static uint8_t buildDeviceInfo(uint8_t *out) {
  static const uint8_t tail[] = {
    'R', 'P', '2', 'B',        // serial number
    0, 0, 0, 1,                // hardware version
    0, 1, 0, 0,                // software version (major 1, minor 0)
    0,                         // parameter field count
    0,                         // parameter version
  };
  uint8_t pos = 2;
  out[0] = CRSF_ADDR_RADIO_TRANSMITTER;
  out[pos++] = CRSF_FRAMETYPE_DEVICE_INFO;
  out[pos++] = CRSF_ADDR_RADIO_TRANSMITTER;   // destination
  out[pos++] = CRSF_ADDR_CRSF_TRANSMITTER;    // origin: we act as the module
  const char *name = DEVICE_INFO_NAME;
  do {
    out[pos++] = (uint8_t)*name;               // includes the terminating 0
  } while (*name++);
  memcpy(&out[pos], tail, sizeof(tail));
  pos += sizeof(tail);
  out[1] = pos - 1;                            // type .. crc
  out[pos] = crsfCrc(&out[2], pos - 2);
  return pos + 1;
}

// ---------------------------------------------------------------------------
// Half-duplex pin direction control
// ---------------------------------------------------------------------------
// Run once after RadioSerial.begin(). SerialPIO's RX setup takes the pad last
// and turns its output off; hand the pad's output to the TX state machine
// instead (PIO inputs read every pad regardless of the function select) and
// apply the inversion here. gpio_set_function() clears all overrides, so the
// inversions are set after it. From here on only the override fields change.
static void radioPinInit() {
  gpio_set_function(PIN_RADIO_HALF_DUPLEX, RadioSerial.txFunction());
  gpio_set_outover(PIN_RADIO_HALF_DUPLEX,
                   RADIO_INVERTED ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
}

static void radioPinListen() {
  // Output driver forced off -> hi-Z input; RX state machine sees the wire.
  gpio_set_oeover(PIN_RADIO_HALF_DUPLEX, GPIO_OVERRIDE_LOW);
  gpio_set_pulls(PIN_RADIO_HALF_DUPLEX, !RADIO_INVERTED, RADIO_INVERTED);
  gpio_set_inover(PIN_RADIO_HALF_DUPLEX,
                  RADIO_INVERTED ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
}

static void radioPinTransmit() {
  // Feed the RX state machine a constant idle (mark) level so our own bytes
  // are not echoed.
  gpio_set_inover(PIN_RADIO_HALF_DUPLEX, GPIO_OVERRIDE_HIGH);
  gpio_disable_pulls(PIN_RADIO_HALF_DUPLEX);
  // Enable the push-pull driver; the idle TX state machine holds the mark level.
  gpio_set_oeover(PIN_RADIO_HALF_DUPLEX, GPIO_OVERRIDE_HIGH);
}

static void radioSend(const uint8_t *frame, uint8_t len) {
#if REPLY_TURNAROUND_US > 0
  delayMicroseconds(REPLY_TURNAROUND_US);
#endif
  radioPinTransmit();
  RadioSerial.write(frame, len);
  RadioSerial.flush();   // waits until the last stop bit has been shifted out
                         // (TX FIFO empty, then one more character time)
  radioPinListen();
}

// One reply per radio frame: a pending ping answer first, then link stats when
// due, otherwise queued FC telemetry.
static void radioReplySlot() {
  uint32_t now = millis();
  if (deviceInfoPending) {
    uint8_t frame[CRSF_MAX_FRAME_LEN];
    uint8_t len = buildDeviceInfo(frame);
    radioSend(frame, len);
    deviceInfoPending = false;
    statDeviceInfo++;
    return;
  }
  if (now - lastLinkStatsRadioMs >= LINK_STATS_INTERVAL_MS) {
    uint8_t frame[CRSF_LINK_STATS_PAYLOAD_LEN + 4];
    uint8_t len = buildLinkStats(frame, CRSF_ADDR_RADIO_TRANSMITTER);
    radioSend(frame, len);
    lastLinkStatsRadioMs = now;
    statToRadioLinkStats++;
    return;
  }
  if (telemCount > 0) {
    QueuedFrame &qf = telemQueue[telemHead];
    radioSend(qf.data, qf.len);
    telemHead = (telemHead + 1) % FC_TELEM_QUEUE_LEN;
    telemCount--;
    statToRadioTelem++;
  }
}

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------
static void setLed(uint8_t r, uint8_t g) {
  analogWrite(PIN_LED_RED, 255 - r);     // active low
  analogWrite(PIN_LED_GREEN, 255 - g);
}

// Every kind of bad input on either port: CRC failures, bytes the parser had
// to throw away, and UART driver errors (overflow, buffer full, framing,
// break, parity).
static uint32_t totalErrors() {
  uint32_t total = radioParser.crcErrors + radioParser.junkBytes +
                   fcParser.crcErrors + fcParser.junkBytes;
  for (int i = 0; i < UERR_COUNT; i++) {
    total += radioUartErr[i] + fcUartErr[i];
  }
  return total;
}

static void updateLed() {
  static int8_t lastState = -1;   // 0 = off, 1 = green, 2 = red
  static uint32_t lastErrors = 0;
  static uint32_t errorFlashStartMs = 0;
  static bool errorFlash = false;

  uint32_t errors = totalErrors();
  if (errors != lastErrors) {
    lastErrors = errors;
    errorFlash = true;
    errorFlashStartMs = millis();   // restarted by each new error
  }
  if (errorFlash && millis() - errorFlashStartMs >= LED_ERROR_FLASH_MS) {
    errorFlash = false;
  }

  int8_t state;
  if (linkUp) {
    state = errorFlash ? 2 : 1;
  } else {
    state = ((millis() / 250) & 1) ? 2 : 0;   // 2 Hz red blink
  }
  if (state == lastState) {
    return;
  }
  lastState = state;
  switch (state) {
    case 1:  setLed(0, LED_BRIGHTNESS); break;
    case 2:  setLed(LED_BRIGHTNESS, 0); break;
    default: setLed(0, 0); break;
  }
}

// ---------------------------------------------------------------------------
// USB statistics
// ---------------------------------------------------------------------------
static void printLine(const char *line, int n) {
  if (n > 0 && Serial.availableForWrite() >= n) {
    Serial.write((const uint8_t *)line, n);
  }
}

static void printStats() {
#if ENABLE_USB_STATS
  static uint32_t lastPrintMs = 0;
  uint32_t now = millis();
  if (now - lastPrintMs < 1000) {
    return;
  }
  lastPrintMs = now;
  // Three short lines, each sent only if it fits in the USB CDC TX buffer
  // whole, so a missing host never stalls the bridge.
  char line[200];
  int n = snprintf(line, sizeof(line),
                   "RC    hz=%lu changed=%lu ch1-4=%u,%u,%u,%u maxGapMs=%lu.%lu link=%s "
                   "loopMaxUs=%lu skipped=%lu pings=%lu devInfo=%lu\r\n",
                   (unsigned long)statRcFrames, (unsigned long)statRcChanged,
                   lastCh[0], lastCh[1], lastCh[2], lastCh[3],
                   (unsigned long)(statRcMaxGapUs / 1000),
                   (unsigned long)((statRcMaxGapUs % 1000) / 100),
                   linkUp ? "UP" : "DOWN", (unsigned long)statLoopMaxUs,
                   (unsigned long)statSkippedSlots,
                   (unsigned long)statPings, (unsigned long)statDeviceInfo);
  printLine(line, n);
  n = snprintf(line, sizeof(line),
               "RADIO frames=%lu crcErr=%lu junk=%lu rxPeak=%lu bufFull=%lu "
               "toTelem=%lu toLS=%lu\r\n",
               (unsigned long)statRadioFrames, (unsigned long)radioParser.crcErrors,
               (unsigned long)radioParser.junkBytes, (unsigned long)statRadioRxPeak,
               (unsigned long)radioUartErr[UERR_BUF_FULL],
               (unsigned long)statToRadioTelem, (unsigned long)statToRadioLinkStats);
  printLine(line, n);
  n = snprintf(line, sizeof(line),
               "FC    frames=%lu crcErr=%lu junk=%lu rxPeak=%lu "
               "bufFull=%lu break=%lu toLS=%lu qDrops=%lu q=%u\r\n",
               (unsigned long)statFcFrames, (unsigned long)fcParser.crcErrors,
               (unsigned long)fcParser.junkBytes, (unsigned long)statFcRxPeak,
               (unsigned long)fcUartErr[UERR_BUF_FULL], (unsigned long)fcUartErr[UERR_BREAK],
               (unsigned long)statToFcLinkStats, (unsigned long)statQueueDrops,
               (unsigned)telemCount);
  printLine(line, n);
  statRcFrames = 0;
  statRcChanged = 0;
  statRcMaxGapUs = 0;
  statLoopMaxUs = 0;
  statRadioRxPeak = 0;
  statFcRxPeak = 0;
#endif
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
#if ENABLE_USB_STATS
  Serial.begin(115200);   // USB CDC; the baud rate is ignored
#endif

  // Radio: PIO UART, RX and TX state machines on the same pin. The PIO RX
  // state machine pushes every byte as soon as its stop bit is sampled.
  // Inversion is done with the pad overrides, not SerialPIO's setInverted().
  // (begin() briefly drives the pin before radioPinInit() makes it hi-Z.)
  RadioSerial.begin(RADIO_BAUD, SERIAL_8N1);
  radioPinInit();
  radioPinListen();

  // Flight controller: UART0, ordinary full duplex, non-inverted. There is
  // no software TX buffer, only the 32-byte hardware FIFO, so a write only
  // blocks if more than ~32 bytes are already queued (< 1 ms at 420000).
  FcSerial.setFIFOSize(1024);      // software RX buffer
  FcSerial.setTX(PIN_FC_TX);
  FcSerial.setRX(PIN_FC_RX);
  FcSerial.begin(FC_BAUD, SERIAL_8N1);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  setLed(0, 0);
}

void loop() {
  uint32_t now = millis();

  static uint32_t lastLoopUs = 0;
  uint32_t loopUs = micros();
  if (lastLoopUs != 0 && loopUs - lastLoopUs > statLoopMaxUs) {
    statLoopMaxUs = loopUs - lastLoopUs;
  }
  lastLoopUs = loopUs;

  // ---- Radio -> FC ----------------------------------------------------------
  bool replyDue = false;
  uint32_t waiting = RadioSerial.available();
  if (waiting > statRadioRxPeak) {
    statRadioRxPeak = waiting;
  }
  while (RadioSerial.available()) {
    uint8_t b = (uint8_t)RadioSerial.read();
    if (radioParser.push(b)) {
      statRadioFrames++;
      lastRadioFrameMs = millis();
      radioSeen = true;
      if (!linkUp) {
        linkUp = true;
        lastLinkStatsRadioMs = lastRadioFrameMs - LINK_STATS_INTERVAL_MS;  // send stats at once
        lastLinkStatsFcMs = lastRadioFrameMs - LINK_STATS_INTERVAL_MS;
      }
      if (radioParser.buf[2] == CRSF_FRAMETYPE_DEVICE_PING &&
          (radioParser.buf[3] == CRSF_ADDR_BROADCAST ||
           radioParser.buf[3] == CRSF_ADDR_CRSF_TRANSMITTER)) {
        deviceInfoPending = true;
        statPings++;
      }
      if (radioParser.buf[2] == CRSF_FRAMETYPE_RC_CHANNELS) {
        uint32_t nowUs = micros();
        uint32_t gap = nowUs - lastRcFrameUs;
        if (lastRcFrameUs != 0 && gap > statRcMaxGapUs) {
          statRcMaxGapUs = gap;
        }
        lastRcFrameUs = nowUs;
        statRcFrames++;
        const uint8_t *p = &radioParser.buf[3];
        if (radioParser.frameLen >= 3 + sizeof(lastRcPayload) + 1 &&
            memcmp(p, lastRcPayload, sizeof(lastRcPayload)) != 0) {
          memcpy(lastRcPayload, p, sizeof(lastRcPayload));
          statRcChanged++;
          lastCh[0] = (p[0] | p[1] << 8) & 0x7FF;
          lastCh[1] = (p[1] >> 3 | p[2] << 5) & 0x7FF;
          lastCh[2] = (p[2] >> 6 | p[3] << 2 | p[4] << 10) & 0x7FF;
          lastCh[3] = (p[4] >> 1 | p[5] << 7) & 0x7FF;
        }
      }
      radioParser.buf[0] = CRSF_ADDR_FLIGHT_CONTROLLER;
      FcSerial.write(radioParser.buf, radioParser.frameLen);
      replyDue = true;
    }
  }

  // Only use the slot if the radio isn't already sending its next frame (that
  // would mean we fell behind and a reply could collide).
  if (ENABLE_RADIO_REPLY && replyDue) {
    if (RadioSerial.available() == 0 && radioParser.pos == 0) {
      radioReplySlot();
    } else {
      statSkippedSlots++;
    }
  }

  // ---- Failsafe ---------------------------------------------------------------
  now = millis();
  if (linkUp && (!radioSeen || now - lastRadioFrameMs > RADIO_TIMEOUT_MS)) {
    linkUp = false;
    telemClear();
  }

  // ---- FC -> radio queue ------------------------------------------------------
  waiting = FcSerial.available();
  if (waiting > statFcRxPeak) {
    statFcRxPeak = waiting;
  }
  while (FcSerial.available()) {
    uint8_t b = (uint8_t)FcSerial.read();
    if (fcParser.push(b)) {
      statFcFrames++;
      if (linkUp) {
        fcParser.buf[0] = CRSF_ADDR_RADIO_TRANSMITTER;
        telemPush(fcParser.buf, fcParser.frameLen);
      }
    }
  }

  // ---- Injected link statistics to FC --------------------------------------------
  if (linkUp && now - lastLinkStatsFcMs >= LINK_STATS_INTERVAL_MS) {
    uint8_t frame[CRSF_LINK_STATS_PAYLOAD_LEN + 4];
    uint8_t len = buildLinkStats(frame, CRSF_ADDR_FLIGHT_CONTROLLER);
    FcSerial.write(frame, len);
    lastLinkStatsFcMs = now;
    statToFcLinkStats++;
  }

  pollUartErrors();
  updateLed();
  printStats();
}
