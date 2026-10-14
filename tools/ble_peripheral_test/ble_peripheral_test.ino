// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// ============================================================================
// TOOL: BLE peripheral test
//
// A bare BLE peripheral for bench testing a central's connection behaviour. It
// advertises, accepts a connection, notifies one fixed-content data packet at
// 20 Hz, and prints each connection event and each inbound write to the
// console as timestamped hex.
//
// It has no GNSS, IMU or battery hardware: the packet's contents are constants
// with a little noise. Nothing is ever sent in reply to a write.
//
// Target: Seeed XIAO nRF52840 (Seeeduino core), serial console at 115200.
// ============================================================================

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

#if !defined(NRF52840_XXAA)
#error "ble_peripheral_test supports the nRF52840 boards only"
#endif

#include <bluefruit.h>

#include <atomic>

namespace {

// Advertised identity.
constexpr char kNamePrefix[] = "Gnimu-RB";
constexpr char kManufacturer[] = "Gnimu";
constexpr char kFirmwareRev[] = "3.3";
constexpr char kHardwareRev[] = "1";
constexpr char kServiceUuid[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kTxUuid[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // notify
constexpr char kRxUuid[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // write
constexpr int kTxPowerDbm = -12;

// Data packet.
constexpr size_t kPacketBytes = 88;
constexpr uint32_t kPacketPeriodMs = 50; // 20 Hz

// Events queued by the BLE callbacks (Bluefruit's task) for the loop to print.
enum class EventKind : uint8_t { Connect, Disconnect, Subscribe, Write };

struct Event {
  uint32_t atMs;
  EventKind kind;
  uint16_t len;   // Write: bytes written (may exceed kMaxCaptured)
  uint16_t value; // Subscribe: CCCD value; Disconnect: reason
  uint8_t data[247];
};

constexpr size_t kQueueSize = 32;
Event queue[kQueueSize];
std::atomic<uint32_t> qHead{0}; // written by the callbacks
std::atomic<uint32_t> qTail{0}; // written by the loop
std::atomic<uint32_t> qDropped{0};

BLEService service{BLEUuid(kServiceUuid)};
BLECharacteristic txChar{BLEUuid(kTxUuid)};
BLECharacteristic rxChar{BLEUuid(kRxUuid)};
BLEDis deviceInfo;
BLEBas batteryService;

std::atomic<uint16_t> connHandle{BLE_CONN_HANDLE_INVALID};
std::atomic<bool> subscribed{false};

Event *beginEvent(EventKind kind) {
  const uint32_t head = qHead.load(std::memory_order_relaxed);
  if (head - qTail.load(std::memory_order_acquire) >= kQueueSize) {
    qDropped.fetch_add(1, std::memory_order_relaxed);
    return nullptr;
  }
  Event *e = &queue[head % kQueueSize];
  e->atMs = millis();
  e->kind = kind;
  e->len = 0;
  e->value = 0;
  return e;
}

void commitEvent() {
  qHead.store(qHead.load(std::memory_order_relaxed) + 1,
              std::memory_order_release);
}

void onConnect(uint16_t handle) {
  connHandle.store(handle, std::memory_order_relaxed);
  if (Event *e = beginEvent(EventKind::Connect)) {
    e->value = handle;
    commitEvent();
  }
}

void onDisconnect(uint16_t, uint8_t reason) {
  connHandle.store(BLE_CONN_HANDLE_INVALID, std::memory_order_relaxed);
  subscribed.store(false, std::memory_order_relaxed);
  if (Event *e = beginEvent(EventKind::Disconnect)) {
    e->value = reason;
    commitEvent();
  }
}

void onCccd(uint16_t, BLECharacteristic *, uint16_t value) {
  subscribed.store((value & 0x0001) != 0, std::memory_order_relaxed);
  if (Event *e = beginEvent(EventKind::Subscribe)) {
    e->value = value;
    commitEvent();
  }
}

void onRxWrite(uint16_t, BLECharacteristic *, uint8_t *data, uint16_t len) {
  if (Event *e = beginEvent(EventKind::Write)) {
    e->len = len;
    memcpy(e->data, data, len < sizeof(e->data) ? len : sizeof(e->data));
    commitEvent();
  }
}

// Little-endian store.
void putLe(uint8_t *p, uint32_t v, size_t bytes) {
  for (size_t i = 0; i < bytes; i++)
    p[i] = static_cast<uint8_t>(v >> (8 * i));
}

// Small pseudo-random jitter in [-range, +range].
int32_t jitter(int32_t range) {
  static uint32_t s = 12345;
  s = s * 1664525u + 1013904223u;
  return static_cast<int32_t>((s >> 16) %
                              static_cast<uint32_t>(2 * range + 1)) -
         range;
}

// Builds the 88-byte packet: B5 62 FF 01, 80-byte payload, Fletcher-8 checksum.
void buildPacket(uint32_t iTowMs, uint8_t (&out)[kPacketBytes]) {
  memset(out, 0, sizeof(out));
  uint8_t *p = out + 6;

  const uint32_t secOfDay = (iTowMs / 1000) % 86400;
  putLe(p + 0, iTowMs, 4);
  putLe(p + 4, 2026, 2);
  p[6] = 10;
  p[7] = 13;
  p[8] = static_cast<uint8_t>(secOfDay / 3600);
  p[9] = static_cast<uint8_t>((secOfDay / 60) % 60);
  p[10] = static_cast<uint8_t>(secOfDay % 60);
  p[11] = 0x07;         // date, time, fully resolved
  putLe(p + 12, 25, 4); // time accuracy, ns
  putLe(p + 16, 0, 4);  // nanoseconds
  p[20] = 3;            // 3D fix
  p[21] = 0x01;         // fix OK
  p[22] = 0xE0;         // time, date, confirmed
  p[23] = 14;           // satellites
  putLe(p + 24, static_cast<uint32_t>(-980000000 + jitter(5)),
        4); // lon, 1e-7 deg
  putLe(p + 28, static_cast<uint32_t>(390000000 + jitter(5)), 4); // lat
  putLe(p + 32, 250000, 4);                                       // height, mm
  putLe(p + 36, 220000, 4); // MSL height, mm
  putLe(p + 40, 800, 4);    // horizontal accuracy, mm
  putLe(p + 44, 1200, 4);   // vertical accuracy, mm
  putLe(p + 48, static_cast<uint32_t>(jitter(30) + 30),
        4);                  // ground speed, mm/s
  putLe(p + 52, 9000000, 4); // heading, 1e-5 deg
  putLe(p + 56, 150, 4);     // speed accuracy, mm/s
  putLe(p + 60, 1000000, 4); // heading accuracy, 1e-5 deg
  putLe(p + 64, 120, 2);     // PDOP, 0.01
  p[67] = 87;                // battery
  const int16_t accel[3] = {static_cast<int16_t>(jitter(6)),
                            static_cast<int16_t>(jitter(6)),
                            static_cast<int16_t>(1000 + jitter(6))}; // milli-g
  const int16_t gyro[3] = {static_cast<int16_t>(jitter(8)),
                           static_cast<int16_t>(jitter(8)),
                           static_cast<int16_t>(jitter(8))}; // centi-deg/s
  for (int i = 0; i < 3; i++) {
    putLe(p + 68 + 2 * i, static_cast<uint16_t>(accel[i]), 2);
    putLe(p + 74 + 2 * i, static_cast<uint16_t>(gyro[i]), 2);
  }

  out[0] = 0xB5;
  out[1] = 0x62;
  out[2] = 0xFF;
  out[3] = 0x01;
  out[4] = 80;
  out[5] = 0;
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6 + 80; i++) {
    a = static_cast<uint8_t>(a + out[i]);
    b = static_cast<uint8_t>(b + a);
  }
  out[86] = a;
  out[87] = b;
}

// UBX-style frame summary of a write, or nothing when it isn't one.
void printFrameSummary(const uint8_t *d, size_t len) {
  if (len < 8 || d[0] != 0xB5 || d[1] != 0x62)
    return;
  const uint16_t payload = static_cast<uint16_t>(d[4] | (d[5] << 8));
  Serial.printf("          frame: class 0x%02X id 0x%02X payload %u bytes",
                d[2], d[3], payload);
  if (static_cast<size_t>(payload) + 8 != len) {
    Serial.printf(" (write is %u bytes, frame needs %u)\n",
                  static_cast<unsigned>(len),
                  static_cast<unsigned>(payload) + 8u);
    return;
  }
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6u + payload; i++) {
    a = static_cast<uint8_t>(a + d[i]);
    b = static_cast<uint8_t>(b + a);
  }
  const bool ok = a == d[6 + payload] && b == d[7 + payload];
  Serial.printf(", checksum %s\n", ok ? "ok" : "BAD");
}

void printEvent(const Event &e) {
  switch (e.kind) {
  case EventKind::Connect:
    Serial.printf("[%8lu ms] connected (handle %u)\n",
                  static_cast<unsigned long>(e.atMs), e.value);
    break;
  case EventKind::Disconnect:
    Serial.printf("[%8lu ms] disconnected (reason 0x%02X)\n",
                  static_cast<unsigned long>(e.atMs), e.value);
    break;
  case EventKind::Subscribe:
    Serial.printf("[%8lu ms] notifications %s (CCCD 0x%04X)\n",
                  static_cast<unsigned long>(e.atMs),
                  (e.value & 1) ? "on" : "off", e.value);
    break;
  case EventKind::Write: {
    const size_t kept = e.len < sizeof(e.data) ? e.len : sizeof(e.data);
    Serial.printf(
        "[%8lu ms] write %u bytes:", static_cast<unsigned long>(e.atMs), e.len);
    for (size_t i = 0; i < kept; i++)
      Serial.printf(" %02X", e.data[i]);
    Serial.println();
    printFrameSummary(e.data, kept);
    break;
  }
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000)
    delay(10);

  // A stable ten-digit suffix from the chip's factory ID.
  char suffix[11];
  snprintf(suffix, sizeof(suffix), "%010lu",
           static_cast<unsigned long>(
               (static_cast<uint64_t>(NRF_FICR->DEVICEID[1]) << 32 |
                NRF_FICR->DEVICEID[0]) %
               4000000000ULL));
  char name[32];
  snprintf(name, sizeof(name), "%s %s", kNamePrefix, suffix);

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  if (!Bluefruit.begin()) {
    Serial.println("BLE stack failed to start");
    return;
  }
  Bluefruit.setName(name);
  Bluefruit.setTxPower(kTxPowerDbm);
  Bluefruit.autoConnLed(false);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  deviceInfo.setManufacturer(kManufacturer);
  deviceInfo.setModel(kNamePrefix);
  deviceInfo.setSerialNum(suffix);
  deviceInfo.setFirmwareRev(kFirmwareRev);
  deviceInfo.setHardwareRev(kHardwareRev);
  deviceInfo.begin();
  batteryService.begin();
  batteryService.write(87);

  const uint16_t maxMtu = Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH);
  service.begin();
  txChar.setProperties(CHR_PROPS_NOTIFY);
  txChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  txChar.setMaxLen(maxMtu);
  txChar.setCccdWriteCallback(onCccd);
  txChar.begin();
  rxChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  rxChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  rxChar.setMaxLen(maxMtu);
  rxChar.setWriteCallback(onRxWrite, true);
  rxChar.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(service);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  const bool ok = Bluefruit.Advertising.start(0);

  Serial.println("BLE peripheral test");
  Serial.printf("advertising as \"%s\": %s\n", name, ok ? "ok" : "FAILED");
}

void loop() {
  static uint32_t nextPacketMs = 0;
  static uint32_t iTowMs = 43200000; // 12:00:00

  // Print everything the callbacks queued.
  uint32_t tail = qTail.load(std::memory_order_relaxed);
  while (tail != qHead.load(std::memory_order_acquire)) {
    printEvent(queue[tail % kQueueSize]);
    qTail.store(++tail, std::memory_order_release);
  }
  const uint32_t dropped = qDropped.exchange(0, std::memory_order_relaxed);
  if (dropped)
    Serial.printf("(%lu events lost: queue full)\n",
                  static_cast<unsigned long>(dropped));

  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextPacketMs) < 0)
    return;
  nextPacketMs += kPacketPeriodMs;
  if (static_cast<int32_t>(now - nextPacketMs) > 0)
    nextPacketMs = now + kPacketPeriodMs;
  iTowMs += kPacketPeriodMs;

  const uint16_t handle = connHandle.load(std::memory_order_relaxed);
  if (handle == BLE_CONN_HANDLE_INVALID ||
      !subscribed.load(std::memory_order_relaxed))
    return;
  BLEConnection *c = Bluefruit.Connection(handle);
  if (!c || c->getMtu() < kPacketBytes + 3)
    return; // one whole packet per notification
  uint8_t packet[kPacketBytes];
  buildPacket(iTowMs, packet);
  txChar.notify(handle, packet, sizeof(packet));
}
