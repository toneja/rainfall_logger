/************************************************************************
* Rainfall Datalogger                                                   *
*  - Texas Electronics TR-525M                                          *
*                                                                       *
* Copyright (c) 2026 Jason Toney                                        *
*                                                                       *
* This program is free software: you can redistribute it and/or modify  *
* it under the terms of the GNU General Public License as published by  *
* the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                   *
*                                                                       *
* This program is distributed in the hope that it will be useful,       *
* but WITHOUT ANY WARRANTY; without even the implied warranty of        *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
* GNU General Public License for more details.                          *
*                                                                       *
* You should have received a copy of the GNU General Public License     *
* along with this program. If not, see <https://www.gnu.org/licenses/>. *
************************************************************************/

// Board Manager index:
// https://raw.githubusercontent.com/RAKwireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless_index.json
#include <bluefruit.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // http://librarymanager/All#SparkFun%20u-blox%20GNSS
#include <U8g2lib.h>                               // http://librarymanager/All#U8g2
#include "SD.h"                                    // http://librarymanager/All#SD

#define DEBUG 1

// DISPLAY
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2);  // R2 = Rotate display 180°
char displayMsg[32];
#define OLED_REFRESH_MS 1000UL
uint32_t lastOledMs = 0;

// TEXAS ELECTRONICS TR-525M
#define RAIN_PIN WB_IO3
#define MM_PER_TIP 0.1f
#define HOUR_MS 3600000UL
#define DEBOUNCE_US 5000UL
#define TIP_QUEUE_SIZE 32
volatile uint32_t tipHead = 0;               // total tips accepted by ISR
volatile uint32_t tipTimes[TIP_QUEUE_SIZE];  // millis() stamp of each tip
volatile uint32_t lastTipUs = 0;
volatile bool firstTip = true;
uint32_t reportedTips = 0;  // tips consumed by processTips()
uint32_t hourStartMs = 0;   // millis() when current bin began
uint32_t currentHour = 0;   // monotonic hour index
uint32_t hourlyBins[24] = { 0 };
float hourlyRainfall = 0.0f;
float dailyRainfall = 0.0f;
float totalRainfall = 0.0f;

// BLUETOOTH
BLEDfu bledfu;  // OTA updates
#define MAX_SAMPLERS 4
typedef struct {
  char name[16 + 1];
  uint16_t conn_handle;
  BLEClientUart clientUart;
} Peripheral;
Peripheral peripherals[MAX_SAMPLERS];
uint8_t connections = 0;
#define BLE_BUF_SIZE 20  // default BLEUart packet size
char bleMsg[BLE_BUF_SIZE];
char bleName[12] = "RAIN LOGGER";

// Log files
File csvFile;

// GPS: position + timestamp
SFE_UBLOX_GNSS g_myGNSS;
char timestamp[20];

void setup() {
#if DEBUG
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);  // wait up to 3 s for a monitor
  Serial.printf("Starting %s\n", bleName);
#endif

  pinMode(RAIN_PIN, INPUT_PULLUP);

  hourStartMs = millis();

  attachInterrupt(
    digitalPinToInterrupt(RAIN_PIN),
    tipISR,
    FALLING);

  sensor_init();
  oled_init();
  led_init();
  sd_init();
  gps_init();
  ble_init();
  oled_update();
}

void loop() {
  processTips();

  // Timer-driven OLED refresh keeps values current during quiet periods
  // and hour rollovers, independent of tip activity.
  if (millis() - lastOledMs >= OLED_REFRESH_MS) {
    lastOledMs = millis();
    oled_update();
  }
}

void sensor_init(void) {
  // power up the sensors
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(1000);
}

// DISPLAY
void oled_init(void) {
  u8g2.begin();
  u8g2.setContrast(255);  // Brightness range [0-255]
  u8g2.setFont(u8g2_font_9x15_tf);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 15, bleName);
  u8g2.sendBuffer();
}

void oled_update(void) {
  u8g2.clearBuffer();
  u8g2.drawStr(0, 15, bleName);
  memset(displayMsg, 0, sizeof(displayMsg));
  snprintf(displayMsg, sizeof(displayMsg), "Hourly  %.1fmm", hourlyRainfall);
  u8g2.drawStr(0, 30, displayMsg);
  memset(displayMsg, 0, sizeof(displayMsg));
  snprintf(displayMsg, sizeof(displayMsg), "Daily   %.1fmm", dailyRainfall);
  u8g2.drawStr(0, 45, displayMsg);
  memset(displayMsg, 0, sizeof(displayMsg));
  snprintf(displayMsg, sizeof(displayMsg), "Total   %.1fmm", totalRainfall);
  u8g2.drawStr(0, 60, displayMsg);
  u8g2.sendBuffer();
}

void led_init(void) {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  led_loop(10);
}

void led_loop(int count) {
  for (uint8_t i = 0; i < count * 2; i++) {
    digitalToggle(LED_GREEN);
    delay(100);
    digitalToggle(LED_BLUE);
    delay(100);
  }
}

void sd_init(void) {
  // Check if card is inserted
  pinMode(WB_IO6, INPUT_PULLUP);
  if (!digitalRead(WB_IO6) == LOW) { return; }
  if (!SD.begin()) { return; }
#if DEBUG
  Serial.println("SD Card mounted.");
#endif
  char csvFilename[12 + 1];
  for (uint8_t i = 0; i <= 99; i++) {
    snprintf(csvFilename, sizeof(csvFilename), "RAIN_%02d.csv", i);
    if (!SD.exists(csvFilename)) { break; }
  }
#if DEBUG
  Serial.printf("Output file: %s\n", csvFilename);
#endif
  csvFile = SD.open(csvFilename, FILE_WRITE);
  if (!csvFile) { return; }
  if (csvFile.size() == 0) {
    csvFile.println("Date,Time(UTC),Tips,Hourly,Daily,Total");
    csvFile.flush();
  }
}

void gps_init(void) {
  if (!g_myGNSS.begin()) { return; }
  g_myGNSS.setI2COutput(COM_TYPE_UBX);
  g_myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  // Wait on the GPS fix for accurate timestamps
#if DEBUG
  Serial.print("Searching for GPS...");
  uint16_t fixTime = 0;
#endif
  u8g2.clearBuffer();
  u8g2.drawStr(5, 30, "SEARCHING FOR");
  u8g2.drawStr(19, 45, "GPS SIGNAL");
  u8g2.sendBuffer();
  while (g_myGNSS.getFixType() < 3) {
    digitalToggle(LED_GREEN);
    digitalToggle(LED_BLUE);
    delay(1000);
#if DEBUG
    Serial.print(".");
    fixTime++;
    if (fixTime == 10) { break; }  // don't loop forever
#endif
  }
  // make sure LEDs are off
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
#if DEBUG
  Serial.printf("GPS fix acquired in %d seconds.\n", fixTime);
#endif
}

void gps_gettime(void) {
  snprintf(timestamp,
           sizeof(timestamp),
           "%d-%02d-%02d,%02d:%02d:%02d",
           g_myGNSS.getYear(), g_myGNSS.getMonth(), g_myGNSS.getDay(),
           g_myGNSS.getHour(), g_myGNSS.getMinute(), g_myGNSS.getSecond());
}

void tipISR() {
  uint32_t now = micros();

  if (firstTip || (uint32_t)(now - lastTipUs) >= DEBOUNCE_US) {
    firstTip = false;
    lastTipUs = now;
    tipTimes[tipHead % TIP_QUEUE_SIZE] = millis();  // stamp each tip
    tipHead++;
  }
}

// Advance the hourly bins to nowMs. Unsigned subtraction keeps this correct
// across the ~49.7-day millis() rollover. Never moves backward.
void updateHourlyBins(uint32_t nowMs) {
  int32_t sinceStart = (int32_t)(nowMs - hourStartMs);
  if (sinceStart < (int32_t)HOUR_MS) { return; }  // still inside current hour

  uint32_t elapsed = (uint32_t)sinceStart / HOUR_MS;  // whole hours passed

  if (elapsed >= 24) {
    memset(hourlyBins, 0, sizeof(hourlyBins));
  } else {
    for (uint32_t i = 1; i <= elapsed; i++) {
      hourlyBins[(currentHour + i) % 24] = 0;
    }
  }

  currentHour += elapsed;
  hourStartMs += elapsed * HOUR_MS;
}

void recalcRainfall(void) {
  uint32_t dailyTips = 0;
  for (uint8_t i = 0; i < 24; i++) {
    dailyTips += hourlyBins[i];
  }
  hourlyRainfall = hourlyBins[currentHour % 24] * MM_PER_TIP;
  dailyRainfall = dailyTips * MM_PER_TIP;
}

void processTips() {
  // Read the clock before the tip count. Any tip stamped before snapNow is
  // already in the ring when tipHead is read, so the hour clock can safely
  // advance to snapNow at the end without binning a pre-boundary tip into
  // the new hour.
  uint32_t snapNow = millis();
  uint32_t head;

  noInterrupts();
  head = tipHead;
  interrupts();

  // If the ISR lapped the consumer, the oldest tips were overwritten; skip them.
  if ((uint32_t)(head - reportedTips) > TIP_QUEUE_SIZE) {
    reportedTips = head - TIP_QUEUE_SIZE;
  }

  while (reportedTips < head) {
    uint32_t ts;
    noInterrupts();
    ts = tipTimes[reportedTips % TIP_QUEUE_SIZE];
    interrupts();
    reportedTips++;

    updateHourlyBins(ts);  // bin each tip at its own timestamp
    hourlyBins[currentHour % 24]++;
    totalRainfall += MM_PER_TIP;
    recalcRainfall();

    // Send BLE data
    if (Bluefruit.Central.connected()) {
      digitalWrite(LED_BLUE, HIGH);
      ble_send();
      delay(2);
      digitalWrite(LED_BLUE, LOW);
    }

    gps_gettime();
    if (csvFile) {
      csvFile.printf(
        "%s | TIP #%lu | Hour: %.1f mm | 24h: %.1f mm | Total: %.1f mm\n",
        timestamp,
        (unsigned long)reportedTips,
        hourlyRainfall,
        dailyRainfall,
        totalRainfall);
      csvFile.flush();
    }

#if DEBUG
    Serial.printf(
      "%s | TIP #%lu | Hour: %.1f mm | 24h: %.1f mm | Total: %.1f mm\n",
      timestamp,
      (unsigned long)reportedTips,
      hourlyRainfall,
      dailyRainfall,
      totalRainfall);
#endif
  }

  // Roll the clock forward even when no tips arrived, so an empty hour clears.
  updateHourlyBins(snapNow);
  recalcRainfall();
}

bool ble_init(void) {
  // Bandwidth and connection settings must be set before Bluefruit.begin().
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);
  // Disable connection LED
  Bluefruit.autoConnLed(false);
  if (!Bluefruit.begin(0, MAX_SAMPLERS)) { return false; }
  Bluefruit.setTxPower(8);  // Check bluefruit.h for supported values
  Bluefruit.setName(bleName);
  Bluefruit.Central.setConnectCallback(central_connect_callback);
  Bluefruit.Central.setDisconnectCallback(central_disconnect_callback);
  bledfu.begin();
  for (uint8_t id = 0; id < MAX_SAMPLERS; id++) {
    memset(peripherals[id].name, 0, sizeof(peripherals[id].name));
    peripherals[id].conn_handle = BLE_CONN_HANDLE_INVALID;
    peripherals[id].clientUart.begin();
  }
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);
  return true;
}

void ble_send(void) {
  snprintf(bleMsg,
           sizeof(bleMsg),
           "%.1f,%.1f,%.1f",
           hourlyRainfall,
           dailyRainfall,
           totalRainfall);
  // Send data to all peripherals, let them decide whether or not to sample
  if (Bluefruit.Central.connected()) {
    for (uint8_t id = 0; id < MAX_SAMPLERS; id++) {
      Peripheral* peer = &peripherals[id];
      if (peer->clientUart.discovered()) {
        peer->clientUart.write(bleMsg);
      }
    }
  }
}

void scan_callback(ble_gap_evt_adv_report_t* report) {
#if DEBUG
  Serial.print("BLE UART service detected. Connecting ... ");
#endif
  Bluefruit.Central.connect(report);
}

void central_connect_callback(uint16_t conn_handle) {
  int id = findConnHandle(BLE_CONN_HANDLE_INVALID);
  if (id < 0) { return; }
  Peripheral* peer = &peripherals[id];
  peer->conn_handle = conn_handle;
  Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name) - 1);
#if DEBUG
  Serial.printf("Connected to %s\n", peer->name);
#endif
  if (peer->clientUart.discover(conn_handle)) {
#if DEBUG
    Serial.println("Found it");
#endif
    peer->clientUart.enableTXD();
    connections++;
    if (connections < MAX_SAMPLERS) { Bluefruit.Scanner.start(0); }
  } else {
#if DEBUG
    Serial.println("Found NONE");
#endif
    Bluefruit.disconnect(conn_handle);
  }
}

void central_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  int id = findConnHandle(conn_handle);
  if (id < 0) { return; }
  memset(peripherals[id].name, 0, sizeof(peripherals[id].name));
  peripherals[id].conn_handle = BLE_CONN_HANDLE_INVALID;
  connections--;
  Bluefruit.Scanner.start(0);
#if DEBUG
  Serial.printf("%s disconnected, reason = %#X\n", peripherals[id].name, reason);
#endif
}

int findConnHandle(uint16_t conn_handle) {
  for (uint8_t id = 0; id < MAX_SAMPLERS; id++) {
    if (conn_handle == peripherals[id].conn_handle) { return id; }
  }
  return -1;
}
