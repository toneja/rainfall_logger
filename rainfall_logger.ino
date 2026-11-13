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
#include <Adafruit_TinyUSB.h>
#define DEBUG 1

// TEXAS ELECTRONICS TR-525M
#define RAIN_PIN WB_IO1
#define MM_PER_TIP 0.1f
#define HOUR_MS 3600000UL
#define DEBOUNCE_US 5000UL
volatile uint32_t tipCount = 0;
volatile uint32_t lastTipMs = 0;
volatile uint32_t lastTipUs = 0;
volatile bool firstTip = true;
uint32_t reportedTips = 0;
uint32_t currentHour = 0;
uint32_t hourlyBins[24] = { 0 };
float hourlyRainfall = 0.0f;
float dailyRainfall = 0.0f;
float totalRainfall = 0.0f;

void setup() {
#if DEBUG
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);  // wait up to 3 s for a monitor
  Serial.printf("Starting TR-525M Logger\n");
#endif

  pinMode(RAIN_PIN, INPUT_PULLUP);

  currentHour = millis() / HOUR_MS;

  attachInterrupt(
    digitalPinToInterrupt(RAIN_PIN),
    tipISR,
    FALLING);
}

void loop() {
  updateHourlyBins();
  processTips();
}

void tipISR() {
  uint32_t now = micros();

  if (firstTip || (uint32_t)(now - lastTipUs) >= DEBOUNCE_US) {
    firstTip = false;
    lastTipUs = now;
    lastTipMs = millis();
    tipCount++;
  }
}

void updateHourlyBins() {
  uint32_t hour = millis() / HOUR_MS;
  uint32_t elapsed = hour - currentHour;

  if (elapsed > 0) {
    if (elapsed >= 24) {
      memset(hourlyBins, 0, sizeof(hourlyBins));
    } else {
      for (uint32_t i = 1; i <= elapsed; i++) {
        hourlyBins[(currentHour + i) % 24] = 0;
      }
    }

    currentHour = hour;
  }

  hourlyRainfall = hourlyBins[currentHour % 24] * MM_PER_TIP;

  uint32_t dailyTips = 0;
  for (uint8_t i = 0; i < 24; i++) {
    dailyTips += hourlyBins[i];
  }

  dailyRainfall = dailyTips * MM_PER_TIP;
}

void processTips() {
  uint32_t tips;
  uint32_t timestamp;

  noInterrupts();
  tips = tipCount;
  timestamp = lastTipMs;
  interrupts();

  while (reportedTips < tips) {
    reportedTips++;

    hourlyBins[currentHour % 24]++;
    totalRainfall += MM_PER_TIP;

    updateHourlyBins();

#if DEBUG
    uint32_t seconds = timestamp / 1000UL;

    Serial.printf(
      "%02lu:%02lu:%02lu | TIP #%lu | Hour: %.1f mm | 24h: %.1f mm | Total: %.1f mm\n",
      (unsigned long)(seconds / 3600UL),
      (unsigned long)((seconds / 60UL) % 60UL),
      (unsigned long)(seconds % 60UL),
      (unsigned long)reportedTips,
      hourlyRainfall,
      dailyRainfall,
      totalRainfall);
#endif
  }
}
