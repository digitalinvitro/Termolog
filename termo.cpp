/* ============================================================
 *  Функции обработки температуры с датчика DS18B20
 *  Логгер температуры — STM32F401 + SSD1306 + DS18B20
 * ============================================================ */

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "config.h"

// Объект OneWire и DallasTemperature
extern OneWire oneWire;
extern DallasTemperature sensors;

// Переменные состояния температуры
extern float currentTempC;
extern float minTempC;
extern float maxTempC;
extern bool hasData;

// Логирование
extern Print& LOG_OBJECT;

// Функция безопасного flush из GLM.ino
extern void safeSerialFlush(uint32_t timeout_ms);

// ============================================================
//  Вспомогательное: чтение температуры с DS18B20
// ============================================================
bool readTemperature(float &out) {
  sensors.requestTemperatures();
  // Блокирующее ожидание преобразования. 12-бит ≈ 750 мс.
  // Это приемлемо: 0.75 с из каждых 15 с (≈5 %) в режиме сна.
  delay(DS18B20_CONV_MS);

  float t = sensors.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C || t <= -55.0f || t >= 125.0f) {
    return false;            // ошибка датчика / обрыв / КЗ
  }
  out = t;
  return true;
}

// ============================================================
//  Вспомогательное: обновление минимума/максимума
// ============================================================
void updateStats(float t) {
  if (!hasData) {
    minTempC = maxTempC = t;
    hasData  = true;
  } else {
    if (t < minTempC) minTempC = t;
    if (t > maxTempC) maxTempC = t;
  }
}

// ============================================================
//  Сброс диапазона температур
// ============================================================
void resetStats() {
  minTempC = currentTempC;
  maxTempC = currentTempC;
  LOG_OBJECT.println(F("[Thermo] stats RESET by user"));
}

// ============================================================
//  ФАЗА 2: замер и лог SLEEP-цикла. Вызывается ПОСЛЕ завершённой
//  конверсии DS18B20. Выделено из loop(), т.к. конверсия теперь
//  высиживается двумя способами: deepSleep (STOP-режим) или delay.
// ============================================================
void readAndLogSleepTemp() {
  float t = sensors.getTempCByIndex(0);
  if (t != DEVICE_DISCONNECTED_C && t > -55.0f && t < 125.0f) {
    currentTempC = t;
    updateStats(t);
    LOG_OBJECT.print(F("[Thermo] wake: TIMER  T="));
    LOG_OBJECT.print(currentTempC, 2);
    LOG_OBJECT.print(F("  range=["));
    LOG_OBJECT.print(minTempC, 2);
    LOG_OBJECT.print(F(" .. "));
    LOG_OBJECT.print(maxTempC, 2);
    LOG_OBJECT.println(F("]"));
    safeSerialFlush(100);
  } else {
    LOG_OBJECT.println(F("[Thermo] wake: TIMER  DS18B20 error"));
  }
}
