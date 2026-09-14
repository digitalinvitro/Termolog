/* ============================================================
 *  Функции управления энергией и питанием микроконтроллера
 *  Логгер температуры — STM32F401 + SSD1306 + DS18B20
 * ============================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>
#include "config.h"

// Внешние объекты и переменные
extern Adafruit_SSD1306 display;
extern Print& LOG_OBJECT;
extern OneWire oneWire;
extern volatile bool wokenByButton;
extern volatile bool wokenByRTC;

// Пины
extern const int PIN_BUTTON;
extern const int PIN_DS18B20;
extern const int LED_PIN;
extern const int LED_ON;
extern const int LED_OFF;

// Состояния системы
extern enum SystemState : uint8_t {
  ST_SLEEP  = 0,
  ST_ACTIVE = 1
};
extern SystemState state;

// Источники пробуждения
extern enum WakeSource : uint8_t {
  WAKE_UNKNOWN  = 0,
  WAKE_RTC      = 1,
  WAKE_BUTTON   = 2
};
extern WakeSource lastWakeSource;

// Функции из других модулей
extern void oledPowerOff();
extern void wakeFromStop();
extern void resumeSysTick();
extern void readAndLogSleepTemp();
extern void enterActive();

// ============================================================
//  Снижение энергопотребления: настройка неиспользуемых пинов
//  и отключение неиспользуемой периферии.
//
//  Согласно ST Application Note AN4899 (GPIO software guidelines
//  for power optimization), неиспользуемые пины следует настраивать
//  в режим Analog Input. Это:
//    1) отключает входной буфер Шмитта (он осциллирует на floating
//       пине и потребляет 50-500 мкА на пин);
//    2) отключает pull-up/pull-down резисторы (~50-100 мкА на пин);
//    3) отключает выходной драйвер.
//
//  Также важно полностью отключить АЦП (даже когда он "не используется",
//  его внутренний регулятор остаётся активным и потребляет 50-200 мкА).
//  И выключить тактирование неиспользуемой периферии: в Sleep-режиме
//  тактирование не останавливается автоматически, поэтому SPI/USART/TIM
//  продолжают потреблять, если их явно не выключить.
//
//  Список "занятых" пинов на нашей плате F401RBT6 (LQFP64):
//    PA11, PA12 — USB DM/DP   (трогать нельзя — теряем USB-CDC;
//                 исключение: при NO_LOG — в Analog, ветка 1 ниже)
//    PA13, PA14 — SWDIO/SWCLK (трогать нельзя — теряем отладку/прошивку)
//    PA15       — кнопка активности
//    PB0        — DS18B20 (1-Wire)
//    PB6, PB7   — I2C для OLED
//    PC11       — кнопка сброса
//    PC13       — индикаторный светодиод
//    PC14, PC15 — LSE-кварц (если есть; на BlackPill F401 обычно нет)
//    PD2        — на LQFP64 единственный выведенный пин порта D.
//                 Если не используется в схеме — переводим в Analog.
//    PH0, PH1   — HSE-кварц (трогать нельзя — теряем тактирование)
// ============================================================

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

void disableUnusedPinsAndPeripherals() {
  // --- 1. Неиспользуемые пины порт A: в режим Analog ---
  // PA13/PA14 (SWD), PA15 (кнопка) — не трогаем всегда.
  // Раскладка остальных — по веткам ниже: PA11/PA12 (USB DM/DP) в Analog
  // только когда USB мёртв (NO_LOG); PA9/PA10 — в Analog, кроме USART1-режима.
  // ФИКС F1: при LOG_VIA_USART1=1 пины PA9/PA10 заняты аппаратным
  // USART1 (порт логирования) — переводить их в Analog НЕЛЬЗЯ:
  // эта функция вызывается в setup() ПОСЛЕ LOG_BEGIN и молча
  // переключила бы UART-пины в Analog (симптом: лог печатается до
  // строки «[Thermo] entering SLEEP mode» и замолкает навсегда).
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = 0;
  // ============================================================
  // ПРАВКА 5.3 (v2, объединённая цепочка): порт A паркуется ОДНИМ
  // вызовом HAL_GPIO_Init, раскладка пинов — по трём веткам.
  //
  // Порядок веток зеркалит приоритеты выбора LOG_OBJECT выше
  // (там NO_LOG тоже первый): сначала NO_LOG, затем USART1, и
  // только потом живой USB-CDC. Ставить первой ветку
  // «!defined(NO_LOG) && USE_STOP_MODE && LOG_VIA_USART1» с
  // PA11/PA12 в #else НЕЛЬЗЯ: этот #else накрывает и режим USB-CDC,
  // где PA11/PA12 заняты OTG FS, а Serial.begin() уже отработал в
  // setup() (наш вызов идёт ПОСЛЕ него). GPIO_MODE_ANALOG уводит пин
  // из альтернативной функции AF10 — для хоста это выглядит как
  // физическое выдёргивание кабеля: лог через USB-CDC замолкает.
  // ============================================================
#if defined(NO_LOG)
  // NO_LOG: USB не инициализируется вовсе (LOG_BEGIN — no-op,
  // Serial.begin() не вызывается, USB-ветка в wakeFromStop()
  // исключена препроцессором). Свободен весь порт A, кроме
  // PA13/PA14 (SWD) и PA15 (кнопка) — паркуем PA0-PA12.
  // Плавающие D+/D- иначе держат входные буферы Шмитта в осцилляции
  // (AN4899): десятки-сотни мкА, кандидат в «полке» тока сна 0.77 мА.
  // Достаточно ОДНОЙ настройки здесь: prepareStopMode() GPIO не
  // трогает, USB-стек не запускается — переопределять пины некому.
  GPIO_InitStruct.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
                        GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  |
                        GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 |
                        GPIO_PIN_11 | GPIO_PIN_12;
#elif USE_STOP_MODE && LOG_VIA_USART1
  // «Батарейный» USART1-режим (без NO_LOG): PA9/PA10 обслуживают
  // USART1 — не трогаем (ФИКС F1 выше). PA11/PA12 здесь тоже мертвы
  // и висят в воздухе; при желании добавить GPIO_PIN_11 | GPIO_PIN_12
  // и в эту ветку (USB периферия в этом режиме не инициализируется).
  GPIO_InitStruct.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
                        GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  |
                        GPIO_PIN_8;
#else
  // USB-CDC режим: лог через Serial, устройство уже перечислено.
  // PA11/PA12 (DM/DP, AF10 OTG FS) — НЕ ТРОГАЕМ. PA9/PA10 свободны
  // (USART1 не используется) — в Analog.
  GPIO_InitStruct.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
                        GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  |
                        GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10;
#endif
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // --- 2. Неиспользуемые пины порт B: PB1-PB5, PB8-PB15 ---
  // PB0 (1-Wire), PB6/PB7 (I2C) — не трогаем.
  GPIO_InitStruct.Pin = GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
                        GPIO_PIN_5  | GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 |
                        GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                        GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // --- 3. Неиспользуемые пины порт C: PC0-PC10, PC12, PC14, PC15 ---
  // PC11 (кнопка сброса), PC13 (LED) — не трогаем.
  // PC14/PC15 (LSE) — если кварц не распаян, переводим в Analog.
  // Если кварц есть — закомментируйте PC14/PC15 ниже!
  GPIO_InitStruct.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
                        GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  |
                        GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 | GPIO_PIN_12 |
                        GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // --- 3b. Порт D: на F401RBT6 выведен только PD2 (LQFP64) ---
  // Если PD2 не используется в схеме — переводим в Analog.
  // ВНИМАНИЕ: если используете PD2 в своём проекте — закомментируйте!
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // --- 4. Полное отключение АЦП ---
  // По умолчанию в STM32duino АЦП инициализирован (для analogRead).
  // Даже когда ADC не используется в скетче, его регулятор остаётся
  // активным. Отключаем тактирование АЦП напрямую.
  __HAL_RCC_ADC1_CLK_DISABLE();
  // Дополнительно: выключаем внутренний датчик температуры и VREFINT.
  // Это важный источник потребления (~50 мкА), который включён по умолчанию.
  ADC->CCR &= ~(ADC_CCR_TSVREFE | ADC_CCR_VBATE);

  // --- 5. Отключение тактирования неиспользуемой периферии ---
  // В Sleep-режиме тактирование не останавливается автоматически.
  // Перечислено только то, что есть на F401RBT6 и не используется в скетче.
  // USART2 — может использоваться STM32duino для Serial2. Если не нужен —
  // отключаем. USART1 оставляем (для Serial).
  __HAL_RCC_USART2_CLK_DISABLE();
  // SPI1/2/3 — на F401RBT6 доступны SPI1 и SPI2. SPI3 отсутствует на F401.
  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_SPI2_CLK_DISABLE();
  // I2C2 — не используется. I2C1 нужен для OLED. I2C3 отсутствует на F401.
  __HAL_RCC_I2C2_CLK_DISABLE();
  // TIM2-TIM5 — доступны на F401RBT6. TIM6/TIM7/TIM12-14 отсутствуют!
  // TIM1 оставляем (его может использовать HAL для delay()/micros()).
  __HAL_RCC_TIM2_CLK_DISABLE();
  __HAL_RCC_TIM3_CLK_DISABLE();
  __HAL_RCC_TIM4_CLK_DISABLE();
  __HAL_RCC_TIM5_CLK_DISABLE();
  // CAN1 — отсутствует на F401 (есть на F407/F429).
  // SDIO — отсутствует на F401.

  LOG_OBJECT.println(F("[Thermo] unused pins & peripherals disabled"));
}

// ============================================================
//  ФАЗА 2.1: полная остановка порта логирования ПЕРЕД каждым
//  входом в STOP (и основным 15-секундным, и конверсионным 0.8-с).
//  Вызывается ТОЛЬКО перед LowPower.deepSleep(); после каждого
//  пробуждения UART поднимается заново (wakeFromStop / конверсионный
//  блок через LOG_BEGIN). Решает две задачи:
//
//  1) ПОТЕРЯ СТРОК ЛОГА. safeSerialFlush() перед сном вызывался
//     только в Sleep-режиме (USE_STOP_MODE=0). В STOP хвост TX-буфера
//     замерзал вместе с UART: хвост строки «sleep zZz...» терялся,
//     соседние строки склеивались в одну («sleep zZz...[Thermo] woke
//     from STOP»), а в промежуточных билдах пропадали и строки замеров.
//     Теперь flush выполняется перед КАЖДЫМ засыпанием.
//
//  2) УТЕЧКА В ОБЕСТОЧЕННЫЙ МОСТ. Пока USART1 инициализирован, PA9
//     (TX) держит HIGH всю длину сна. Если USB-TTL мост подключён к
//     плате проводами, а его USB-кабель выдернут, PA9 через входной
//     защитный диод моста обратнозапитывает обесточенный CH340
//     (~0.5-1 мА из батареи на всю длину сна). Важно: по исходникам
//     ядра (uart.c: uart_deinit) Serial1.end() сбрасывает только сам
//     USART через RCC и НЕ трогает GPIO — пин остался бы в AF-режиме
//     с «замороженным» выходом. Поэтому ЯВНО переводим PA9/PA10 в
//     INPUT_ANALOG (Hi-Z, без подтяжек): нет ни утечки в мост, ни
//     осцилляции входного буфера на floating-пине (рекомендация
//     AN4899 — неиспользуемые пины в Analog).
//
//  Симметричность: вызов и на пути загрузки (первый сон), и на пути
//  цикла — GPIO-состояния ВСЕХ снов идентичны, что упрощает разбор
//  замеров тока (ступеньки «первый сон / повторные сны»).
// ============================================================

// Объявление функции safeSerialFlush из GLM.ino
extern void safeSerialFlush(uint32_t timeout_ms);

static void serialOffForStop() {
  safeSerialFlush(200);          // вытолкнуть последние строки в провод
#if !defined(NO_LOG) && USE_STOP_MODE && LOG_VIA_USART1
  Serial1.end();                 // USART1: сброс RCC + стоп тактирования
  pinMode(PA9,  INPUT_ANALOG);   // PA9/PA10 -> Analog (Hi-Z)
  pinMode(PA10, INPUT_ANALOG);
#endif
}

// ============================================================
//  Подготовка к STOP-режиму — полная оптимизация по AN4899
//  и образцу sketch_apr8a.ino.
//  Вызывается из enterSleep() только при USE_STOP_MODE = true.
// ============================================================
void prepareStopMode() {
  // МИНИМАЛЬНАЯ ПОДГОТОВКА — только то, что безопасно и не ломает пробуждение.
  // Ранее здесь было много кода, который отключал периферию и прерывания,
  // но это приводило к проблемам: STOP не входил (slept=11ms), пробуждение
  // по RTC/кнопке ломалось. Теперь делаем минимум.

  // 1. Очищаем флаги пробуждения — чтобы не сработали мгновенно после входа.
  EXTI->PR = 0xFFFFFFFF;
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

  // 2. Конфигурируем PWR для STOP с LP-регулятором (LPDS=1).
  // Это единственная настройка, которая реально снижает потребление
  // и не мешает пробуждению. FPDS, отключение периферии, USB — убраны,
  // т.к. они либо ломали пробуждение, либо не давали эффекта.
  // HAL_PWR_EnterSTOPMode внутри LowPower.deepSleep() сам установит
  // LPDS через MODIFY_REG, но дублируем для надёжности.
  PWR->CR |=  PWR_CR_LPDS;
  PWR->CR &= ~PWR_CR_PDDS;  // гарантия STOP, не Standby
}

// ============================================================
//  Переходы между состояниями
// ============================================================
void enterSleep() {
  state = ST_SLEEP;
  // ФАЗА 2: clearDisplay()+display() УБРАНЫ. Раньше перед каждым засыпанием
  // гонялся полный I2C-кадр (1 КБ GDDRAM @ 100 кГц ≈ 95-100 мс при полном
  // тактировании 84 МГц) при УЖЕ ВЫКЛЮЧЕННОМ дисплее — только чтобы
  // «зачистить» память на будущее включение. В этом нет нужды: drawScreen()
  // в ACTIVE каждый кадр начинается с clearDisplay() и полностью
  // перерисовывает экран, так что первое же обновление (оно идёт немедленно,
  // lastRefreshMs = 0) затирает прошлое состояние. Побочный эффект: первые
  // ~100 мс после пробуждения видно «замороженное» прошлое состояние —
  // естественное поведение просыпающегося устройства.
  oledPowerOff();
  if (LED_PIN >= 0) digitalWrite(LED_PIN, LED_OFF);
  // ВАЖНО: после выхода из сна внутренние флаги EXTI для пина
  // кнопки могут быть сброшены, поэтому перед каждым засыпанием
  // нужно перерегистрировать прерывание пробуждения.
  LowPower.attachInterruptWakeup(PIN_BUTTON, [](){}, FALLING);
  // Дождаться отправки буфера UART/USB-CDC — иначе последние
  // строки лога могут «исчезнуть» вместе с переходом в сон.
  //
  // safeSerialFlush() с таймаутом БЕЗОПАСНА и в Sleep, и в STOP:
  //   - В Sleep-режиме: USB-CDC работает, flush вытолкнёт все байты.
  //   - В STOP-режиме: USB-CDC сейчас работает (после wakeFromStop),
  //     flush вытолкнёт байты ПЕРЕД тем, как prepareStopMode погасит
  //     USB-периферию. Если flush НЕ сделать — данные в TX-буфере
  //     пропадут после перехода в STOP.
  //
  // НЕ безопасна только «сырая» Serial.flush() без таймаута — она
  // может блокировать бесконечно, если USB-CDC нерабочая.
  // safeSerialFlush ждёт не более 200 мс и возвращает управление.
  safeSerialFlush(200);

  // Если включён STOP-режим — дополнительная подготовка.
  if (USE_STOP_MODE) {
    prepareStopMode();
  }

  // ФИКС ЛОЖНОГО «wake: BUTTON» (гигиена флагов перед сном).
  // Раньше volatile-флаги wokenByButton/wokenByRTC очищались
  // ТОЛЬКО в detectWakeSource() — т.е. потреблялись в момент
  // отчёта. Любой EXTI15-фронт во время активной фазы (дребезг
  // кнопки на размыкании — см. waitForButtonRelease) оставлял
  // wokenByButton=true на весь следующий сон, и пробуждение по
  // RTC-таймеру (slept=15000!) ложно отчётывалось как BUTTON.
  // Теперь флаги гасятся непосредственно перед засыпанием, под
  // маской прерываний: классифицируются только фронты, случившиеся
  // ВО ВРЕМЯ сна. prepareStopMode() выше уже сбросил EXTI->PR и
  // NVIC-pending; здесь остаётся погасить сами volatile-флаги.
  // Побочный эффект: нажатие в последние ~200 мс перед засыпанием
  // (окно safeSerialFlush) будет подхвачено не текущим, а следующим
  // пробуждением — классификация при этом останется честной.
  noInterrupts();
  wokenByButton = false;
  wokenByRTC    = false;
  interrupts();
}

// Экспорт функции serialOffForStop для использования в GLM.ino
extern "C" void serialOffForStop_export() {
  serialOffForStop();
}
