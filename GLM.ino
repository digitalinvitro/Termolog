/* ============================================================
 *  Логгер температуры — STM32F401 + SSD1306 + DS18B20
 * ============================================================
 *
 * Аппаратная часть
 * ---------------
 *   - MCU:     STM32F401RBT6 (LQFP64, 128KB Flash, 64KB SRAM).
 *              Arduino IDE: Board → Generic STM32F4 series,
 *              Board Part Number → BlackPill F401RB (или Generic F401RBT6).
 *   - Дисплей: SSD1306 OLED 128x64, I2C (библиотека Adafruit)
 *              SCL -> PB6,  SDA -> PB7
 *              VCC -> 3V3,  GND -> GND
 *   - Датчик:  DS18B20 (1-Wire)
 *              DATA -> PB0
 *              VCC  -> 3V3, GND -> GND
 *              ПОДТЯГИВАЮЩИЙ резистор 4.7 кОм между DATA и 3V3 — обязателен!
 *   - Кнопка активности:  Внешний pull-up к 3V3 (пин в покое = HIGH).
 *              При нажатии замыкается на GND (пин = LOW).
 *              Один контакт -> PA15, второй -> GND.
 *              Будит MCU из SLEEP. В ACTIVE — продлевает активную минуту.
 *              (PA15 на STM32F4 по умолчанию занят JTAG/SWD — см. ниже
 *               освобождение пина в setup().)
 *   - Кнопка сброса: Внешний pull-up к 3V3 (пин в покое = HIGH).
 *              При нажатии замыкается на GND (пин = LOW).
 *              Один контакт -> PC11, второй -> GND.
 *              Работает ТОЛЬКО в ACTIVE. Сбрасывает min/max к текущему T.
 *   - Питание: Li-ion аккумулятор 3.7 В через встроенный LDO (3V3).
 *
 * Библиотеки (через Менеджер библиотек Arduino IDE)
 * -------------------------------------------------
 *   - Adafruit SSD1306
 *   - Adafruit GFX Library
 *   - OneWire                       (Paul Stoffregen)
 *   - DallasTemperature
 *   - STM32duino LowPower           (входит в пакет STM32 core)
 *
 * Настройка платы
 * ---------------
 *   1. В Preferences → Additional boards manager URLs добавьте:
 *      https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
 *   2. Boards Manager → установить "STM32 MCU based boards" (STMicroelectronics).
 *   3. Выберите плату: Generic STM32F4 series → Board Part Number
 *      "BlackPill F401RB" (или "Generic F401RBT6").
 *   4. USB-режим — на ваше усмотрение (для отладки Serial).
 *
 * Поведение программы
 * -------------------
 *   • На старте делается одно измерение, чтобы инициализировать
 *     минимум/максимум, после чего контроллер уходит в SLEEP.
 *     Дисплей при этом ВЫКЛЮЧЕН.
 *   • SLEEP: каждые 15 с контроллер просыпается по RTC-таймеру,
 *     измеряет температуру, обновляет диапазон min/max и снова
 *     уходит в глубокий сон. На дисплей ничего не выводится.
 *   • Нажатие кнопки будит MCU по спаду фронта на PA15.
 *     Контроллер переходит в ACTIVE на 1 минуту, дисплей ВКЛЮЧАЕТСЯ.
 *     В ACTIVE каждые 5 с экран обновляется:
 *        Строка 1 (крупный шрифт): текущая температура.
 *          Положительная — без знака, отрицательная — со знаком "−".
 *        Строка 2 (мелкий шрифт): диапазон "min − max" за всё время.
 *   • По истечении минуты дисплей выключается и MCU снова уходит в SLEEP.
 *   • Нажатие кнопки в ACTIVE — продлевает активную минуту.
 * ============================================================ */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <STM32RTC.h>
#include <STM32LowPower.h>
#include <math.h>

// ============================================================
//  Назначение пинов
// ============================================================
#define PIN_DS18B20   PB0       // 1-Wire шина данных
#define PIN_BUTTON    PA15      // Кнопка активности: внешний pull-up, активный LOW
#define PIN_RESET     PC11      // Кнопка сброса диапазона: внешний pull-up, активный LOW

// Пины I2C для OLED (Black Pill F401)
#define I2C_SDA       PB7
#define I2C_SCL       PB6

// Индикаторный светодиод. На плате термометра светодиод подключён
// катодом к GND, анодом через резистор к пину MCU — поэтому он
// горит при ВЫСОКОМ уровне (HIGH) на пине.
#define LED_PIN       PC13
#define LED_ON        HIGH
#define LED_OFF       LOW

// ============================================================
//  Параметры дисплея
// ============================================================
#define OLED_W        128
#define OLED_H        64
#define OLED_ADDR     0x3C
#define OLED_RESET    -1        // -1 = совместно с линией RESET MCU

// ============================================================
//  Временные константы
// ============================================================
static const uint32_t ACTIVE_DURATION_MS = 60000UL;   // 1 минута активного режима
static const uint32_t ACTIVE_REFRESH_MS  = 5000UL;    // обновление экрана каждые 5 с
static const uint32_t SLEEP_PERIOD_MS    = 15000UL;   // 15 с между замерами в SLEEP

// Флаг отладочного режима: если true — НЕ засыпать, а каждую секунду
// печатать время RTC в терминал. Так можно убедиться, что RTC живёт.
// После проверки переключите в false.
static const bool DEBUG_RTC = false;

// ============================================================
//  Выбор режима сна.
//  0 — Sleep-режим через LowPower.sleep().
//      Стабильный, но ток ~5-10 мА.
//  1 — STOP-режим через LowPower.deepSleep()
//      с полной оптимизацией по AN4899 и
//      образцу sketch_apr8a.ino. Ток ~50-200 мкА,
//      но требует реинициализации периферии
//      после пробуждения.
//
//  ВАЖНО: при USE_STOP_MODE = 1 USB-CDC неработоспособен после
//  пробуждения. Логирование переключается на USART1 (PA9=TX, PA10=RX),
//  если LOG_VIA_USART1 = 1. Подключите USB-TTL конвертер.
//
//  ФИКС F1: USE_STOP_MODE и LOG_VIA_USART1 — теперь МАКРОСЫ, а не
//  C++-переменные. Препроцессор '#if' не видит C++-идентификаторов:
//  любая неизвестная ему лексема подставляется как 0, поэтому запись
//      #if USE_STOP_MODE && LOG_VIA_USART1
//  при 'static const bool' всегда вычислялась как '#if 0 && 0':
//  ветка USART1 не компилировалась НИКОГДА, лог намертво уходил
//  в USB-CDC, а блок '#if !(USE_STOP_MODE && LOG_VIA_USART1)'
//  (ожидание энумерации USB в setup) оставался включённым.
//  Как макросы значения видны и препроцессору, и обычному коду:
//  runtime-проверки вида 'if (USE_STOP_MODE) {...}' работают как
//  прежде (компилятор сворачивает их в константу).
// ============================================================
#define USE_STOP_MODE 1

// === ТЕСТОВЫЙ РЕЖИМ: полное отключение логирования ===
// Цель: проверить, что USB-CDC инициализация (Serial.begin) активирует
// OTG_FS_WKUP interrupt (IRQ 31), который будит MCU из STOP.
// Без лога USB peripheral не инициализируется → STOP должен работать.
// После теста: закомментировать #define NO_LOG и раскомментировать LOG_BEGIN.
// #define NO_LOG

// 1 = при USE_STOP_MODE=1 весь лог идёт через аппаратный USART1 (Serial1)
//     вместо USB-CDC. Подключите USB-TTL конвертер: PA9->RX, PA10->TX (3V3).
//     Это «батарейный» режим: USB-периферия НЕ инициализируется вообще —
//     нет задержек на энумерацию (3 с в setup, ~550 мс после каждого
//     пробуждения) и не остаётся активной в STOP (главный подозреваемый
//     в «полке» 3.8-4.9 мА по замерам без USB-кабеля).
// 0 = лог через USB-CDC, как раньше (отладка за ПК; поведение прежнее).
#define LOG_VIA_USART1 1
#define NO_LOG

// Антидребезг кнопки: сколько мс ждать стабилизации уровня
static const uint16_t BUTTON_DEBOUNCE_MS = 50;
// Таймаут ожидания отпускания кнопки (защита от зависания,
// если пробуждение было ложным). После этого считаем, что кнопка не нажата.
static const uint16_t BUTTON_PRESS_TIMEOUT_MS = 3000;
// Время преобразования DS18B20 при 12-бит: ~750 мс
static const uint16_t DS18B20_CONV_MS = 750;
// ФАЗА 2 (энергопотребление вспышки SLEEP-цикла):
static const uint16_t LED_BLINK_MS  = 50;   // длительность видимого LED-блика «замер идёт»
static const uint16_t CONV_GUARD_MS = 100;  // запас к 750 мс: дрейф LSI (±1-3%) + субсекундная
                                            // сетка RTC-будильника; чтение ПОЗЖЕ конца
                                            // конверсии всегда безопасно (скретчпад готов)

// ============================================================
//  Состояния конечного автомата
// ============================================================
enum SystemState : uint8_t {
  ST_SLEEP  = 0,
  ST_ACTIVE = 1
};

SystemState state         = ST_SLEEP;
uint32_t    activeStartMs = 0;   // момент входа в ACTIVE
uint32_t    lastRefreshMs = 0;   // момент последнего обновления экрана

// ============================================================
//  Источник последнего пробуждения.
//  Определяется ПО ФЛАГАМ EXTI/RTC сразу после выхода из STOP,
//  ДО вызова wakeFromStop() (которая занимает ~600 мс).
//  Это гарантирует корректное определение даже при коротком
//  нажатии кнопки (50-200 мс) — кнопка уже отпущена, но флаг
//  EXTI->PR остаётся установленным (latched).
// ============================================================
enum WakeSource : uint8_t {
  WAKE_UNKNOWN  = 0,   // флаги не установлены (сбой/первый старт)
  WAKE_RTC      = 1,   // пробуждение по RTC Wakeup Timer (EXTI17)
  WAKE_BUTTON   = 2    // пробуждение по кнопке PA15 (EXTI15)
};

WakeSource lastWakeSource = WAKE_UNKNOWN;

// ============================================================
//  Volatile-флаги пробуждения, устанавливаются в ISR.
//  Используются вместо прямого чтения EXTI->PR / RTC->ISR.WUTF,
//  потому что STM32duino LowPower library очищает эти флаги
//  во внутренних обработчиках прерываний ДО возврата из deepSleep().
//  Наш callback запускается из ISR и устанавливает флаг —
//  он сохраняется до detectWakeSource().
// ============================================================
volatile bool wokenByButton = false;   // устанавливается в ISR кнопки PA15
volatile bool wokenByRTC    = false;   // устанавливается в ISR RTC Wakeup

// Callback-функции для LowPower.attachInterruptWakeup() и enableWakeupFrom().
// Запускаются в контексте прерывания (ISR) — должны быть минимальными.
// voidFuncPtrParam: void (*)(void*) — сигнатура с параметром-указателем,
// который библиотека использует для передачи данных (нам не нужен, но
// сигнатура должна совпадать).
void buttonWakeCallback() {
  wokenByButton = true;
}

void rtcWakeCallback(void * /*data*/) {
  wokenByRTC = true;
}

// ============================================================
//  Температурные данные
// ============================================================
float currentTempC = 0.0f;
float minTempC     = 0.0f;
float maxTempC     = 0.0f;
bool  hasData      = false;

// ============================================================
//  Объекты библиотек
// ============================================================
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);
OneWire           oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// ============================================================
//  Логирование с учётом режима сна.
//  При USE_STOP_MODE = true и LOG_VIA_USART1 = true весь лог
//  идёт через аппаратный USART1 (Serial1) вместо USB-CDC.
//  USART1 восстанавливается после STOP простой подачей тактирования,
//  в отличие от USB-CDC, который зависает на flush().
//
//  Подключение USB-TTL конвертера:
//    PA9  (TX)  -> RX конвертера
//    PA10 (RX)  -> TX конвертера
//    GND        -> GND
//    3V3        -> VCC (если конвертер без отдельного питания)
// ============================================================
// ФИКС F1: порядок веток изменён — NO_LOG (полное отключение лога)
// теперь имеет приоритет, затем USART1, и только потом USB-CDC.
// Раньше '#elif defined(NO_LOG)' стоял после '#if USE_STOP_MODE &&
// LOG_VIA_USART1', который из-за бага F1 всегда был ложным — но при
// LOG_VIA_USART1=1(NO_LOG) ветка NO_LOG оставалась недостижимой.
#if defined(NO_LOG)
  // Тестовый режим без логирования — все LOG_OBJECT.* превращаются в no-op.
  #define LOG_OBJECT  NoLogInstance
  #define LOG_BEGIN(baud)  ((void)0)
  struct NoLogClass {
    template<typename T> void print(T) {}
    template<typename T> void println(T) {}
    template<typename T, typename U> void print(T, U) {}
    template<typename T, typename U> void println(T, U) {}
    void flush() {}
    int availableForWrite() { return 64; }
  } NoLogInstance;
#elif USE_STOP_MODE && LOG_VIA_USART1
  // ФИКС ЛИНКОВКИ (undefined reference to `Serial1'):
  // ядро STM32duino ОБЪЯВЛЯЕТ 'extern Uart Serial1;' в Serial.h для
  // любого чипа с USART1, но СОЗДАЁТ объект 'Uart Serial1(USART1);'
  // в Serial.cpp только при HAVE_HWSERIAL1. Для варианта платы
  // «Generic F401RBTx» (и Nucleo-F401RE) variant.h задаёт
  // SERIAL_UART_INSTANCE=2 — ядро создаёт только Serial2 (USART2),
  // ENABLE_HWSERIAL1 не определён, Serial1 остаётся без определения:
  // компиляция проходит (extern-объявление есть), а линковка падает.
  // Решение: создаём объект сами — ровно той же строкой, что и ядро.
  // Конструктор не трогает железо (таблицы PinMap), пины задаются
  // ниже в LOG_BEGIN; ~192 байта RAM (2 x 64 Б буферы + дескриптор).
  // Альтернатива (НЕ использовать одновременно с этой строкой!):
  // файл build_opt.h рядом со скетчем со строкой «-DENABLE_HWSERIAL1».
  Uart Serial1(USART1);
  #define LOG_OBJECT  Serial1
  #define LOG_BEGIN(baud)  do { \
      Serial1.setTx(PA9); \
      Serial1.setRx(PA10); \
      Serial1.begin(baud); \
    } while(0)
#else
  #define LOG_OBJECT  Serial
  #define LOG_BEGIN(baud)  Serial.begin(baud)
#endif

// Безопасный flush с таймаутом. Serial.flush() для USB-CDC блокирует
// бесконечно, если USB-периферия нерабочая. safeSerialFlush ждёт
// не более timeout_ms, потом возвращает управление.
// ФИКС F1: опрашиваем LOG_OBJECT (USART1 / USB / NoLog), а не «сырой»
// Serial — в режиме USART1 дожидаемся опустошения именно аппаратного
// TX-буфера (64 байта при 115200 бод уходят за ~6 мс, так что задержка
// фактически исчезающе мала).
static inline void safeSerialFlush(uint32_t timeout_ms = 50) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (LOG_OBJECT.availableForWrite() == (int)SERIAL_TX_BUFFER_SIZE) break;
    delay(1);
  }
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
static void serialOffForStop() {
  safeSerialFlush(200);          // вытолкнуть последние строки в провод
#if !defined(NO_LOG) && USE_STOP_MODE && LOG_VIA_USART1
  Serial1.end();                 // USART1: сброс RCC + стоп тактирования
  pinMode(PA9,  INPUT_ANALOG);   // PA9/PA10 -> Analog (Hi-Z)
  pinMode(PA10, INPUT_ANALOG);
#endif
}

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
//  Инициализация I2C и OLED
//  ВАЖНО: на STM32F401 необходимо явно назначать SDA/SCL
//         через Wire.setSDA()/Wire.setSCL() ДО Wire.begin(),
//         иначе I2C-периферия остаётся на пинах по умолчанию
//         и экран молчит.
// ============================================================
bool initOLED() {
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire.setClock(100000);        // 100 кГц — стабильно для SSD1306
  delay(50);                    // пауза для стабилизации шины

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    LOG_OBJECT.println(F("[Thermo] OLED init FAILED"));
    return false;
  }
  display.clearDisplay();
  display.display();
  return true;
}

// ============================================================
//  Примечание о снижении тактовой частоты.
//  Изначально планировалось переключать SYSCLK с 84 МГц (HSE+PLL) на
//  4 МГц (HSI/4) для экономии энергии в SLEEP. На практике в Arduino-среде
//  это ломает USB-CDC (теряет синхронизацию), I2C и 1-Wire тайминги,
//  а также может ломать EXTI-конфигурацию кнопки. Поэтому от смены частоты
//  решено отказаться — оставляем стабильные 84 МГц в обоих режимах.
//  Если в будущем понадобится экономия энергии, лучше перейти на
//  полноценный STOP-режим с ручной настройкой RTC Wakeup Timer.
// ============================================================

// ============================================================
//  Вспомогательное: корректное определение нажатия кнопки
//  с антидребезгом и таймаутом.
//  Возвращает true, если кнопка действительно нажата.
// =============================================================
bool isButtonPressed() {
  if (digitalRead(PIN_BUTTON) != LOW) return false;  // активный LOW
  delay(BUTTON_DEBOUNCE_MS);
  return digitalRead(PIN_BUTTON) == LOW;
}

// Ждать отпускания кнопки с таймаутом (защита от зависания).
// ФИКС ЛОЖНОГО «wake: BUTTON»: цикл выше выходит при ПЕРВОМ уровне
// HIGH — но контакт кнопки в этот момент ещё дребезгит (серия
// LOW->HIGH->LOW переходов). Каждый спад = FALLING-фронт на EXTI15:
// ISR срабатывает уже ПОСЛЕ того, как detectWakeSource() потребил и
// очистил флаг пробуждения, и ставит wokenByButton=true «впрок».
// Флаг переживает всю активную фазу (его никто не чистит) и на
// СЛЕДУЮЩЕМ пробуждении — уже от RTC-таймера — ложно
// классифицирует источник как BUTTON (у флага кнопки приоритет).
// Лечение: окно тишины после отпускания + поглощение всего,
// что успело налетать (latched EXTI + NVIC + volatile-флаг).
void waitForButtonRelease() {
  uint32_t start = millis();
  while (digitalRead(PIN_BUTTON) == LOW) {
    if (millis() - start > BUTTON_PRESS_TIMEOUT_MS) {
      LOG_OBJECT.println(F("[Thermo] button release timeout"));
      break;
    }
    delay(10);
  }

  // Окно тишины: пока дребезг продолжается (пин снова LOW), окно
  // продлевается. Выходим только после BUTTON_DEBOUNCE_MS
  // непрерывного HIGH — фронты физически закончились.
  uint32_t quietUntil = millis() + BUTTON_DEBOUNCE_MS;
  while ((int32_t)(millis() - quietUntil) < 0) {
    if (digitalRead(PIN_BUTTON) == LOW) {
      quietUntil = millis() + BUTTON_DEBOUNCE_MS;
    }
    delay(2);
  }

  // Проглотить следы дребезга. Порядок важен: сначала pending NVIC,
  // затем latched EXTI->PR, и только потом volatile-флаг — под
  // маской прерываний, чтобы ISR не вписал его заново между
  // очистками. PA15 -> линия EXTI15 (при смене PIN_BUTTON
  // поменять и номер линии!).
  EXTI->PR = (1u << 15);
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  noInterrupts();
  wokenByButton = false;
  interrupts();
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

// ============================================================
//  Управление питанием OLED (SSD1306)
//  ВАЖНО: display.begin() вызывается ОДИН РАЗ в setup().
//  После глубокого сна повторно его вызывать нельзя —
//  Adafruit_SSD1306 это не поддерживает.
//
//  Согласно datasheet SSD1306, для МАКСИМАЛЬНОЙ экономии энергии
//  недостаточно одной команды 0xAE (DISPLAYOFF). Нужно дополнительно
//  выключить внутренний charge pump (DC-DC converter). Иначе ток
//  потребления остаётся ~50-100 мкА вместо возможных <10 мкА.
//
//  Порядок засыпания:
//    1) DISPLAYOFF  (0xAE) — гасит панель, останавливает oscillator и drivers
//    2) CHARGEPUMP  (0x8D) + 0x10 — выключает DC-DC charge pump
//
//  Порядок пробуждения (обратный):
//    1) CHARGEPUMP  (0x8D) + 0x14 — включает charge pump обратно
//    2) DISPLAYON   (0xAF) — включает панель
//    3) Небольшая задержка (~100 мс) для раскрутки charge pump.
//
//  Источники:
//    - SSD1306 datasheet, sec. 9 Command Table (Set DC-DC 0x8D)
//    - Adafruit forum: 0uA sleep achieved by combining 0xAE + charge pump off
//    - lexus2k/ssd1306 issue #103: sleep mode draws <10uA only with charge pump off
// ============================================================

// Команды SSD1306 (не все определены в Adafruit_SSD1306.h)
#define SSD1306_CHARGEPUMP       0x8D
#define SSD1306_CHARGEPUMP_ON    0x14
#define SSD1306_CHARGEPUMP_OFF   0x10

void oledPowerOff() {
  // Полное засыпание OLED: панель OFF + charge pump OFF.
  display.ssd1306_command(SSD1306_DISPLAYOFF);    // 0xAE
  display.ssd1306_command(SSD1306_CHARGEPUMP);    // 0x8D
  display.ssd1306_command(SSD1306_CHARGEPUMP_OFF);// 0x10
}

void oledPowerOn() {
  // Пробуждение OLED: charge pump ON + панель ON.
  display.ssd1306_command(SSD1306_CHARGEPUMP);     // 0x8D
  display.ssd1306_command(SSD1306_CHARGEPUMP_ON);  // 0x14
  display.ssd1306_command(SSD1306_DISPLAYON);      // 0xAF
  // Раскрутка charge pump: по даташиту 100 мс достаточно,
  // по факту на дешёвых модулях бывает до 200 мс.
  delay(100);
}

// ============================================================
//  ИЗМЕРЕНИЕ НАПРЯЖЕНИЯ ПИТАНИЯ (VDD = батарея) через VREFINT
// ============================================================
//  Напрямую измерить шину нельзя: VREF+ соединён с VDDA, а VDDA —
//  это сама батарея (4.0–4.2 В напрямую на шину). Классический приём
//  (RM0368 §15.3.6): измеряем внутренний источник опоры VREFINT
//  (~1.21 В) и сравниваем с заводской калибровкой из системной памяти:
//
//      VDDA = 3.0 В * VREFINT_CAL / ADC(VREFINT)
//
//  VREFINT_CAL — 12-битный код, откалиброванный при VDDA = 3.0 В,
//  лежит по адресу 0x1FFF7A2A (DS9716 «Calibration data»). Каналы
//  ADC F401: IN16 = датчик T, IN17 = VREFINT, IN18 = VBAT.
//
//  ВНИМАНИЕ к точности: спецификация VREFINT дана при VDDA 2.4–3.6 В,
//  а у нас 4.0–4.2 В (вся плата вне abs max — осознанное решение).
//  Показание годится как индикатор батареи, не как вольтметр.
//
//  Энергогигиена (аудит периферии, Task 7): функция самодостаточна —
//  поднимает клок ADC1 + TSVREFE на время замера (~100 мкс) и ГАСИТ их
//  в конце, восстанавливая ровно то состояние, которое задаёт
//  disableUnusedPinsAndPeripherals() (секция 4). Вызывается
//  только из drawScreen() в активной фазе — в STOP АЦП обесточен.
// ============================================================
#define VREFINT_CAL_ADDR_F401 ((const uint16_t*)0x1FFF7A2A) // калибровка при 3.0 В
#define VDDA_CAL_MV           3300.0f

//  ВНИМАНИЕ (проверено по вендоренной CMSIS ядра STM32duino, stm32f401xe.h):
//  у поля ADCPRE в CMSIS F4 есть только позиционные биты _0/_1 — DIV-алиасов
//  (DIV2/DIV4/DIV6/DIV8) нет НИ В ОДНОМ F4-заголовке (нет и у F407);
//  делители существуют лишь в HAL под именем ADC_CLOCK_SYNC_PCLK_DIVx
//  (именно его ядро ставит по умолчанию в analog.cpp). По RM0368 §11.12.9
//  код 01 = PCLK2/4, т.е. ADCPRE_DIV4 == ADCPRE_0 (0x00010000 = 21 МГц).
#ifndef ADC_CCR_ADCPRE_DIV4
#define ADC_CCR_ADCPRE_DIV4   ADC_CCR_ADCPRE_0   // PCLK2/4 = 84/4 = 21 МГц
#endif

//  Диагностика Task 8-b: 1 = вместо напряжения печатать сырые коды
//  "raw/cal" в том же слоте экрана (расшифровка — в комментарии там же).
//  Разделяет: нет заводской калибровки (маркер клона, Task 5) vs
//  сам VREFINT низкий при VDDA 4.0-4.2 В (вне spec).
#define VDD_DEBUG 0

//  Сырые коды последнего замера; пишутся при каждом вызове.
uint16_t g_vddRaw = 0, g_vddCal = 0;

uint16_t readVddMillivolts() {
  // 1. Клок ADC1 (APB2), прескалер и VREFINT — в общем регистре CCR.
  //    Прескалер ADC на F401 — это ADCPRE в ADC->CCR (RM0368), а НЕ
  //    RCC->CFGR (там он только у F1/F2). Сбросовое PCLK2/2 = 42 МГц
  //    выше спецификации (<=36 МГц) → ставим /4 = 21 МГц. Ядро
  //    STM32duino CCR не трогало, VBATE погашен в секции 4
  //    disableUnusedPinsAndPeripherals().
  __HAL_RCC_ADC1_CLK_ENABLE();
  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_DIV4;
  ADC->CCR |= ADC_CCR_TSVREFE;

  // 2. Одиночный замер: 12 бит, последовательность из 1, канал 17,
  //    сэмпл 480 циклов (внутренний источник высокоимпедансный).
  ADC1->SR    = 0;                 // сброс застоявшихся EOC/OVR
  ADC1->CR1   = 0;                 // RES = 00 → 12 бит
  ADC1->CR2   = ADC_CR2_ADON;      // АЦП включён, без DMA/EOCS
  ADC1->SMPR2 = (7U << 21);        // SMP17 = 111 → 480 цикла
  ADC1->SQR1  = 0;                 // L = 0 → 1 преобразование
  ADC1->SQR3  = 17U;               // SQ1 = IN17 (VREFINT)

  // 3. Стабилизация и холостая конверсия. RM0368: первая конверсия после
  //    подачи ADON может стартовать не раньше t_STAB (3 мкс), а VREFINT
  //    после TSVREFE=1 ещё и набирает номинал десятки мкс. Прежняя версия
  //    мерила сразу — ловила незрелый VREFINT (raw ~480 вместо ~1240 →
  //    "10.28V" вместо ~4.0 В). delayMicroseconds — API ядра STM32duino.
  delayMicroseconds(50);           // t_STAB + startup VREFINT, с запасом
  ADC1->CR2 |= ADC_CR2_SWSTART;    // холостая конверсия: разгон АЦП
  uint32_t guard = 100000;
  while (!(ADC1->SR & ADC_SR_EOC)) {
    if (--guard == 0) break;
  }
  (void)ADC1->DR;                  // холостой результат выбрасываем

  // 4. Рабочий замер (нужно ~23 мкс, ждём до ~1 мс).
  ADC1->CR2 |= ADC_CR2_SWSTART;
  guard = 100000;
  while (!(ADC1->SR & ADC_SR_EOC)) {
    if (--guard == 0) break;
  }
  uint32_t raw = ADC1->DR;         // чтение DR снимает EOC
  g_vddRaw = (uint16_t)raw;

  // 5. Обратно всё гасим — инвариант STOP: перед сном АЦП обесточен.
  ADC1->CR2 = 0;
  ADC->CCR &= ~ADC_CCR_TSVREFE;
  __HAL_RCC_ADC1_CLK_DISABLE();

  if (guard == 0 || raw < 64) return 0;  // таймаут/мусор — рисовать нечего
  uint32_t cal = *VREFINT_CAL_ADDR_F401; // заводской код при 3.0 В
  g_vddCal = (uint16_t)cal;
  return (uint16_t)(VDDA_CAL_MV * cal / raw + 0.5f);
}

// ============================================================
//  Печать числа со знаком (минус — только для отрицательных)
//  Используется для обеих строк экрана.
// ============================================================
void printTempSigned(float t) {
  if (t < 0.0f) display.print('-');
  // Абсолютное значение с одним знаком после запятой
  display.print(fabsf(t), 1);
}

// ============================================================
//  Отрисовка экрана
// ============================================================
void drawScreen() {
  display.clearDisplay();

  // --- Верхняя строка: текущая температура крупным шрифтом ---
  // Размер 3 -> 18×24 px на символ. "-23.4" → 5 символов = 90 px.
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 4);
  printTempSigned(currentTempC);

  // Градус-метка "C" крупным шрифтом справа от значения
  int16_t cx = display.getCursorX();
  display.setCursor(cx + 2, 4);
  display.print('C');

  // --- Разделитель ---
  display.drawFastHLine(0, 35, OLED_W, SSD1306_WHITE);

  // --- Нижняя строка: "min − max" ---
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 42);
  printTempSigned(minTempC);
  display.print(F(".."));
  printTempSigned(maxTempC);

  // --- Индикатор батареи: правый край той же строки ---
  // Формат "X.XXV" = 5 символов × 6 px = 30 px → x = 128-30 = 98.
  // Самый широкий диапазон ("−40.0 - 123.4") занимает <=78 px —
  // пересечений с правым краем нет.
  uint16_t vddMv = readVddMillivolts();
  display.setCursor(OLED_W - 50, 42);
  if (vddMv > 0) {
    display.print(vddMv / 1000.0f, 2);
    display.print('V');
  } else {
    display.print(F("--.-V"));
  }

  display.display();
}

// ============================================================
//  Определение источника пробуждения.
//  ВЫЗЫВАТЬ НЕМЕДЛЕННО после выхода из deepSleep.
//
//  Используются ДВА метода одновременно для максимальной надёжности:
//
//  Метод 1 (callback-флаги):
//    - volatile bool wokenByButton / wokenByRTC устанавливаются
//      в ISR через callback, переданный в attachInterruptWakeup().
//    - Недостаток: callback может не вызываться, если библиотека
//      конфигурирует EXTI как "event" (без ISR) в STOP-режиме.
//
//  Метод 2 (прямое чтение GPIO):
//    - Проверяем digitalRead(PIN_BUTTON) сразу после пробуждения.
//    - MCU просыпается за <1 мкс, кнопку держат минимум 50 мс.
//    - Если пин LOW → кнопка точно была нажата.
//    - Если пин HIGH → пробуждение от RTC (или кнопку уже отпустили,
//      но это маловероятно для коротких нажатий).
//
//  Приоритет: callback-флаг > GPIO-чтение.
//  Если ни один метод не сработал — WAKE_UNKNOWN (аномалия).
// ============================================================
WakeSource detectWakeSource() {
  WakeSource src = WAKE_UNKNOWN;

  // Метод 1: проверяем volatile-флаги из callbacks.
  if (wokenByButton) {
    src = WAKE_BUTTON;
  } else if (wokenByRTC) {
    src = WAKE_RTC;
  }

  // Метод 2: если callback не сработал — проверяем GPIO напрямую.
  // Это надёжный fallback: MCU просыпается мгновенно, кнопка ещё нажата.
  if (src == WAKE_UNKNOWN) {
    if (digitalRead(PIN_BUTTON) == LOW) {
      // Пин замкнут на GND → кнопка нажата → это пробуждение по кнопке.
      src = WAKE_BUTTON;
    } else {
      // Пин HIGH (внешний pull-up) → кнопка не нажата.
      // Если MCU проснулся без кнопки — это RTC-таймер.
      src = WAKE_RTC;
    }
  }

  // Сбрасываем volatile-флаги для следующего цикла.
  wokenByButton = false;
  wokenByRTC    = false;

  return src;
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
//  Восстановление после STOP-режима.
//  Вызывается в loop() после LowPower.deepSleep() при USE_STOP_MODE = true.
//
//  Стратегия: делаем МИНИМУМ необходимого.
//  Периферия, которую НЕ трогаем (оставляем как есть после STOP):
//    - I2C/OLED  (адрес и так валиден, достаточно DISPLAYON)
//    - OneWire   (пин PB0 уже сконфигурирован, не переинициализируем)
//  Что делаем:
//    1) Восстанавливаем SysTick (без него delay()/millis() не работают)
//    2) Включаем тактирование периферии (в STOP оно выключается)
//    3) Если LOG_VIA_USART1 — реинициализируем USART1
//    4) Иначе — пробуем реинициализировать USB-CDC (Serial.end + begin)
//    5) Перерегистрируем EXTI для кнопки пробуждения
// ============================================================
// ФАЗА 2: восстановление SysTick после ЛЮБОГО deepSleep() (STOP).
// Библиотека STM32LowPower не возобновляет SysTick сама — без этого
// delay()/millis() «заморожены». Вынесено в хелпер: вызывается и после
// основного сна (wakeFromStop), и после сна на время конверсии DS18B20.
void resumeSysTick() {
  HAL_ResumeTick();
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;
}

void wakeFromStop() {
  // МИНИМАЛЬНОЕ ВОССТАНОВЛЕНИЕ.
  // В STOP тактирование периферии автоматически выключается, но HAL_PWR_EnterSTOPMode
  // сам восстанавливает основное тактирование при пробуждении.
  // Нужно только восстановить SysTick, чтобы delay()/millis() работали.

  // 1. Восстанавливаем SysTick — БЕЗ ЭТОГО delay()/millis() не работают!
  resumeSysTick();

  // 2. Восстановление порта логирования.
  //    ФИКС F1: раньше USB-CDC пересоздавался здесь БЕЗУСЛОВНО —
  //    Serial.end() + delay(50) + Serial.begin() + delay(500) —
  //    это ~560 мс полного тактирования 84 МГц при КАЖДОМ пробуждении
  //    (0.56 с × 33 мА из каждых 15 с ≈ +1.2 мА к среднему току цикла),
  //    даже когда USB-кабель не подключён. Теперь ветка выбирается той
  //    же условной компиляцией, что и сам порт логирования.
#if defined(NO_LOG)
  // NO_LOG: порт логирования не существует — восстанавливать нечего.
#elif USE_STOP_MODE && LOG_VIA_USART1
  // USART1: перед каждым STOP порт деинициализируется serialOffForStop()
  // (PA9/PA10 уходят в Analog), поэтому на пробуждении достаточно поднять
  // его заново: LOG_BEGIN делает setTx/setRx/begin — пины возвращаются
  // в AF, UART готов сразу (бутстреп-время ничтожно, задержек не нужно).
  LOG_BEGIN(115200);
#else
  // Реинициализация USB-CDC после STOP (прежнее поведение).
  //    USB-периферия в STOP выключается, после пробуждения нужно
  //    пересоздать CDC-стек. Serial.end() + Serial.begin() делает
  //    переэнумерацию на стороне хоста.
  Serial.end();
  delay(50);
  Serial.begin(115200);
  delay(500);  // дать хосту время на переэнумерацию
#endif

  // 3. Перерегистрируем прерывание пробуждения —
  //    в STOP флаги EXTI могут быть сброшены.
  LowPower.attachInterruptWakeup(PIN_BUTTON, buttonWakeCallback, FALLING);

  // 4. Маленькая пауза стабилизации.
  delay(10);
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
  LowPower.attachInterruptWakeup(PIN_BUTTON, buttonWakeCallback, FALLING);
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
void enterActive() {
  state         = ST_ACTIVE;
  activeStartMs = millis();
  lastRefreshMs = 0;          // форсировать немедленное обновление
  oledPowerOn();
  if (LED_PIN >= 0) digitalWrite(LED_PIN, LED_ON);
  // Явная первая отрисовка — иначе на экране может остаться
  // мусор от предыдущего состояния или просто чёрный кадр,
  // так как SSD1306 не всегда корректно реактериризует GDDRAM
  // после команды DISPLAYON.
  if (readTemperature(currentTempC)) {
    updateStats(currentTempC);
  }
  drawScreen();
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  LOG_BEGIN(115200);
  // USB-CDC (виртуальный COM-порт) инициализируется не сразу —
  // нужно подождать, пока хост опросит устройство. Иначе первые
  // строки лога (включая «[Thermo] boot») будут потеряны.
  // Для аппаратного UART задержка не нужна, но она не вредит.
  // ФИКС F1: блок ожидания энумерации USB компилируется ТОЛЬКО в режиме
  // USB-CDC — при NO_LOG или LOG_VIA_USART1=1 он исключён из прошивки
  // (раньше из-за бага #if он был включён всегда: +3 с при старте).
#if !defined(NO_LOG) && !(USE_STOP_MODE && LOG_VIA_USART1)
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) {
    ;  // ждём подключения USB-CDC (но не более 3 с)
  }
#endif
  delay(100);  // дополнительная пауза для стабилизации
  LOG_OBJECT.println(F("\n[Thermo] boot"));

  // ФИКС F5 (мини): гасим биты DBGMCU DBG_SLEEP/DBG_STOP/DBG_STANDBY.
  // Если после отладочной сессии эти биты остались взведёнными (они
  // сбрасываются ТОЛЬКО полным обесточиванием!), ядро продолжает
  // тактироваться в STOP — ток сна вырастает с ~42 мкА до единиц мА.
  // Классическая ловушка при замерах энергопотребления (AN4899 /
  // ST Community «Tips for using STM32 low-power modes»). На всякий
  // случай снимаем их программно при каждом старте.
  HAL_DBGMCU_DisableDBGSleepMode();
  HAL_DBGMCU_DisableDBGStopMode();
  HAL_DBGMCU_DisableDBGStandbyMode();

  // Инициализация дисплея (с явной привязкой I2C-пинов)
  initOLED();
  oledPowerOff();

  // Датчик температуры
  sensors.begin();
  sensors.setResolution(12);          // максимальное разрешение
  sensors.setWaitForConversion(false); // сами ждём через delay()

  // Кнопка активности: внешний pull-up (в покое HIGH), при нажатии LOW.
  // Внутренний pull-up НЕ включаем — иначе конфликт с внешним.
  // Прерывание пробуждения — по спаду фронта FALLING.
  // ВАЖНО: PA15 на STM32F4 по умолчанию занят JTAG (JTDI).
  // STM32duino при вызове pinMode автоматически переключит пин
  // в режим GPIO — но JTAG будет недоступен. SWD остаётся рабочим.
  pinMode(PIN_BUTTON, INPUT);

  // Кнопка сброса диапазона: внешний pull-up, активный LOW.
  pinMode(PIN_RESET, INPUT);

  // Индикаторный светодиод (если есть)
  if (LED_PIN >= 0) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);
  }

  // === Инициализация RTC на LSI — обязательна для deepSleep ===
  // Без этого LowPower.deepSleep(timeout) не имеет источника
  // пробуждения по таймеру, и MCU остаётся спать вечно.
  STM32RTC& rtc = STM32RTC::getInstance();
  rtc.setClockSource(STM32RTC::LSI_CLOCK);
  rtc.begin(true);
  rtc.setTime(0, 0, 0);

  // Проверка: сразу выведем время в терминал.
  {
    uint8_t hh, mm, ss; uint32_t sub; STM32RTC::AM_PM ap;
    rtc.getTime(&hh, &mm, &ss, &sub, &ap);
    LOG_OBJECT.print(F("[Thermo] RTC after init: "));
    LOG_OBJECT.print(hh); LOG_OBJECT.print(':');
    LOG_OBJECT.print(mm); LOG_OBJECT.print(':');
    LOG_OBJECT.print(ss); LOG_OBJECT.print('.');
    LOG_OBJECT.println(sub);
  }

  // Подсистема низкого энергопотребления
  LowPower.begin();
  // Пробуждение по спаду фронта на PA15 (кнопка -> GND).
  // Передаём callback, который установит wokenByButton=true в ISR.
  LowPower.attachInterruptWakeup(PIN_BUTTON, buttonWakeCallback, FALLING);
  // Пробуждение по RTC Wakeup Timer — callback установит wokenByRTC=true.
  // Это нужно для корректного определения источника пробуждения в
  // detectWakeSource(), т.к. библиотека сама сбрасывает RTC->ISR.WUTF
  // во внутреннем обработчике до возврата из deepSleep().
  LowPower.enableWakeupFrom(&rtc, rtcWakeCallback);

  // Первичное измерение — инициализирует min/max
  if (readTemperature(currentTempC)) {
    updateStats(currentTempC);
    LOG_OBJECT.print(F("[Thermo] seed T = "));
    LOG_OBJECT.print(currentTempC, 2);
    LOG_OBJECT.println(F(" C"));
  } else {
    LOG_OBJECT.println(F("[Thermo] DS18B20 not responding"));
  }

  // === Стартовый сплэш-экран на 3 секунды ===
  // Нужен, чтобы визуально убедиться, что OLED живёт.
  // Если сплэш не появляется — проблема с инициализацией
  // I2C (пины, адрес, подтяжки), а не с логикой сна.
  oledPowerOn();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Thermo"));
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.print(F("T="));
  display.print(currentTempC, 1);
  display.print(F(" C"));
  display.setCursor(0, 40);
  display.print(F("Sleep 15s"));
  display.setCursor(0, 52);
  display.print(F("Btn = active"));
  display.display();
  delay(3000);

  // === Снижение энергопотребления ===
  // Настраиваем неиспользуемые пины в Analog Input и отключаем
  // тактирование неиспользуемой периферии (USART2/3, UART4/5, SPI1/2/3,
  // I2C2/3, TIM2-TIM7, TIM12-14, ADC1).
  // Вызываем ПОСЛЕ инициализации всех используемых модулей (OLED,
  // датчик, кнопки, RTC), но ДО первого перехода в сон.
  disableUnusedPinsAndPeripherals();

  if (DEBUG_RTC) {
    // ОТЛАДКА RTC: не засыпаем. Каждую секунду печатаем время,
    // чтобы убедиться, что счётчик идёт.
    LOG_OBJECT.println(F("[Thermo] DEBUG_RTC=1 — entering RTC test loop"));
    for (;;) {
      uint8_t hh, mm, ss; uint32_t sub; STM32RTC::AM_PM ap;
      rtc.getTime(&hh, &mm, &ss, &sub, &ap);
      LOG_OBJECT.print(F("RTC: "));
      LOG_OBJECT.print(hh); LOG_OBJECT.print(':');
      LOG_OBJECT.print(mm); LOG_OBJECT.print(':');
      LOG_OBJECT.print(ss); LOG_OBJECT.print('.');
      LOG_OBJECT.println(sub);
      delay(1000);
    }
  }

  LOG_OBJECT.println(F("[Thermo] entering SLEEP mode"));
  enterSleep();
}

// ============================================================
//  Главный цикл
// ============================================================
void loop() {
  if (state == ST_ACTIVE) {
    // === Блокирующий цикл ACTIVE ===
    // Используем millis() — в Sleep-режиме (в отличие от Stop)
    // SysTick продолжает работать, поэтому millis() корректен.
    //
    // Логика:
    //   • Каждые 5 с — обновление экрана.
    //   • При нажатии кнопки — продлеваем активную минуту
    //     и сразу обновляем экран.
    //   • Через 1 минуту без нажатий — уходим в сон.
    uint32_t activeUntil = millis() + ACTIVE_DURATION_MS;
    uint32_t nextRefresh = 0;  // 0 = немедленно

    while (millis() < activeUntil) {
      // Кнопка активности нажата (активный LOW)?
      if (isButtonPressed()) {
        // Продлеваем активную минуту
        activeUntil = millis() + ACTIVE_DURATION_MS;
        nextRefresh = 0;  // форсируем немедленное обновление
        waitForButtonRelease();
      }

      // Кнопка сброса диапазона нажата (активный LOW)?
      // Работает ТОЛЬКО в ACTIVE — в SLEEP не проверяем.
      if (digitalRead(PIN_RESET) == LOW) {
        delay(BUTTON_DEBOUNCE_MS);
        if (digitalRead(PIN_RESET) == LOW) {
          // Ждём отпускания кнопки сброса
          uint32_t t0 = millis();
          while (digitalRead(PIN_RESET) == LOW) {
            if (millis() - t0 > BUTTON_PRESS_TIMEOUT_MS) break;
            delay(10);
          }
          // Сбрасываем диапазон к текущей температуре
          resetStats();
          nextRefresh = 0;  // форсируем перерисовку
        }
      }

      // Время обновлять экран?
      if (nextRefresh == 0 || millis() >= nextRefresh) {
        if (readTemperature(currentTempC)) {
          updateStats(currentTempC);
          LOG_OBJECT.print(F("[Thermo] T="));
          LOG_OBJECT.print(currentTempC, 2);
          LOG_OBJECT.println(F(" C (active)"));
        }
        drawScreen();
        nextRefresh = millis() + ACTIVE_REFRESH_MS;
      }

      // Небольшая задержка, чтобы не крутиться вхолостую
      delay(100);
    }

    // Активное окно истекло — уходим в сон.
    LOG_OBJECT.println(F("[Thermo] active expired -> SLEEP"));
    enterSleep();
  }
  else { // ST_SLEEP
    // Дисплей уже выключен. Засыпаем.
    LOG_OBJECT.println(F("[Thermo] sleep zZz..."));

    if (USE_STOP_MODE) {
      // === STOP-режим ===
      // Полное отключение периферии, ток ~50-200 мкА.
      // ВАЖНО: millis() НЕ работает в STOP (SysTick остановлен)!
      // Поэтому измеряем реальное время сна через RTC.
      // STM32RTC.getEpoch() возвращает секунды с 1970 — монотонно растёт.
      STM32RTC& rtc = STM32RTC::getInstance();
      uint32_t t_before = rtc.getEpoch();
      // ФАЗА 2.1: UART гасим ДО входа в STOP. Раньше flush перед сном
      // делался только для Sleep-режима: в STOP хвост TX-буфера замерзал
      // вместе с UART и терялся — отсюда склейка «sleep zZz...[Thermo]
      // woke from STOP» в одну строку и пропажа строк между циклами.
      // Теперь flush выталкивает «sleep zZz...» в провод, а PA9/PA10
      // уходят в Analog — мост не подпитывается из батареи во время сна.
      serialOffForStop();
      LowPower.deepSleep(SLEEP_PERIOD_MS);
      uint32_t t_after = rtc.getEpoch();
      uint32_t slept = (t_after - t_before) * 1000UL;  // секунды → миллисекунды

      lastWakeSource = detectWakeSource();
      wakeFromStop();

      // ФАЗА 2: delay(50) «на стабилизацию» УБРАН — рудимент классификации
      // по уровню пина (isButtonPressed), заменённой флагами ISR.
      // Экономия: 50 мс полного тактирования на каждом цикле.
      //
      // ФАЗА 2.1: источник пробуждения — в лог ПЕРВЫМ делом после
      // оживления UART (свежий LOG_BEGIN — самое надёжное место печати).
      // Раньше строка «woke from STOP» не опознавала источник: RTC-события
      // выглядели так же, как кнопочные. slept считается по целым секундам
      // RTC (округление вниз): нажатие через ~1 с печатается как 1000.
      LOG_OBJECT.print(F("[Thermo] wake: "));
      if (lastWakeSource == WAKE_BUTTON) {
        LOG_OBJECT.print(F("BUTTON"));
      } else if (lastWakeSource == WAKE_RTC) {
        LOG_OBJECT.print(F("RTC"));
      } else {
        LOG_OBJECT.print(F("UNKNOWN"));
      }
      LOG_OBJECT.print(F(" (slept="));
      LOG_OBJECT.print(slept);
      LOG_OBJECT.println(F(" ms)"));
    } else {
      // === Sleep-режим ===
      // Безопасный flush с таймаутом: Serial.flush() блокирует навсегда,
      // если USB-CDC не работает после STOP.
      safeSerialFlush();
      LowPower.sleep(SLEEP_PERIOD_MS);
      // В Sleep-режime флаги EXTI/RTC также устанавливаются, поэтому
      // определение источника работает одинаково для обоих режимов.
      lastWakeSource = detectWakeSource();
    }

    // Пробуждение произошло — по таймеру или по кнопке.
    // ФАЗА 2: delay(5) «на стабилизацию уровня» убран — классификация
    // идёт по флагам ISR, уровень пина не читается.

    // Ветвление по определённому источнику пробуждения.
    // Источник уже напечатан выше («wake: RTC/BUTTON/UNKNOWN (slept=…)») —
    // дублирующих строк больше нет, «wake: BUTTON» встречается в логе
    // ровно один раз на каждое нажатие.
    // Больше НЕ используем isButtonPressed() — она проверяет текущее
    // состояние пина, а это ненадёжно (кнопка уже отпущена).
    if (lastWakeSource == WAKE_BUTTON) {
      waitForButtonRelease();
      enterActive();
    } else {
      // Сюда попадаем при WAKE_RTC или WAKE_UNKNOWN — замер температуры.

      // Пробуждение по таймеру — короткая вспышка + замер, без вывода на экран.
      //
      // ФАЗА 2 (энергопотребление вспышки): раньше 750 мс конверсии DS18B20
      // высиживались блокирующими delay() при полном тактировании 84 МГц
      // (~0.75 с × ~28 мА из каждых 15 с). Теперь на время конверсии MCU
      // уходит в STOP: бодрствование цикла сжимается до ~60-90 мс, а сама
      // конверсия идёт при токе полки — MCU спит, датчик работает (DS18B20
      // в 3-проводном включении с VDD не нуждается в мастере во время
      // конверсии; шина удерживается в high резистором ~4.7 кОм).
      if (LED_PIN >= 0) digitalWrite(LED_PIN, LED_ON);
      sensors.requestTemperatures();              // старт конверсии (неблокирующий)
      delay(LED_BLINK_MS);                        // видимый блик = «замер идёт»
      if (LED_PIN >= 0) digitalWrite(LED_PIN, LED_OFF);
      // ФАЗА 2.1: гарантируем, что 1-Wire шина ОТПУЩЕНА перед сном:
      // пин DQ в INPUT (floating), линия держится HIGH внешней подтяжкой
      // 4.7 кОм — тока через подтяжку нет. Защита от «залипания» линии
      // в лог.0 после транзакции (3.6 В / 4.7 кОм = 0.77 мА непрерывно).
      pinMode(PIN_DS18B20, INPUT);

      if (USE_STOP_MODE) {
        // Спим в STOP до конца конверсии. Кнопка может разбудить досрочно —
        // конверсия прервана, замер этого цикла приносится в жертву (чтение
        // скретчпада дало бы прошлоцикловое значение; ACTIVE всё равно
        // замерит T через мгновение). Бонус: нажатие во время конверсии
        // теперь обрабатывается немедленно, а не теряется.
        // ФАЗА 2.1: и здесь UART гасим ДО сна (serialOffForStop: flush уже
        // пуст — «wake: RTC…» ушла в провод ещё до блика), а поднимаем
        // ПОСЛЕ — симметрично основному сну.
        serialOffForStop();
        prepareStopMode();
        LowPower.deepSleep(DS18B20_CONV_MS + CONV_GUARD_MS - LED_BLINK_MS);
        resumeSysTick();
#if !defined(NO_LOG) && USE_STOP_MODE && LOG_VIA_USART1
        // UART был остановлен перед конверсионным сном — поднимаем заново
        // (setTx/setRx/begin вернут пины в AF). В конфигурациях USB/NO_LOG
        // пропускаем: USB-CDC всё равно пересоздаётся следующим wakeFromStop().
        LOG_BEGIN(115200);
#endif
        if (wokenByButton) {
          // Нажатие в окне бодрствования (между пробудкой и конверсионным
          // сном) или во время самого конверсионного сна. Раньше этот
          // путь не был опознан в логе — активация выглядела «из ниоткуда»,
          // сразу после RTC-строки, без объяснения, кто её вызвал.
          LOG_OBJECT.println(F("[Thermo] wake: BUTTON (in wake window)"));
          LOG_OBJECT.flush();
          waitForButtonRelease();
          enterActive();
        } else {
          readAndLogSleepTemp();   // «wake: TIMER  T=…  range=[…]»
        }
      } else {
        // Sleep-режим (USE_STOP_MODE=0): прежняя блокирующая схема ожидания.
        delay(DS18B20_CONV_MS + CONV_GUARD_MS - LED_BLINK_MS);
        readAndLogSleepTemp();
      }
      // Состояние определит следующий виток loop():
      // ST_SLEEP -> новый сон, ST_ACTIVE -> активная минута (кнопка).
    }
  }
}
