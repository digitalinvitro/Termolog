/* ============================================================
 *  Логгер температуры — STM32F401 + SSD1306 + DS18B20
 *  Основной файл (GLM.ino)
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
#include "config.h"

// ============================================================
//  Объекты библиотек
// ============================================================
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);
OneWire           oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// ============================================================
//  Температурные данные
// ============================================================
float currentTempC = 0.0f;
float minTempC     = 0.0f;
float maxTempC     = 0.0f;
bool  hasData      = false;

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

// Сырые коды последнего замера напряжения; пишутся при каждом вызове.
uint16_t g_vddRaw = 0, g_vddCal = 0;

// ============================================================
//  Вспомогательное: корректное определение нажатия кнопки
//  с антидребезгом и таймаутом.
//  Возвращает true, если кнопка действительно нажата.
// ============================================================
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

// Объявления функций из power.cpp
extern void disableUnusedPinsAndPeripherals();
extern void prepareStopMode();
extern void serialOffForStop_export();

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
      serialOffForStop_export();
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
    // состояние пина, а этоненадёжно (кнопка уже отпущена).
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
        serialOffForStop_export();
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
