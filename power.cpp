/* ============================================================
 *  power.cpp — управление энергией MCU и OLED-дисплея
 * ============================================================
 *
 *  Содержит:
 *   - ISR-callbacks и volatile-флаги пробуждения
 *     (buttonWakeCallback / rtcWakeCallback,
 *      wokenByButton / wokenByRTC);
 *   - detectWakeSource()     — классификация источника пробуждения
 *                              (флаги ISR + прямое чтение GPIO);
 *   - serialOffForStop()     — остановка порта логирования ПЕРЕД
 *                              входом в STOP (flush + пины в Analog);
 *   - disableUnusedPinsAndPeripherals() — парковка неиспользуемых
 *                              пинов в Analog Input и отключение
 *                              тактирования периферии (AN4899);
 *   - prepareStopMode()      — настройка PWR/EXTI перед STOP;
 *   - resumeSysTick() / wakeFromStop() — восстановление после STOP;
 *   - oledPowerOff() / oledPowerOn() — питание SSD1306
 *                              (DISPLAYOFF + charge pump);
 *   - readVddMillivolts()    — измерение напряжения батареи
 *                              через VREFINT (RM0368 §15.3.6).
 *
 *  Константы (VREFINT_CAL_ADDR_F401, VDDA_CAL_MV, команды
 *  OLED_CMD_CHARGEPUMP* и пр.) — в config.h.
 *  HAL-заголовки (stm32f4xx_hal.h / _gpio.h) подключаются
 *  через config.h.
 * ============================================================ */

#include "config.h"

// Контроль синхронности версий файлов: все 6 файлов скетча должны
// быть из ОДНОГО архива (см. GLM_CONFIG_VERSION в config.h).
#if !defined(GLM_CONFIG_VERSION) || GLM_CONFIG_VERSION < 20
#error "config.h устарел (нужна v20): замените ВСЕ 6 файлов скетча из актуального архива"
#endif

// ============================================================
//  Volatile-флаги пробуждения, устанавливаются в ISR.
//  Используются вместо прямого чтения EXTI->PR / RTC->ISR.WUTF,
//  потому что STM32duino LowPower library очищает эти флаги
//  во внутренних обработчиках прерываний ДО возврата из deepSleep().
//  Наш callback запускается из ISR и устанавливает флаг —
//  он сохраняется до detectWakeSource().
// ============================================================
volatile bool wokenByButton = false;   // устанавливается в ISR кнопки PIN_BUTTON
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
//  Полная остановка порта логирования ПЕРЕД каждым входом в STOP
//  (и основным 15-секундным, и конверсионным ~0.15-с). Вызывается
//  ТОЛЬКО перед LowPower.deepSleep(); после каждого пробуждения
//  UART поднимается заново (wakeFromStop / waitForConversion
//  через LOG_BEGIN). Решает две задачи:
//
//  1) ПОТЕРЯ СТРОК ЛОГА. В STOP хвост TX-буфера замерзает вместе
//     с UART: невытолкнутые байты теряются, соседние строки
//     склеиваются в одну. safeSerialFlush() ниже вытолкнет хвост
//     в провод ПЕРЕД каждым засыпанием.
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
//  Не static: функция вызывается из GLM.ino (loop(): основной сон
//  и waitForConversion; объявлена в config.h).
// ============================================================
void serialOffForStop() {
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
//  тактирование шин продолжается, поэтому SPI/USART/TIM/DMA/CRC
//  продолжают потреблять, если их явно не выключить (в STOP шины
//  останавливаются аппаратно целиком — там это профилактика).
//
//  Список "занятых" пинов на нашей плате F401RBT6 (LQFP64):
//    PA11, PA12 — USB DM/DP   (трогать нельзя — теряем USB-CDC;
//                 исключения: USB мёртв — при NO_LOG (ветка 1) и
//                 в USART1-режиме (ветка 2) — тогда в Analog)
//    PA13, PA14 — SWDIO/SWCLK (при SWD_ENABLE=1 не трогаем — отладка/
//                 прошивка; при SWD_ENABLE=0 паркуем в Analog — секция 1b)
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
void disableUnusedPinsAndPeripherals() {
  // --- 1. Неиспользуемые пины порт A: в режим Analog ---
  // PA15 (кнопка) — не трогаем всегда. PA13/PA14 (SWD) — только при
  // SWD_ENABLE=1; при 0 они добавляются к списку в секции 1b ниже.
  // Раскладка остальных — по веткам ниже: PA11/PA12 (USB DM/DP) в Analog,
  // когда USB мёртв (NO_LOG или USART1-режим); PA9/PA10 — в Analog,
  // кроме USART1-режима.
  // При LOG_VIA_USART1=1 пины PA9/PA10 заняты аппаратным USART1 (порт
  // логирования) — переводить их в Analog НЕЛЬЗЯ: эта функция
  // вызывается в setup() ПОСЛЕ LOG_BEGIN и молча переключила бы
  // UART-пины в Analog (симптом: лог печатается до строки
  // «[Thermo] entering SLEEP mode» и замолкает навсегда).
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = 0;
  // ============================================================
  // Порт A паркуется ОДНИМ вызовом HAL_GPIO_Init, раскладка пинов
  // — по трём веткам ниже.
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
  // PA15 (кнопка) и PA13/PA14 при SWD_ENABLE=1 — паркуем PA0-PA12.
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
  // USART1 — не трогаем (см. комментарий выше). PA11/PA12 (USB DM/DP)
  // мертвы так же, как при NO_LOG: USB-стек не запускается
  // (Serial.begin() не вызывается, LOG_BEGIN поднимает только USART1,
  // prepareStopMode() GPIO не трогает) — переопределять пины некому.
  // Паркуем в Analog: плавающие D+/D- иначе держат входные буферы
  // Шмитта в осцилляции (AN4899).
  GPIO_InitStruct.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
                        GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  |
                        GPIO_PIN_8  | GPIO_PIN_11 | GPIO_PIN_12;
#else
  // USB-CDC режим: лог через Serial, устройство уже перечислено.
  // PA11/PA12 (DM/DP, AF10 OTG FS) — НЕ ТРОГАЕМ. PA9/PA10 свободны
  // (USART1 не используется) — в Analog.
  GPIO_InitStruct.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
                        GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  |
                        GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10;
#endif

  // --- 1b. SWD-пины PA13/PA14 (SWDIO/SWCLK) ---
  // По умолчанию (SWD_ENABLE=1) не трогаются: доступны отладка и
  // прошивка по ST-Link. При SWD_ENABLE=0 уходят в Analog вместе с
  // прочими неиспользуемыми: плавающие SWD-штырьки держат входные
  // буферы Шмитта в осцилляции (AN4899); в Sleep-режиме это прямая
  // потеря, в STOP — та же «полка» тока при подключённых проводах.
  // Прошивка при SWD_ENABLE=0 — DFU-загрузчик (BOOT0=1 + RESET:
  // системный бутлоадер сам переконфигурирует пины) или «connect
  // under reset» в бут-окне до этапа «Pins» (см. config.h).
#if !SWD_ENABLE
  GPIO_InitStruct.Pin |= GPIO_PIN_13 | GPIO_PIN_14;  // SWDIO/SWCLK -> Analog
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
  // В Sleep-режиме тактирование шин продолжается — всё неиспользуемое
  // гасим явно (в STOP AHB/APB останавливаются аппаратно целиком, здесь
  // это профилактика, но код обязан быть корректным и для Sleep).
  // Перечислено только то, что есть на F401RBT6 и не используется в скетче.
  // USART2 — может использоваться STM32duino для Serial2. Если не нужен —
  // отключаем. USART1 оставляем (порт логирования в USART1-режиме).
  __HAL_RCC_USART2_CLK_DISABLE();
  // SPI1/2/3 — на F401RBT6 доступны SPI1 и SPI2. SPI3 отсутствует на F401.
  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_SPI2_CLK_DISABLE();
  // I2C2 — не используется. I2C1 нужен для OLED. I2C3 отсутствует на F401.
  __HAL_RCC_I2C2_CLK_DISABLE();
  // DMA1/DMA2 (AHB1) — не используются: I2C, UART и АЦП работают
  // в блокирующем режиме. Если понадобится DMA, HAL включит клок
  // на этапе инициализации канала.
  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_DMA2_CLK_DISABLE();
  // CRC-юнит (AHB1) — скетчем не используется.
  __HAL_RCC_CRC_CLK_DISABLE();
  // Таймеры. На F401RBT6 их 8: TIM1, TIM2-TIM5, TIM9-TIM11
  // (TIM6/TIM7/TIM12-14 отсутствуют). Ни один не нужен: delay()/
  // millis()/micros() держатся на SysTick (счётчик ядра Cortex-M4,
  // вне периферийных шин), время — RTC Wakeup Timer. Если в будущем
  // понадобится таймерная периферия (tone/analogWrite/Servo —
  // HardwareTimer), ядро включит клок нужного таймера само при
  // создании объекта.
  // TIM1, TIM9-TIM11 (APB2):
  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_TIM9_CLK_DISABLE();
  __HAL_RCC_TIM10_CLK_DISABLE();
  __HAL_RCC_TIM11_CLK_DISABLE();
  // TIM2-TIM5 (APB1):
  __HAL_RCC_TIM2_CLK_DISABLE();
  __HAL_RCC_TIM3_CLK_DISABLE();
  __HAL_RCC_TIM4_CLK_DISABLE();
  __HAL_RCC_TIM5_CLK_DISABLE();
  // CAN1 — отсутствует на F401 (есть на F407/F429).
  // SDIO — отсутствует на F401.

  LOG_OBJECT.println(F("[Thermo] unused pins & peripherals disabled"));
}

// ============================================================
//  Примечание о снижении тактовой частоты.
//  SYSCLK не понижается (84 МГц в обоих режимах): в Arduino-среде
//  смена частоты ломает USB-CDC (синхронизация), I2C и 1-Wire
//  тайминги и EXTI-конфигурацию кнопки. Если понадобится экономия —
//  лучше полноценный STOP-режим с ручной настройкой RTC Wakeup
//  Timer. История попытки — в CHANGELOG.md.
// ============================================================

// ============================================================
//  Подготовка к STOP-режиму — оптимизация по AN4899.
//  Вызывается перед КАЖДЫМ входом в STOP при USE_STOP_MODE = 1:
//  из enterSleep() (основной 15-с сон) и из waitForConversion()
//  GLM.ino (сон на время конверсии DS18B20).
// ============================================================
void prepareStopMode() {
  // МИНИМАЛЬНАЯ ПОДГОТОВКА — только то, что безопасно и не ломает
  // пробуждение. Расширенные варианты (отключение периферии и
  // прерываний) НЕ возвращать без проверки на железе: они нарушают
  // вход в STOP и пробуждение (см. CHANGELOG.md).

  // 1. Очищаем флаги пробуждения — чтобы не сработали мгновенно после входа.
  EXTI->PR = 0xFFFFFFFF;                  // latched-флаги всех линий (RTC EXTI17 + кнопка)
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
  NVIC_ClearPendingIRQ(BUTTON_EXTI_IRQN); // IRQ EXTI-линии кнопки (config.h)

  // 2. Конфигурируем PWR для STOP с LP-регулятором (LPDS=1).
  // Это единственная настройка, которая реально снижает потребление
  // и не мешает пробуждению. FPDS и отключение периферии/USB здесь
  // НЕ применяются: они нарушают пробуждение или не дают эффекта
  // (см. CHANGELOG.md).
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
// Восстановление SysTick после ЛЮБОГО deepSleep() (STOP).
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
  //    Ветка выбирается той же условной компиляцией, что и сам порт
  //    логирования: в конфигурациях NO_LOG/USART1 USB-CDC не
  //    пересоздаётся — безусловный end+begin+delay(500) добавлял бы
  //    ~560 мс полного тактирования 84 МГц к каждому пробуждению
  //    (≈ +1.2 мА к среднему току цикла при батарейном питании).
#if defined(NO_LOG)
  // NO_LOG: порт логирования не существует — восстанавливать нечего.
#elif USE_STOP_MODE && LOG_VIA_USART1
  // USART1: перед каждым STOP порт деинициализируется serialOffForStop()
  // (PA9/PA10 уходят в Analog), поэтому на пробуждении достаточно поднять
  // его заново: LOG_BEGIN делает setTx/setRx/begin — пины возвращаются
  // в AF, UART готов сразу (бутстреп-время ничтожно, задержек не нужно).
  LOG_BEGIN(115200);
#else
  // Реинициализация USB-CDC после STOP.
  //    USB-периферия в STOP выключается, после пробуждения нужно
  //    пересоздать CDC-стек. Serial.end() + Serial.begin() делает
  //    переэнумерацию на стороне хоста.
  Serial.end();
  delay(50);
  Serial.begin(115200);
  delay(500);  // дать хосту время на переэнумерацию
  // Если хост так и не вернулся (батарейный режим без ПК) — гасим
  // CDC-стек снова. У незапущенного порта LOG_OBJECT.print(...) —
  // no-op; иначе в ряде версий ядра STM32duino печать при непустой
  // очереди TX (хост не читает) может блокироваться навечно. Если
  // хост появится позже — USB будет поднят заново на следующем
  // пробуждении (этот же блок).
  if (!Serial) {
    Serial.end();
  }
#endif

  // 3. Перерегистрируем прерывание пробуждения —
  //    в STOP флаги EXTI могут быть сброшены.
  LowPower.attachInterruptWakeup(PIN_BUTTON, buttonWakeCallback, FALLING);

  // 4. Маленькая пауза стабилизации.
  delay(10);
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
// (Команды OLED_CMD_CHARGEPUMP / _ON / _OFF определены в config.h.)

void oledPowerOff() {
  // Полное засыпание OLED: панель OFF + charge pump OFF.
  display.ssd1306_command(SSD1306_DISPLAYOFF);    // 0xAE
  display.ssd1306_command(OLED_CMD_CHARGEPUMP);     // 0x8D
  display.ssd1306_command(OLED_CMD_CHARGEPUMP_OFF);// 0x10
}

void oledPowerOn() {
  // Пробуждение OLED: charge pump ON + панель ON.
  display.ssd1306_command(OLED_CMD_CHARGEPUMP);     // 0x8D
  display.ssd1306_command(OLED_CMD_CHARGEPUMP_ON);  // 0x14
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
//      VDDA = 3.3 В * VREFINT_CAL / ADC(VREFINT)
//
//  VREFINT_CAL — 12-битный код, откалиброванный при VDDA = 3.3 В
//  (согласовано с VDDA_CAL_MV = 3300.0f в config.h), лежит по адресу
//  0x1FFF7A2A (DS9716 «Calibration data»). Каналы
//  ADC F401: IN16 = датчик T, IN17 = VREFINT, IN18 = VBAT.
//
//  ВНИМАНИЕ к точности: спецификация VREFINT дана при VDDA 2.4–3.6 В,
//  а у нас 4.0–4.2 В (вся плата вне abs max — осознанное решение).
//  Показание годится как индикатор батареи, не как вольтметр.
//
//  Энергогигиена: функция самодостаточна —
//  поднимает клок ADC1 + TSVREFE на время замера (~100 мкс) и ГАСИТ их
//  в конце, восстанавливая ровно то состояние, которое задаёт
//  disableUnusedPinsAndPeripherals() (секция 4). Вызывается
//  только из drawScreen() в активной фазе — в STOP АЦП обесточен.
// ============================================================
// (Константы VREFINT_CAL_ADDR_F401, VDDA_CAL_MV, ADC_CCR_ADCPRE_DIV4
//  и флаг диагностики VDD_DEBUG определены в config.h.)

//  Сырые коды последнего замера; пишутся при каждом вызове.
uint16_t g_vddRaw = 0;
uint16_t g_vddCal = 0;

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
  ADC1->SMPR2 = (7U << 21);        // SMP17 = 111 → 480 циклов
  ADC1->SQR1  = 0;                 // L = 0 → 1 преобразование
  ADC1->SQR3  = 17U;               // SQ1 = IN17 (VREFINT)

  // 3. Стабилизация и холостая конверсия. RM0368: первая конверсия после
  //    подачи ADON может стартовать не раньше t_STAB (3 мкс), а VREFINT
  //    после TSVREFE=1 ещё и набирает номинал десятки мкс — без паузы и
  //    холостой конверсии показание недостоверно (см. CHANGELOG.md).
  //    delayMicroseconds — API ядра STM32duino.
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
  uint32_t cal = *reinterpret_cast<const uint16_t *>(VREFINT_CAL_ADDR_F401); // заводской код при 3.3 В
  g_vddCal = (uint16_t)cal;
  return (uint16_t)(VDDA_CAL_MV * cal / raw + 0.5f);
}
