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
 *              Будит MCU из SLEEP. В ACTIVE: короткое нажатие
 *              продлевает активную минуту и переключает экраны
 *              (стандартный ↔ график журнала); длинное
 *              удержание (≥ JOURNAL_DUMP_HOLD_MS) выгружает
 *              журнал в CSV по UART (при JUR_2_UART = 1).
 *              (PA15 на STM32F4 по умолчанию занят JTAG/SWD — см. ниже
 *               освобождение пина в setup().)
 *   - Кнопка сброса: Внешний pull-up к 3V3 (пин в покое = HIGH).
 *              При нажатии замыкается на GND (пин = LOW).
 *              Один контакт -> PC11, второй -> GND.
 *              Работает ТОЛЬКО в ACTIVE. Сбрасывает min/max к текущему T.
 *   - Питание: Li-ion аккумулятор 3.7 В (заряженный 4.0-4.2 В) —
 *              НАПРЯМУЮ на шину питания (пин 3V3), МИМО встроенного
 *              LDO платы: LDO (LM1117) сам тянет ~3-3.5 мА утечки
 *              с вывода Vout — для батарейного режима недопустимо.
 *              ВНИМАНИЕ: VDD = VDDA = напряжение батареи — вне
 *              спецификации MCU (осознанное решение, см. power.cpp,
 *              readVddMillivolts()).
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
 *   • ДИАГНОСТИКА ЗАГРУЗКИ: от первого включения OLED до
 *     ухода в сон на экране виден экран загрузки — строка
 *     «boot N/6: этап» и полоса прогресса, обновляемые после
 *     каждого этапа init (OLED, DS18B20, Journal, RTC, Wake,
 *     Pins). Если прошивка зависает при старте — экран замирает
 *     на ПОСЛЕДНЕМ ПРОЙДЕННОМ этапе: по нему сбой локализуется
 *     без отладчика. В заголовке напечатана версия комплекта —
 *     видно, какая прошивка реально на плате.
 *   • Если USB-хост не подключён (батарейный режим), CDC-стек
 *     закрывается после USB_ENUM_WAIT_MS (Serial.end()): печать
 *     лога становится no-op — без этого первая же печать при
 *     непустой очереди TX (хост не читает) в ряде версий ядра
 *     блокируется навечно, и бут замирает до первого initOLED.
 *   • SLEEP: каждые 15 с контроллер просыпается по RTC-таймеру,
 *     измеряет температуру, обновляет диапазон min/max и снова
 *     уходит в глубокий сон. На дисплей ничего не выводится.
 *   • Нажатие кнопки будит MCU по спаду фронта на PA15.
 *     Контроллер переходит в ACTIVE на 1 минуту, дисплей ВКЛЮЧАЕТСЯ.
 *     В ACTIVE каждые 5 с экран обновляется:
 *        Строка 1 (крупный шрифт): живой СЫРОЙ отсчёт tempRawC —
 *          всегда; табло интерактивно реагирует на любые изменения.
 *          Положительная — без знака, отрицательная — со знаком "−".
 *        Строка 2 (мелкий шрифт): диапазон "min..max" за всё время.
 *   • По истечении минуты дисплей выключается и MCU снова уходит в SLEEP.
 *   • Нажатие кнопки в ACTIVE — продлевает активную минуту.
 *   • Короткое нажатие кнопки в ACTIVE переключает экраны:
 *     стандартный ↔ график журнала (до 128 последних образцов,
 *     15 мин/точка = 32 ч, автомасштаб min..max). После 1 минуты
 *     бездействия на графике устройство показывает стандартный
 *     кадр и уходит в сон (таймаут — та же минута активности).
 *   • Длинное нажатие (≥ 2 с) кнопки в ACTIVE выгружает журнал
 *     по UART дампом в CSV «время,значение» (только при
 *     JUR_2_UART = 1): время — RTC-эпоха [с] начала 15-минутного
 *     окна образца, значение — температура [°C]; заголовок —
 *     строки с '#'. На время дампа экран показывает «CSV dump...»
 *     и не обновляется (полный журнал при 115200 бод — ~20 с).
 *     Дамп сам поднимает порт логирования (ensureLogPortForDump):
 *     если хост не обнаружен — Serial.end + begin + ожидание до
 *     USB_ENUM_WAIT_MS с обратным отсчётом на экране; «no host!»
 *     и отказ, если хост так и не появился; пропажа хоста посреди
 *     дампа прерывает выгрузку («host lost!»), а не блокирует
 *     печать навечно.
 *   • Температура читается и хранится как int8 в формате °C×2
 *     (шаг 0.5 °C, датчик — в 9-битном режиме); рабочий диапазон
 *     уличного логгера: -50..+50 °C, выход за него — ошибка замера.
 *   • Все измерения проходят медианный фильтр из 3 отсчётов:
 *     запрос датчика возвращает «ранее рассчитанную медиану»
 *     (после перезахвата ворота — признанный уровень), пересчёт —
 *     на каждом третьем принятом отсчёте (в SLEEP — раз в 45 с).
 *     Ошибочные и отброшенные отсчёты фильтр не сдвигают.
 *   • Защита от «солнечных вспышек» на датчике: сырой отсчёт,
 *     отличающийся от последнего принятого больше чем на 1 °C
 *     (TEMP_SLEW_MAX_C), отбрасывается — показания замораживаются
 *     на последнем достоверном значении. Уровень, стабильный
 *     дольше 10 минут (TEMP_RELOCK_S, по RTC), признаётся реальным
 *     изменением (в терминал — событие RELOCK). Новые min/max
 *     требуют подтверждения 4 замерами подряд.
 *   • При включённом терминале (LOG_OBJECT) в лог идут сырые
 *     отсчёты и решения ворота: «[Thermo] raw T=... gate=OK/REJ».
 *   • Крупная строка экрана ВСЕГДА показывает ЖИВОЙ СЫРОЙ отсчёт:
 *     табло интерактивно реагирует на любые изменения среды — видно,
 *     что термометр работает. Пока ворот отбрасывает отсчёты,
 *     рядом выводится инвертированный бейдж «REJ» (информирование
 *     об отбрасывании), внизу — строка «hold X.X C» с замороженным
 *     достоверным значением.
 *
 * Структура проекта (мультифайловая версия)
 * ------------------------------------------
 *   config.h    — все константы (constexpr; #define — только для
 *                 препроцессора: версия комплекта, флаги веток #if,
 *                 макросы-подстановки LOG_OBJECT/LOG_BEGIN)
 *                 с комментариями, подсистема
 *                 логирования (LOG_OBJECT/LOG_BEGIN), общий тип
 *                 WakeSource, extern-объявления и прототипы функций
 *                 модулей. Подключается каждым модулем.
 *   display.cpp — вывод информации на OLED:
 *                 initOLED(), printTempSigned(), drawScreen()
 *                 (крупная строка — всегда живой сырой отсчёт;
 *                 в REJ-режиме — бейдж REJ + строка «hold»),
 *                 drawGraph() — экран графика журнала.
 *   power.cpp   — управление энергией MCU и дисплея:
 *                 STOP-режим (prepareStopMode/wakeFromStop/
 *                 resumeSysTick), парковка пинов и отключение
 *                 периферии, питание SSD1306 (oledPowerOn/Off),
 *                 измерение VDD батареи, флаги и callbacks
 *                 пробуждения, detectWakeSource().
 *   termo.cpp   — накопление и обработка температуры DS18B20:
 *                 конвейер TemperaturePipeline (ворот скорости
 *                 + медиана из 3 — состояние и логика в одной
 *                 структуре), initThermo() (настройка датчика
 *                 + посев), readTemperature(), updateStats(),
 *                 resetStats(), readAndLogSleepTemp(); состояние
 *                 конвейера для экрана — tempRawC / tempRejected
 *                 (флаг REJ).
 *   journal.cpp — журнал температуры в ОЗУ (репетиция флеш-версии):
 *                 journalInit(), journalOnSample(); отбор
 *                 подтверждённых замеров по 15-минутным окнам RTC,
 *                 упаковка 4×int8 в 32-битное слово, битовая
 *                 карта занятости (1 = слово свободно); при
 *                 JUR_2_UART = 1 — карта пропусков окон для
 *                 точного времени и journalDumpCsv() — дамп CSV
 *                 «время,значение».
 *   GLM.ino     — ОСНОВА: конечный автомат SLEEP/ACTIVE, опрос
 *                 кнопок (isButtonPressed/waitForButtonRelease —
 *                 с измерением длительности нажатия), выбор
 *                 экрана ACTIVE (стандартный/график) и вызов
 *                 дампа журнала с подъёмом порта логирования
 *                 (ensureLogPortForDump), переходы
 *                 enterSleep()/enterActive(), ожидание конверсии
 *                 DS18B20 во сне (waitForConversion), экран
 *                 загрузки с отметками этапов
 *                 (bootFrameStart/bootStep), setup(), loop().
 * ============================================================ */

#include "config.h"

// Контроль синхронности версий файлов: все 6 файлов скетча должны
// быть из ОДНОГО архива (см. GLM_CONFIG_VERSION в config.h).
#if !defined(GLM_CONFIG_VERSION) || GLM_CONFIG_VERSION < 20
#error "config.h устарел (нужна v20): замените ВСЕ 6 файлов скетча из актуального архива"
#endif

// ============================================================
//  Определение объекта логирования (объявлен extern в config.h).
//  Это ЕДИНСТВЕННОЕ место определения — объект один на всю
//  программу.
// ============================================================
#if defined(NO_LOG)
  // NO_LOG: «немой» логгер — все LOG_OBJECT.* превращаются в no-op.
  NoLogClass NoLogInstance;
#elif USE_STOP_MODE && LOG_VIA_USART1
  // Serial1 не создаётся ядром для нашей платы — создаём объект
  // сами (подробности — в config.h и CHANGELOG.md).
  Uart Serial1(USART1);
#endif

// ============================================================
//  Состояния конечного автомата
//  (используются только внутри GLM.ino, поэтому тип держим здесь)
// ============================================================
enum SystemState : uint8_t {
  ST_SLEEP  = 0,
  ST_ACTIVE = 1
};

SystemState state         = ST_SLEEP;
uint32_t    activeStartMs = 0;   // момент входа в ACTIVE
uint32_t    lastRefreshMs = 0;   // момент последнего обновления экрана

// Текущий экран ACTIVE. Пробуждение всегда начинается со
// стандартного экрана; короткие нажатия кнопки переключают режимы.
enum ScreenMode : uint8_t {
  SCR_MAIN  = 0,   // стандартный экран: живой отсчёт + min/max + батарея
  SCR_GRAPH = 1    // график журнала: до GRAPH_MAX_PTS последних образцов
};
ScreenMode screenMode = SCR_MAIN;

// Источник последнего пробуждения (тип WakeSource — в config.h;
// значение вычисляется detectWakeSource() из power.cpp).
WakeSource lastWakeSource = WAKE_UNKNOWN;

// Прототипы локальных функций GLM.ino (реализации — ниже).
bool     isButtonPressed();
uint32_t waitForButtonRelease();  // возвращает длительность удержания, мс
void     enterSleep();
void     enterActive();
#if JUR_2_UART
void     drawDumpFrame();         // кадр «идёт выгрузка CSV»
void     dumpFrameStatus(const __FlashStringHelper *msg); // строка статуса в кадре дампа
bool     ensureLogPortForDump();  // подъём порта логирования перед дампом
#endif

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
// Возвращает длительность удержания, мс — от вызова (сразу после
// распознавания нажатия) до отпускания. В SLEEP-пути (пробуждение
// кнопкой) значение не используется; в ACTIVE по нему выбирается
// жест: короткое нажатие (переключение экранов) или длинное
// ≥ JOURNAL_DUMP_HOLD_MS (дамп CSV журнала).
//
// После основного цикла ожидания — «окно тишины»: контакт кнопки
// дребезгит на размыкании (серия LOW->HIGH->LOW переходов), каждый
// спад = FALLING-фронт на EXTI-линии кнопки — ISR срабатывает уже
// ПОСЛЕ того, как detectWakeSource() потребил и очистил флаг
// пробуждения, и ставит wokenByButton=true «впрок». Без окна
// тишины флаг пережил бы активную фазу и ложно классифицировал
// следующее (RTC-)пробуждение как кнопочное. Поэтому: окно тишины
// дожидается конца дребезга, затем следы дребезга глотаются —
// сначала latched EXTI->PR, затем pending NVIC, и только потом
// volatile-флаг — под маской прерываний, чтобы ISR не вписал его
// заново между очистками (линия и IRQ кнопки — BUTTON_EXTI_LINE /
// BUTTON_EXTI_IRQN из config.h, выведены из PIN_BUTTON).
uint32_t waitForButtonRelease() {
  // pressStart — точка отсчёта длительности удержания
  uint32_t pressStart = millis();
  uint32_t start = millis();
  while (digitalRead(PIN_BUTTON) == LOW) {
    if (millis() - start > BUTTON_PRESS_TIMEOUT_MS) {
      LOG_OBJECT.println(F("[Thermo] button release timeout"));
      break;
    }
    delay(10);
  }
  // Длительность удержания снята СРАЗУ после отпускания — до
  // «окна тишины» (антидребезг), чтобы не завышать жест.
  uint32_t holdMs = millis() - pressStart;

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

  // Проглотить следы дребезга (см. комментарий к функции выше).
  EXTI->PR = (1u << BUTTON_EXTI_LINE);      // latched-флаг линии кнопки
  NVIC_ClearPendingIRQ(BUTTON_EXTI_IRQN);   // pending в NVIC
  noInterrupts();
  wokenByButton = false;
  interrupts();
  return holdMs;
}

// ============================================================
//  Переходы между состояниями
// ============================================================
void enterSleep() {
  state = ST_SLEEP;
  // clearDisplay()+display() перед засыпанием НЕ нужны: drawScreen()
  // в ACTIVE каждый кадр начинается с clearDisplay() и полностью
  // перерисовывает экран, так что первое же обновление (оно идёт
  // немедленно, lastRefreshMs = 0) затирает прошлое состояние.
  // Побочный эффект: первые ~200 мс после пробуждения (разгон
  // charge pump + первый блокирующий замер) видно
  // «замороженное» прошлое состояние — естественное поведение
  // просыпающегося устройства.
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
  //   - В Sleep-режиме: порт логирования активен, flush вытолкнет
  //     все байты.
  //   - В STOP-режиме: порт сейчас работает (оживлён wakeFromStop);
  //     flush вытолкнет байты ПЕРЕД входом в STOP — после входа
  //     тактирование останавливается и хвост TX-буфера теряется
  //     (подробности — serialOffForStop в power.cpp).
  //
  // НЕ безопасна только «сырая» Serial.flush() без таймаута — она
  // может блокировать бесконечно, если USB-CDC нерабочая.
  // safeSerialFlush ждёт не более 200 мс и возвращает управление.
  safeSerialFlush(200);

  // Если включён STOP-режим — дополнительная подготовка.
  if (USE_STOP_MODE) {
    prepareStopMode();
  }

  // Гигиена флагов пробуждения перед сном: prepareStopMode() выше
  // уже сбросил EXTI->PR и NVIC-pending, здесь под маской прерываний
  // гасятся сами volatile-флаги — классифицируются только фронты,
  // случившиеся ВО ВРЕМЯ сна (дребезг кнопки в активной фазе больше
  // не портит следующую классификацию — см. waitForButtonRelease).
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
  screenMode    = SCR_MAIN;   // пробуждение — всегда стандартный экран
  oledPowerOn();
  if (LED_PIN >= 0) digitalWrite(LED_PIN, LED_ON);
  // Явная первая отрисовка — иначе на экране может остаться
  // мусор от предыдущего состояния или просто чёрный кадр,
  // так как SSD1306 не всегда корректно восстанавливает GDDRAM
  // после команды DISPLAYON.
  if (readTemperature(currentTempC)) {
    updateStats(currentTempC);
  }
  drawScreen();
}

// ============================================================
//  Ожидание конверсии DS18B20 и раздача результата SLEEP-цикла.
//  Вызывается из loop() после блика LED, requestTemperatures() и
//  отпускания 1-Wire шины (пробуждение RTC/UNKNOWN).
//  Возвращает true  — конверсия досижена, замер можно забирать
//                     (readAndLogSleepTemp());
//              false — кнопка разбудила MCU досрочно, замер этого
//                     цикла принесён в жертву, цикл уже ушёл в
//                     ACTIVE (enterActive).
//
//  Выбор способа ожидания. USB-CDC НЕ переживает STOP: USB-периферия
//  гаснет, а wakeFromStop() после конверсионного сна НЕ вызывается —
//  Serial остался бы мёртвым до следующего основного пробуждения,
//  строки лога уходили бы «в никуда». Поэтому второй STOP делаем
//  ТОЛЬКО там, где порт логирования его переживает:
//    * NO_LOG           — лог выключен, терять нечего;
//    * LOG_VIA_USART1=1 — UART поднимается заново через LOG_BEGIN.
//  В USB-CDC (LOG_VIA_USART1=0 и без NO_LOG) и при USE_STOP_MODE=0
//  конверсия высиживается обычным delay(): USB остаётся живым, лог
//  печатается штатно. Энергопотери для отладочного профиля нет:
//  USB-CDC изначально подразумевает подключённый ПК и питание от
//  него, а не батарейный режим.
// ============================================================
static bool waitForConversion() {
#if USE_STOP_MODE && (defined(NO_LOG) || LOG_VIA_USART1)
  // --- STOP-путь: конверсия высиживается во сне ---
  // Окно сна = DS18B20_CONV_MS + CONV_GUARD_MS - LED_BLINK_MS
  // (сейчас 94 + 100 - 50 = 144 мс). Кнопка может разбудить досрочно —
  // конверсия прервана, чтение скретчпада дало бы прошлоцикловое
  // значение (ACTIVE всё равно замерит T через мгновение); нажатие
  // при этом обрабатывается немедленно и не теряется. UART гасим
  // ДО сна (serialOffForStop: flush уже пуст — «wake: RTC…» ушла
  // в провод ещё до блика), поднимаем ПОСЛЕ — симметрично
  // основному сну.
  serialOffForStop();
  prepareStopMode();
  LowPower.deepSleep(DS18B20_CONV_MS + CONV_GUARD_MS - LED_BLINK_MS);
  resumeSysTick();
  // UART был остановлен перед конверсионным сном — поднимаем заново:
  // при LOG_VIA_USART1 LOG_BEGIN выполнит setTx/setRx/begin (пины
  // PA9/PA10 вернутся в AF), при NO_LOG LOG_BEGIN — no-op ((void)0).
  // Отдельный #if не нужен: в конфигурации USB-CDC внешняя ветка
  // #if отсекает весь STOP-путь.
  LOG_BEGIN(115200);
  if (wokenByButton) {
    // Нажатие в окне бодрствования (между пробудкой и конверсионным
    // сном) или во время самого конверсионного сна.
    LOG_OBJECT.println(F("[Thermo] wake: BUTTON (in wake window)"));
    LOG_OBJECT.flush();
    waitForButtonRelease();
    enterActive();
    return false;
  }
#else
  // --- USB-CDC (или USE_STOP_MODE=0): блокирующее ожидание ---
  // Второй STOP не делаем — он убил бы USB (см. шапку функции).
  // Нажатия кнопки в этом окне немедленно не обрабатываются:
  // выставленный ISR-флаг wokenByButton будет погашен в
  // enterSleep() перед следующим сном.
  delay(DS18B20_CONV_MS + CONV_GUARD_MS - LED_BLINK_MS);
#endif
  return true;
}

#if JUR_2_UART
// ============================================================
//  Кадр «идёт выгрузка CSV». Показывается на всё время
//  блокирующего journalDumpCsv(): пользователь видит, почему
//  экран не обновляется. Одноразовый служебный кадр — как и
//  стартовый сплэш, рисуется прямо здесь, а не в display.cpp.
// ============================================================
void drawDumpFrame() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 24);
  display.print(F("CSV dump to UART..."));
  dumpFrameStatus(F("see terminal"));
}

// Строка статуса в кадре дампа: нижняя строка y40,
// перерисовывается с заливкой фона. Через неё дамп общается
// с пользователем: «host? 3s» — ожидание хоста, «sending...»,
// «no host!» / «log off» — отказ, «host lost!» — обрыв.
void dumpFrameStatus(const __FlashStringHelper *msg) {
  display.fillRect(0, 40, OLED_W, 8, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 40);
  display.print(msg);
  display.display();
}

// ============================================================
//  Подъём порта логирования перед дампом.
//  После STOP порт логирования может быть закрыт: CDC-стек без
//  хоста закрывается (печать — no-op), а окно ожидания хоста в
//  wakeFromStop — всего 500 мс: USB-хост (особенно Windows) за
//  это время не переэнумеруется. Здесь порт приводится в
//  рабочее состояние ЦЕЛЕНАПРАВЛЕННО:
//    NO_LOG    — печати no-op: «log off», дамп не запускается;
//    USART1    — LOG_BEGIN (setTx/setRx/begin): хост не нужен;
//    USB-CDC   — если хост уже есть (Serial == true), ничего
//                не делаем; иначе end + begin + ожидание до
//                USB_ENUM_WAIT_MS с обратным отсчётом на экране
//                (пользователю даётся время открыть монитор).
//                Хост не появился — Serial.end() (гарантия
//                анти-зависания сохраняется), «no host!», отказ.
//  Возвращает true, если печатать можно.
// ============================================================
bool ensureLogPortForDump() {
#if defined(NO_LOG)
  dumpFrameStatus(F("log off"));
  return false;
#elif USE_STOP_MODE && LOG_VIA_USART1
  LOG_BEGIN(115200);              // хост не нужен, переоткрытие безвредно
  return true;
#else
  if (Serial) return true;        // хост уже подключён — печатаем сразу
  Serial.end();
  delay(50);
  Serial.begin(115200);
  const uint32_t t0 = millis();
  uint8_t lastSec = 0xFF;
  while (!Serial && (millis() - t0) < USB_ENUM_WAIT_MS) {
    // Обратный отсчёт на экране: «host? 3s/2s/1s» — пользователь
    // видит, что устройство ждёт открытия монитора.
    uint8_t secLeft = (uint8_t)((USB_ENUM_WAIT_MS - (millis() - t0) + 999UL) / 1000UL);
    if (secLeft != lastSec) {
      lastSec = secLeft;
      dumpFrameStatus(secLeft >= 3 ? F("host? 3s")
                    : (secLeft == 2 ? F("host? 2s") : F("host? 1s")));
    }
  }
  if (!Serial) {
    Serial.end();                 // печати у остановленного порта — no-op
    dumpFrameStatus(F("no host!"));
    return false;
  }
  return true;
#endif
}
#endif // JUR_2_UART

// ============================================================
//  Экран загрузки с отметками этапов. bootFrameStart()
//  печатает заголовок с версией комплекта; каждый пройденный
//  этап init вызывает bootStep(N, "имя") — строка «boot N/6»
//  и полоса прогресса перерисовываются немедленно. Если прошивка
//  зависает при старте, экран замирает на ПОСЛЕДНЕМ ПРОЙДЕННОМ
//  этапе: точка сбоя читается с экрана без отладчика.
//  Раскладка (128x64): y0 — заголовок, y10 — строка этапа,
//  y30 — полоса прогресса, y40 — температура посева, y52 —
//  подсказки жестов. Отметки рисуются только после успешного
//  initOLED (флаг bootOledOk).
// ============================================================
static bool bootOledOk = false;   // экран прошёл initOLED() — можно рисовать

static void bootFrameStart() {
  if (!bootOledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Thermo v"));
  display.print(GLM_CONFIG_VERSION);
  display.print(F(" boot"));
  display.display();
}

// done — номер ПРОЙДЕННОГО этапа (1..6), name — его короткое имя.
// Строка этапа и полоса перерисовываются с заливкой фона — без
// clearDisplay() (он не нужен: каждое поле затирается своим rect).
static void bootStep(uint8_t done, const char *name) {
  if (!bootOledOk) return;
  display.fillRect(0, 10, OLED_W, 10, SSD1306_BLACK);
  display.fillRect(0, 30, OLED_W, 3, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.print(F("boot "));
  display.print(done);
  display.print(F("/6: "));
  display.print(name);
  display.fillRect(0, 30, (uint16_t)done * (OLED_W / 6), 3, SSD1306_WHITE);
  // Температура посева (y40): 0.0 до конца initThermo, затем —
  // реальный первый замер (currentTempC, формат °C×2)
  display.fillRect(0, 40, OLED_W, 8, SSD1306_BLACK);
  display.setCursor(0, 40);
  display.print(F("T="));
  printTempSigned(currentTempC);
  display.print(F(" C"));
  display.display();
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
  // Блок ожидания компилируется ТОЛЬКО в режиме USB-CDC — при
  // NO_LOG или LOG_VIA_USART1=1 он исключён из прошивки.
#if !defined(NO_LOG) && !(USE_STOP_MODE && LOG_VIA_USART1)
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < USB_ENUM_WAIT_MS)) {
    ;  // ждём подключения USB-CDC (но не более USB_ENUM_WAIT_MS)
  }
  // Если хост так и не подключился (батарейный режим без ПК,
  // «умерший» кабель, питание от зарядки) — закрываем CDC-стек
  // (Serial.end()). У остановленного порта LOG_OBJECT.print(...)
  // — no-op; иначе в ряде версий ядра STM32duino первая же печать
  // при непустой очереди TX блокируется навечно: до initOLED
  // дело не доходит, панель показывает старый кадр. Если хост
  // появится позже — USB вернём на ближайшем пробуждении
  // (wakeFromStop: Serial.begin).
  if (!Serial) {
    Serial.end();
  }
#endif
  delay(100);  // дополнительная пауза для стабилизации
  LOG_OBJECT.println(F("\n[Thermo] boot"));

  // Гасим биты DBGMCU DBG_SLEEP/DBG_STOP/DBG_STANDBY. Если после
  // отладочной сессии эти биты остались взведёнными (они
  // сбрасываются ТОЛЬКО полным обесточиванием!), ядро продолжает
  // тактироваться в STOP — ток сна вырастает с ~42 мкА до единиц мА.
  // Классическая ловушка при замерах энергопотребления (AN4899 /
  // ST Community «Tips for using STM32 low-power modes»). На всякий
  // случай снимаем их программно при каждом старте.
  HAL_DBGMCU_DisableDBGSleepMode();
  HAL_DBGMCU_DisableDBGStopMode();
  HAL_DBGMCU_DisableDBGStandbyMode();

  // Инициализация дисплея (с явной привязкой I2C-пинов).
  // После initOLED панель НЕ гасим до самого enterSleep() —
  // на экране идут отметки этапов загрузки (bootStep): зависание
  // при старте видно на экране. Экономия ~0.5 с выключенной панели
  // не стоит потери диагностики.
  initOLED();
  bootOledOk = true;
  bootFrameStart();
  bootStep(1, "OLED");

  // Датчик температуры: настройка + первичное измерение (посев
  // медианного фильтра). Всё общение с датчиком — в termo.cpp:
  // запуск, 9-битный режим, первое чтение становится начальной
  // «ранее рассчитанной медианой», min/max инициализируются.
  initThermo();
  bootStep(2, "DS18B20");

  // Журнал температуры в ОЗУ (journal.cpp): «стирание» — битовая
  // карта в единицы, слова в 0xFFFFFFFF. Это ОЗУ-репетиция флеш-
  // журналирования: после сброса питания журнал пуст по определению.
  // Подтверждённые замеры подаёт termo.cpp (journalOnSample):
  // отбор — по 15-минутным окнам RTC, 4 образца = слово журнала.
  journalInit();
  bootStep(3, "Journal");

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
  bootStep(4, "RTC");

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
  bootStep(5, "Wake");

  // === Стартовый экран: итог загрузки + подсказки жестов ===
  // Кадр рисуется поэтапно (bootFrameStart/bootStep) начиная
  // с initOLED; здесь добавляем подсказку жестов и выдерживаем
  // паузу, чтобы визуально убедиться, что OLED живёт. Если
  // картинка не появляется вовсе — проблема с инициализацией
  // I2C (пины, адрес, подтяжки), а не с логикой сна.
  display.fillRect(0, 52, OLED_W, 8, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 52);
  display.print(F("hold2s=csv Btn=act"));
  display.display();
  delay(SPLASH_SHOW_MS);

  // === Снижение энергопотребления ===
  // Настраиваем неиспользуемые пины в Analog Input и отключаем
  // тактирование неиспользуемой периферии: USART2, SPI1/2, I2C2,
  // DMA1/DMA2, CRC, TIM1, TIM2-TIM5, TIM9-TIM11, ADC1 (+ TSVREFE/VBATE).
  // Вызов БЕЗУСЛОВНЫЙ: работает и в STOP, и в Sleep-режиме
  // (в Sleep шины тактируются — отключение там обязательно, не
  // профилактика). Полный список с обоснованием — power.cpp, секция 5.
  // Вызываем ПОСЛЕ инициализации всех используемых модулей (OLED,
  // датчик, кнопки, RTC), но ДО первого перехода в сон.
  disableUnusedPinsAndPeripherals();
  bootStep(6, "Pins");

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
    //   • Каждые 5 с — обновление ТЕКУЩЕГО экрана (стандартный
    //     или график — screenMode).
    //   • Короткое нажатие кнопки — переключение экранов;
    //     длинное (≥ JOURNAL_DUMP_HOLD_MS) — CSV-дамп журнала
    //     (при JUR_2_UART = 1). Любое нажатие продлевает
    //     активную минуту и сразу обновляет экран.
    //   • Через 1 минуту без нажатий — уходим в сон (из режима
    //     графика — сначала стандартный кадр).
    uint32_t activeUntil = millis() + ACTIVE_DURATION_MS;
    uint32_t nextRefresh = 0;  // 0 = немедленно

    while (millis() < activeUntil) {
      // Кнопка активности нажата (активный LOW)?
      if (isButtonPressed()) {
        // Продлеваем активную минуту
        activeUntil = millis() + ACTIVE_DURATION_MS;
        nextRefresh = 0;  // форсируем немедленное обновление
        // Длительность удержания выбирает жест
        uint32_t holdMs = waitForButtonRelease();
#if JUR_2_UART
        if (holdMs >= JOURNAL_DUMP_HOLD_MS) {
          // ДЛИННОЕ НАЖАТИЕ — выгрузка журнала по UART.
          // Сначала поднимаем порт логирования (после STOP он
          // почти всегда закрыт — см. ensureLogPortForDump).
          drawDumpFrame();
          if (ensureLogPortForDump()) {
            LOG_OBJECT.println(F("[UI] long press -> journal CSV dump"));
            dumpFrameStatus(F("sending..."));
            if (!journalDumpCsv()) {
              // Хост пропал посреди выгрузки — дамп прерван
              // (конец вывода может не дойти до терминала).
              dumpFrameStatus(F("host lost!"));
            }
          }
          // Дамп — тоже активность пользователя: продлеваем минуту
          // (полный журнал при 115200 бод печатается ~20 с).
          activeUntil = millis() + ACTIVE_DURATION_MS;
          nextRefresh = 0;
        } else
#endif
        {
          // КОРОТКОЕ НАЖАТИЕ — переключение экранов:
          // стандартный ↔ график журнала
          (void)holdMs;   // при JUR_2_UART = 0 длительность не используется
          screenMode = (screenMode == SCR_MAIN) ? SCR_GRAPH : SCR_MAIN;
          LOG_OBJECT.print(F("[UI] screen -> "));
          LOG_OBJECT.println((screenMode == SCR_GRAPH) ? F("GRAPH") : F("MAIN"));
        }
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
          LOG_OBJECT.print(currentTempC * 0.5f, 1);
          LOG_OBJECT.println(F(" C (active)"));
        }
        // Рисуем ТЕКУЩИЙ экран: конвейер замера общий,
        // отличается только отрисовка.
        if (screenMode == SCR_MAIN) {
          drawScreen();
        } else {
          drawGraph();
        }
        nextRefresh = millis() + ACTIVE_REFRESH_MS;
      }

      // Небольшая задержка, чтобы не крутиться вхолостую
      delay(100);
    }

    // Активное окно истекло — уходим в сон.
    if (screenMode == SCR_GRAPH) {
      // После минуты бездействия на графике — стандартный кадр,
      // затем обычный путь в сон (кадр останется в GDDRAM на время
      // сна и первые ~200 мс следующего пробуждения — см. enterSleep).
      LOG_OBJECT.println(F("[UI] graph idle -> main screen"));
      drawScreen();
    }
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
      // UART гасим ДО входа в STOP: хвост TX-буфера, не вытолкнутый
      // в провод, замерзает вместе с UART и теряется — строки
      // склеиваются и пропадают (см. serialOffForStop). flush
      // выталкивает «sleep zZz...» в провод; в режиме USART1
      // PA9/PA10 уходят в Analog — мост не подпитывается
      // из батареи во время сна.
      serialOffForStop();
      LowPower.deepSleep(SLEEP_PERIOD_MS);
      uint32_t t_after = rtc.getEpoch();
      uint32_t slept = (t_after - t_before) * 1000UL;  // секунды → миллисекунды

      lastWakeSource = detectWakeSource();
      wakeFromStop();

      // Источник пробуждения — в лог ПЕРВЫМ делом после оживления
      // UART (свежий LOG_BEGIN — самое надёжное место печати).
      // slept считается по целым секундам RTC (округление вниз):
      // нажатие через ~1 с печатается как 1000.
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
      // если USB-CDC не работает.
      safeSerialFlush();
      LowPower.sleep(SLEEP_PERIOD_MS);
      // В Sleep-режиме флаги EXTI/RTC также устанавливаются, поэтому
      // определение источника работает одинаково для обоих режимов.
      lastWakeSource = detectWakeSource();
    }

    // Пробуждение произошло — по таймеру или по кнопке.
    // Источник уже напечатан выше («wake: RTC/BUTTON/UNKNOWN (slept=…)») —
    // классификация идёт по флагам ISR, уровень пина не читается
    // (кнопка может быть уже отпущена).
    if (lastWakeSource == WAKE_BUTTON) {
      waitForButtonRelease();
      enterActive();
    } else {
      // Сюда попадаем при WAKE_RTC или WAKE_UNKNOWN — замер температуры.

      // Пробуждение по таймеру — короткая вспышка + замер, без вывода
      // на экран. Конверсия DS18B20 (9 бит, максимум 94 мс) идёт при
      // токе полки: MCU спит (способ ожидания — внутри
      // waitForConversion()), бодрствование цикла сжимается до
      // ~60-90 мс. Датчик в 3-проводном включении с VDD не нуждается
      // в мастере во время конверсии — шину удерживает внешняя
      // подтяжка 4.7 кОм.
      if (LED_PIN >= 0) digitalWrite(LED_PIN, LED_ON);
      sensors.requestTemperatures();              // старт конверсии (неблокирующий)
      delay(LED_BLINK_MS);                        // видимый блик = «замер идёт»
      if (LED_PIN >= 0) digitalWrite(LED_PIN, LED_OFF);
      // Гарантируем, что 1-Wire шина ОТПУЩЕНА перед сном:
      // пин DQ в INPUT (floating), линия держится HIGH внешней подтяжкой
      // 4.7 кОм — тока через подтяжку нет. Защита от «залипания» линии
      // в лог.0 после транзакции (3.6 В / 4.7 кОм = 0.77 мА непрерывно).
      pinMode(PIN_DS18B20, INPUT);

      // Высиживаем конверсию (STOP-сон или delay — конфигурационно,
      // см. waitForConversion) и раздаём результат цикла: false =
      // кнопка разбудила досрочно, замер принесён в жертву, цикл
      // уже в ACTIVE.
      if (waitForConversion()) {
        readAndLogSleepTemp();   // «wake: TIMER  T=…  range=[…]»
      }
      // Состояние определит следующий виток loop():
      // ST_SLEEP -> новый сон, ST_ACTIVE -> активная минута (кнопка).
    }
  }
}