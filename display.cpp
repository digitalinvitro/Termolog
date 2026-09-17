/* ============================================================
 *  display.cpp — вывод информации на OLED-дисплей SSD1306
 * ============================================================
 *
 *  Содержит:
 *    - определение объекта display (Adafruit_SSD1306);
 *    - initOLED()        — инициализация I2C и дисплея;
 *    - printTempSigned() — форматирование температуры со знаком
 *      (аргумент в формате хранения °C×2, шаг 0.5 °C — см. config.h);
 *    - drawScreen()      — отрисовка рабочего экрана:
 *        Строка 1 (крупный шрифт): ЖИВОЙ СЫРОЙ отсчёт tempRawC —
 *        ВСЕГДА, при любом состоянии ворота скорости: табло
 *        интерактивно реагирует на любые изменения среды, и
 *        пользователь видит, что термометр работает. Нет флага
 *        REJ — справа единица "C";
 *        Строка 2 (мелкий шрифт):  диапазон "min..max";
 *        правый край строки 2:     напряжение батареи "X.XXV"
 *        (замер выполняется readVddMillivolts() из power.cpp).
 *
 *        REJ-режим — последний отсчёт отброшен воротом скорости
 *        (termo.cpp, tempRejected): вместо единицы "C" выводится
 *        инвертированный бейдж "REJ" (белая плашка, чёрный текст),
 *        внизу — строка «hold X.X C» с замороженным достоверным
 *        значением (currentTempC), которое в это время питает
 *        min/max и лог. Крупная строка продолжает показывать
 *        живой сырой отсчёт.
 *
 *    - drawGraph()       — экран графика журнала: до
 *      GRAPH_MAX_PTS последних образцов журнала (1 точка = колонка
 *      = 15-минутное окно), свежие — у правого края; автомасштаб
 *      по min/max отображаемого участка, в заголовке — диапазон
 *      значений и шаг «15m/pt». Данные читаются напрямую из
 *      journal.cpp (journalWords + буфер упаковки). Пропуски окон
 *      на графике не показываются (1 колонка = 1 образец).
 *
 *  Управление ПИТАНИЕМ дисплея (oledPowerOn/oledPowerOff)
 *  вынесено в power.cpp — это функции управления энергией,
 *  а не отрисовки. Общая конфигурация — в config.h.
 * ============================================================ */

#include "config.h"

// Контроль синхронности версий файлов: все 6 файлов скетча должны
// быть из ОДНОГО архива (см. GLM_CONFIG_VERSION в config.h).
#if !defined(GLM_CONFIG_VERSION) || GLM_CONFIG_VERSION < 20
#error "config.h устарел (нужна v20): замените ВСЕ 6 файлов скетча из актуального архива"
#endif

// ============================================================
//  Объекты библиотек
//  (display — здесь; объекты датчика DS18B20 — в termo.cpp)
// ============================================================
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);

// ============================================================
//  Инициализация I2C и OLED
//  ВАЖНО: на STM32F401 необходимо явно назначать SDA/SCL
//         через Wire.setSDA()/Wire.setSCL() ДО Wire.begin(),
//         иначе I2C-периферия остаётся на пинах по умолчанию
//         и экран молчит.
//
//  ОБРАБОТКА ОТСУТСТВИЯ OLED: если display.begin() не вернул true,
//  функция пишет лог об ошибке и возвращает false. Вызывающий код
//  (GLM.ino setup()) проверяет флаг bootOledOk и продолжает работу
//  без дисплея: устройство работает в «слепом» режиме — измеряет
//  температуру, ведёт журнал, уходит в сон по расписанию.
//  Для индикации состояния без OLED используется светодиод LED_PIN
//  (если есть): мигание при старте сигнализирует об ошибке OLED.
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
//  Печать температуры со знаком (минус — только для отрицательных).
//  Аргумент — в формате хранения °C×2 (шаг 0.5 °C): печатается
//  как "-23.5" / "23.0". Используется для обеих строк экрана
//  и стартового сплэш-экрана (GLM.ino).
// ============================================================
void printTempSigned(int8_t halfC) {
  if (halfC < 0) display.print('-');
  // Абсолютное значение — сразу в int (защита от переполнения
  // int8 при экстремальном -128, хотя рабочий диапазон уже ±100).
  int a = (halfC < 0) ? -(int)halfC : (int)halfC;
  display.print(a / 2);               // целая часть, °C
  display.print('.');
  display.print((a & 1) ? '5' : '0'); // десятые: всегда 0 или 5
}

// ============================================================
//  Отрисовка экрана
// ============================================================
void drawScreen() {
  display.clearDisplay();

  // --- Верхняя строка: температура крупным шрифтом ---
  // Размер 3 -> 18×24 px на символ. "-23.5" → 5 символов = 90 px
  // (диапазон -50..+50: самый широкий вариант "-50.0" — те же 5).
  //
  // Крупно — ВСЕГДА ЖИВОЙ СЫРОЙ отсчёт tempRawC (termo.cpp): табло
  // интерактивно реагирует на любые изменения среды — пользователь
  // видит, что термометр работает. Состояние ворота сообщают
  // элементы вокруг: нет REJ — справа единица "C"; есть REJ —
  // данные в этот момент ОТБРАСЫВАЮТСЯ. 
  // Геометрия бейджа: "-50.0" = 90 px + отступ 4 + плашка 24 = 118 < 128;
  // с бейджем И символом "C" было бы 138 px — не влезает, поэтому в
  // REJ-режиме бейдж заменяет метку; единица остаётся в «hold»-строке.
  bool rej = tempRejected;
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 4);
  printTempSigned(tempRawC);

  int16_t cx = display.getCursorX();
  if (rej) {
    // Инвертированный бейдж "REJ": белая плашка 24×12, чёрный текст
    // size 1, вертикально по центру крупной строки (y 4..28).
    display.fillRect(cx + 4, 10, 24, 12, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(cx + 7, 12);
    display.print(F("REJ"));
    display.setTextColor(SSD1306_WHITE);   // вернуть цвет остальному
  } else {
    // Градус-метка "C" крупным шрифтом справа от значения
    display.setCursor(cx + 2, 4);
    display.print('C');
  }

  // --- Разделитель ---
  display.drawFastHLine(0, 35, OLED_W, SSD1306_WHITE);

  // --- Нижняя строка: "min..max" + батарея + время RTC ---
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 42);
  printTempSigned(minTempC);
  display.print(F(".."));
  printTempSigned(maxTempC);

  // --- Индикатор батареи: правый край той же строки ---
  // Код ставит x = OLED_W - 50 = 78: строка "X.XXV" (5 символов =
  // 30 px) занимает 78..108, до правого края остаётся запас.
  // Самый широкий диапазон ("-50.0..50.0") = 11 символов = 66 px —
  // пересечений с индикатором батареи (x = 78) нет.
  uint16_t vddMv = readVddMillivolts();
  display.setCursor(OLED_W - 50, 42);
  if (vddMv > 0) {
    display.print(vddMv / 1000.0f, 2);
    display.print('V');
  } else {
    display.print(F("--.-V"));
  }

  // --- Время RTC: самая нижняя строка (y=54) ---
  // Формат "HH:MM" слева, под диапазоном температур.
  // Занимает 5 символов = 30 px, не пересекается с min..max (до 66 px).
  {
    STM32RTC& rtc = STM32RTC::getInstance();
    uint8_t hh, mm, ss; uint32_t sub; STM32RTC::AM_PM ap;
    rtc.getTime(&hh, &mm, &ss, &sub, &ap);
    display.setCursor(0, 54);
    if (hh < 10) display.print('0');
    display.print(hh);
    display.print(':');
    if (mm < 10) display.print('0');
    display.print(mm);
  }

  // --- Строка состояния REJ: замороженное достоверное значение ---
  // Пока ворот отбрасывает сырые отсчёты, выход (currentTempC)
  // заморожен. Крупно показано то, что говорит датчик (с REJ),
  // здесь — во что в данный момент «верит» логгер: «hold X.X C».
  // Размещение: справа от времени RTC (y=54), чтобы не перекрывать.
  if (rej) {
    display.setTextSize(1);
    display.setCursor(36, 54);  // после "HH:MM " (30px + отступ)
    display.print(F("hold "));
    printTempSigned(currentTempC);
    display.print(F(" C"));
  }

  display.display();
}

// ============================================================
//  Экран графика журнала.
//  Данные — журнал journal.cpp (поток образцов: сначала слова,
//  затем хвост буфера упаковки). Рисуются ПОСЛЕДНИЕ n образцов,
//  n = min(общее число, GRAPH_MAX_PTS = ширина экрана): одна
//  колонка пикселей = один образец = 15-минутное окно, свежие
//  образцы — у правого края. Ломаная соединяет соседние точки.
//
//  Масштаб по вертикали — авто: min/max отображаемого участка
//  (в формате °C×2) растягиваются на полосу
//  [GRAPH_TOP_Y..GRAPH_BOTTOM_Y]; плоский ряд разворачивается
//  в окно ±0.5 °C. Заголовок: диапазон значений участка и шаг
//  «15m/pt». Внизу (GRAPH_BOTTOM_Y + 1) — ось времени.
//  Пропуски окон (отказ датчика) на графике не видны:
//  соседние образцы стоят в соседних колонках.
// ============================================================

// Образец журнала по сквозному индексу потока (сначала слова,
// затем буфер упаковки). j ОБЯЗАТЕЛЬНО < wordSamples + packed.
static int8_t graphSampleAt(uint32_t j, uint32_t wordSamples) {
  if (j < wordSamples) {
    return (int8_t)(uint8_t)(journalWords[j >> 2] >> (8 * (j & 3)));
  }
  return journalPackedAt((uint8_t)(j - wordSamples));
}

void drawGraph() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  const uint32_t wordSamples = (uint32_t)journalWordCount() * JOURNAL_SAMPLES_PER_WORD;
  const uint32_t total = wordSamples + journalPackedCount();
  const uint16_t n = (total < (uint32_t)GRAPH_MAX_PTS)
                     ? (uint16_t)total : (uint16_t)GRAPH_MAX_PTS;

  // --- Пустой журнал: сообщение и ось времени ---
  if (n == 0) {
    display.setCursor(0, 0);
    display.print(F("G: journal empty"));
    display.drawFastHLine(0, OLED_H - 1, OLED_W, SSD1306_WHITE);
    display.display();
    return;
  }

  // --- Диапазон значений отображаемого участка (°C×2) ---
  int16_t vmin = 32767;
  int16_t vmax = -32768;
  for (uint16_t c = 0; c < n; c++) {
    int16_t v = graphSampleAt(total - n + c, wordSamples);
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
  }
  if (vmin == vmax) {              // плоский ряд: окно ±0.5 °C
    vmin -= 1;
    vmax += 1;
  }
  const int32_t span  = vmax - vmin;                  // > 0
  const int32_t plotH = GRAPH_BOTTOM_Y - GRAPH_TOP_Y; // 52 px

  // --- Заголовок: масштаб участка + шаг ---
  display.setCursor(0, 0);
  display.print(F("G "));
  printTempSigned((int8_t)vmin);
  display.print(F(".."));
  printTempSigned((int8_t)vmax);
  // Заголовок целиком "G -50.0..50.0" = 13 символов = 78 px; метка
  // шага — 36 px у правого края (x=92), пересечений нет.
  display.setCursor(OLED_W - 36, 0);
  display.print(F("15m/pt"));

  // --- Ось времени (под полем графика) ---
  display.drawFastHLine(0, GRAPH_BOTTOM_Y + 1, OLED_W, SSD1306_WHITE);

  // --- Ломаная графика: точка = колонка, свежие образцы справа ---
  // Вертикальная развёртка: vmin -> GRAPH_BOTTOM_Y,
  // vmax -> GRAPH_TOP_Y; округление (+span/2) гарантирует
  // попадание краёв диапазона точно на границы поля.
  int16_t prevY = 0;
  for (uint16_t c = 0; c < n; c++) {
    int16_t v = graphSampleAt(total - n + c, wordSamples);
    int16_t y = GRAPH_BOTTOM_Y
                - (int16_t)(((int32_t)(v - vmin) * plotH + span / 2) / span);
    int16_t x = (int16_t)(OLED_W - n + c);
    if (c == 0) {
      display.drawPixel(x, y, SSD1306_WHITE);
    } else {
      display.drawLine(x - 1, prevY, x, y, SSD1306_WHITE);
    }
    prevY = y;
  }

  display.display();
}
