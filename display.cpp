/* ============================================================
 *  Функции управления дисплеем OLED SSD1306
 *  Логгер температуры — STM32F401 + SSD1306 + DS18B20
 * ============================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

// Объект дисплея
extern Adafruit_SSD1306 display;
extern Print& LOG_OBJECT;

// Переменные температуры (объявлены в GLM.ino)
extern float currentTempC;
extern float minTempC;
extern float maxTempC;

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
//  Печать числа со знаком (минус — только для отрицательных)
//  Используется для обеих строк экрана.
// ============================================================
void printTempSigned(float t) {
  if (t < 0.0f) display.print('-');
  // Абсолютное значение с одним знаком после запятой
  display.print(fabsf(t), 1);
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

// Сырые коды последнего замера; пишутся при каждом вызове.
extern uint16_t g_vddRaw, g_vddCal;

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
