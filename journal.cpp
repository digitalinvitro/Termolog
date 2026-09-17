/* ============================================================
 *  journal.cpp — журнал температуры в ОЗУ (репетиция флеш-версии)
 * ============================================================
 *
 *  НАЗНАЧЕНИЕ
 *  ----------
 *  Отбор подтверждённых замеров температуры в журнал размером
 *  «16К» с битовой картой занятости. Схема специально повторяет
 *  будущую ФЛЕШ-версию (флеш программируется только 1 -> 0,
 *  стирание возвращает единицы), поэтому и в ОЗУ всё ведётся так,
 *  как потом будет во флеш:
 *
 *    - журнал — массив 32-битных слов journalWords[JOURNAL_WORDS]
 *      (16384 байта); в одно слово упаковываются 4 образца int8
 *      (формат °C×2, как везде в проекте): 4 окна по 15 минут =
 *      1 час на слово;
 *    - байты внутри слова идут в хронологическом порядке: МЛАДШИЙ
 *      байт — самый старый образец (little-endian по времени);
 *    - битовая карта journalBitmap[JOURNAL_BITMAP_BYTES]: один бит
 *      на слово журнала (4096 слов = 4096 бит = 512 байт).
 *      Бит = 1 — слово СВОБОДНО (как стёртая флеш), бит = 0 — слово
 *      ЗАПИСАНО. Карта изначально инициализируется ЕДИНИЦАМИ —
 *      точная модель стёртого флеш-сектора;
 *    - заполнение журнала запускает «цикл стирания» (эмуляция
 *      erase сектора): карта снова вся в единицах, слова — в
 *      0xFFFFFFFF, курсор — в ноль.
 *
 *  ОТБОР ОБРАЗЦОВ
 *  --------------
 *  Физический опрос датчика НЕ меняется (15 с в SLEEP, 5 с в
 *  ACTIVE). Журнал прореживает поток выхода конвейера termo.cpp:
 *  на каждое 15-минутное окно RTC (epoch / JOURNAL_PERIOD_S)
 *  фиксируется ОДНО значение — последнее подтверждённое в этом
 *  окне. Окна нумеруются RTC-временем (epoch/900), поэтому отбор
 *  не зависит от режима SLEEP/ACTIVE и пропусков замеров.
 *
 *  ЁМКОСТЬ
 *  -------
 *  4096 слов × 4 образца = 16384 образца по 15 минут = 4096 часов
 *  ≈ 170 суток непрерывной работы до «цикла стирания».
 *
 *  ВРЕМЯ ОБРАЗЦОВ И ВЫГРУЗКА ПО UART (флаг JUR_2_UART)
 *  -------------------------------------------------------
 *  Журнал хранит ТОЛЬКО значения, поэтому время для выгрузки
 *  «время — значение» восстанавливается по двум данным:
 *    - якорь s_firstWindow — номер 15-минутного окна ПЕРВОГО
 *      образца цикла заполнения (после journalInit или «цикла
 *      стирания»);
 *    - карта пропусков s_gap[]: 1 байт на ОБРАЗЕЦ — сколько
 *      15-минутных окон пропущено ПЕРЕД ним (0 = окно соседнее).
 *      Окно образца = окно предыдущего + 1 + пропуск. Молчание
 *      дольше 63 ч (255 окон) ограничивается значением 255
 *      (лог печатает предупреждение) — время после такого разрыва
 *      сдвигается. Пропуски возможны только при отказе датчика
 *      на целые окна: при живом датчике подача значений идёт
 *      каждые 5-15 с без перерывов.
 *  journalDumpCsv() печатает в LOG_OBJECT строки-заголовки с '#'
 *  и строки «<время>,<значение>»: время — RTC-эпоха [с] начала
 *  окна образца, значение — температура [°C]. Дамп — блокирующий
 *  (вызывается из ACTIVE; полный журнал при 115200 бод ~20 с,
 *  экран на это время не обновляется — см. GLM.ino).
 *  Дамп печатает только в ЖИВОЙ порт: перед вызовом GLM.ino
 *  поднимает его (ensureLogPortForDump — CDC-порт без хоста
 *  закрыт, печати — no-op). Каждые JOURNAL_DUMP_ALIVE_EVERY
 *  строк проверяется живость порта (logPortAlive, config.h):
 *  пропажа хоста посреди выгрузки прерывает дамп (возврат false,
 *  GLM.ino рисует «host lost!»), а не блокирует печать навечно.
 *
 *  ИСПОЛЬЗОВАНИЕ ОЗУ
 *  -----------------
 *  журнал 16384 Б + карта 512 Б + служебные переменные ≈ 16.9 КБ
 *  из 64 КБ SRAM F401 (~26 %) — при JUR_2_UART = 0.
 *  При JUR_2_UART = 1 добавляется карта пропусков (16384 Б):
 *  всего ≈ 33 КБ (~52 %). Это плата за тестовый режим: во
 *  флеш-версии журнал и карта уедут во флеш-секторы, ОЗУ
 *  освободится; стратегия хранения времени во флеш-версии —
 *  отдельное решение.
 *
 *  API (прототипы — в config.h):
 *    journalInit()        — инициализация/«стирание» (из setup();
 *                           во флеш-версии станет erase сектора);
 *    journalOnSample()    — подать подтверждённый замер (termo.cpp
 *                           вызывает на каждом выходе конвейера);
 *    journalWordCount()   — слов записано с последнего «стирания»;
 *    journalPackedCount() — образцов ждёт упаковки (0..3);
 *    journalWordAt()      — слово по индексу (выгрузка/диагностика);
 *    journalPackedAt()    — образец буфера упаковки (график);
 *  только при JUR_2_UART = 1:
 *    journalFirstWindow() — окно первого образца цикла (якорь);
 *    journalGapAt()       — пропуск окон перед образцом (диагностика);
 *    journalDumpCsv()     — дамп CSV «время,значение» в LOG_OBJECT;
 *                           false = хост пропал посреди дампа.
 *
 *  Общая конфигурация — в config.h (секция «Журнал температуры»).
 * ============================================================ */

#include "config.h"

// Контроль синхронности версий файлов: все 6 файлов скетча должны
// быть из ОДНОГО архива (см. GLM_CONFIG_VERSION в config.h).
#if !defined(GLM_CONFIG_VERSION) || GLM_CONFIG_VERSION < 20
#error "config.h устарел (нужна v20): замените ВСЕ 6 файлов скетча из актуального архива"
#endif

// ============================================================
//  Определения общих массивов (extern-объявления — в config.h).
//  До journalInit() содержимое не важно: init записывает модель
//  стёртой флеш (0xFF / 0xFFFFFFFF).
// ============================================================
uint32_t journalWords[JOURNAL_WORDS];              // журнал: 4096 слов × 4 Б
uint8_t  journalBitmap[JOURNAL_BITMAP_BYTES];      // карта: 1 = свободно, 0 = записано

// ============================================================
//  Служебное состояние (static — только этому модулю)
// ============================================================
static uint16_t s_writeCursor = 0;   // индекс следующего слова журнала (0..4095)
static uint16_t s_wordCount   = 0;   // слов записано с последнего «стирания»
static uint8_t  s_packN       = 0;   // образцов уже в буфере упаковки (0..3)
static uint8_t  s_packBuf[JOURNAL_SAMPLES_PER_WORD]; // буфер упаковки слова
static uint32_t s_lastBucket  = 0;   // номер последнего 15-минутного окна (epoch/900)
static bool     s_seeded      = false; // пришёл ли первый замер после init
static int8_t   s_lastValue   = 0;   // последнее подтверждённое значение, °C×2

// ============================================================
//  Время образцов — только при JUR_2_UART = 1. См. раздел
//  «ВРЕМЯ ОБРАЗЦОВ И ВЫГРУЗКА ПО UART» в шапке модуля.
// ============================================================
#if JUR_2_UART
static uint32_t s_firstWindow = 0;    // якорь: окно (epoch/900) первого образца цикла
static uint32_t s_lastRecordedWindow = 0; // окно последнего ЗАПИСАННОГО образца
static uint8_t  s_packGap[JOURNAL_SAMPLES_PER_WORD]; // пропуск окон перед каждым образом буфера
static uint32_t s_packWin[JOURNAL_SAMPLES_PER_WORD]; // окно каждого образца буфера
static uint8_t  s_gap[JOURNAL_WORDS * JOURNAL_SAMPLES_PER_WORD]; // карта пропусков, Б/образец
#endif // JUR_2_UART

// ============================================================
//  «Стирание» (эмуляция erase флеш-сектора): журнал — в
//  0xFFFFFFFF, карта — в единицы, курсор и счётчик — в ноль.
//  Молчаливая часть; лог пишут вызывающие (init / цикл стирания).
//  ВНИМАНИЕ: JOURNAL_BITMAP_BYTES = 512 > 255 — индекс цикла
//  обязательно uint16_t (иначе вечный цикл).
// ============================================================
static void eraseArrays() {
  for (uint16_t i = 0; i < JOURNAL_WORDS; i++) {
    journalWords[i] = 0xFFFFFFFFUL;
  }
  for (uint16_t i = 0; i < JOURNAL_BITMAP_BYTES; i++) {
    journalBitmap[i] = 0xFF;
  }
  s_writeCursor = 0;
  s_wordCount   = 0;
}

// ============================================================
//  Инициализация / принудительное «стирание» журнала.
//  Вызывается один раз из setup(). Во флеш-версии здесь будет
//  реальный erase сектора; в ОЗУ достаточно перезаписи. Первое
//  15-минутное окно пересеивается следующим замером.
// ============================================================
void journalInit() {
  eraseArrays();
  s_packN     = 0;
  s_seeded    = false;
  s_lastValue = 0;
#if JUR_2_UART
  s_firstWindow        = 0;   // якорь установит первый замер (посев)
  s_lastRecordedWindow = 0;   // записанных образцов в новом цикле нет
#endif
  LOG_OBJECT.print(F("[Journal] init: "));
  LOG_OBJECT.print(JOURNAL_BYTES);
  LOG_OBJECT.print(F(" B = "));
  LOG_OBJECT.print(JOURNAL_WORDS);
  LOG_OBJECT.print(F(" words ("));
  LOG_OBJECT.print((uint32_t)JOURNAL_WORDS * JOURNAL_SAMPLES_PER_WORD * JOURNAL_PERIOD_S / 3600UL);
  LOG_OBJECT.print(F(" h capacity), bitmap "));
  LOG_OBJECT.print(JOURNAL_BITMAP_BYTES);
  LOG_OBJECT.println(F(" B (1 = word free)"));
}

// ============================================================
//  Запись накопленного слова в журнал по курсору.
//  Бит карты сбрасывается 1 -> 0 — ровно как программирование
//  флеш. При выходе курсора за границу журнала — «цикл стирания».
// ============================================================
static void commitWord() {
  if (s_writeCursor >= JOURNAL_WORDS) {
    LOG_OBJECT.println(F("[Journal] full -> erase cycle (flash erase emulation)"));
    eraseArrays();
#if JUR_2_UART
    // Новое заполнение начинается с ЭТОГО слова: якорь времени —
    // окно его старейшего образца, пропуск перед первым образцом
    // нового цикла не существует (карта стёрта).
    s_firstWindow = s_packWin[0];
    s_packGap[0]  = 0;
#endif
  }

#if JUR_2_UART
  // Карта пропусков пишется в общем потоке образцов (не по словам):
  // base — индекс первого образца этого слова в потоке.
  uint32_t gapBase = (uint32_t)s_writeCursor * JOURNAL_SAMPLES_PER_WORD;
  for (uint8_t k = 0; k < JOURNAL_SAMPLES_PER_WORD; k++) {
    s_gap[gapBase + k] = s_packGap[k];
  }
#endif

  // Упаковка 4 × int8 в слово: младший байт — самый старый образец.
  uint32_t w = (uint32_t)s_packBuf[0]
             | ((uint32_t)s_packBuf[1] << 8)
             | ((uint32_t)s_packBuf[2] << 16)
             | ((uint32_t)s_packBuf[3] << 24);

  journalWords[s_writeCursor] = w;
  // бит (s_writeCursor % 8) байта (s_writeCursor / 8): 1 -> 0
  journalBitmap[s_writeCursor >> 3] &= (uint8_t)~(1u << (s_writeCursor & 7));
  s_writeCursor++;
  s_wordCount++;

  LOG_OBJECT.print(F("[Journal] word #"));
  LOG_OBJECT.print(s_wordCount);
  LOG_OBJECT.print(F(" @"));
  LOG_OBJECT.print(s_writeCursor - 1);
  LOG_OBJECT.print(F(" =0x"));
  LOG_OBJECT.println(w, HEX);
}

// ============================================================
//  Подача подтверждённого замера в журнал (вызывается из termo.cpp
//  на каждом выходе конвейера). Логика отбора:
//    - самый первый замер после init только СЕЕТ окно (записи нет:
//      окно только началось, «последнего значения» у него ещё нет);
//    - при переходе в новое 15-минутное окно завершившееся окно
//      фиксируется своим ПОСЛЕДНИМ подтверждённым значением —
//      образец кладётся в буфер упаковки; каждые 4 образца (1 час)
//      слово дописывается в журнал.
//  Пропущенные окна (устройство молчало) просто не порождают
//  образцов — журнал дописывается плотно, без дыр. При
//  JUR_2_UART = 1 число пропущенных окон перед каждым образцом
//  запоминается в карте пропусков (s_gap) — по ней выгрузка
//  восстанавливает время образцов.
// ============================================================
void journalOnSample(uint32_t epochS, int8_t halfC) {
  uint32_t bucket = epochS / JOURNAL_PERIOD_S;

  if (!s_seeded) {                 // первый замер после init
    s_seeded = true;
#if JUR_2_UART
    s_firstWindow = bucket;        // якорь времени — окно первого замера
#endif
    s_lastBucket = bucket;
    s_lastValue  = halfC;
    return;
  }

  if (bucket != s_lastBucket) {    // окно s_lastBucket завершено
#if JUR_2_UART
    // Сколько окон ПРОПУЩЕНО между предыдущим записанным образцом
    // и этим (записывается ПОСЛЕДНЕЕ значение завершившегося окна)?
    uint32_t skipped;
    if (s_wordCount == 0 && s_packN == 0) {
      skipped = 0;                 // самый первый образец цикла
    } else {
      skipped = s_lastBucket - s_lastRecordedWindow - 1;
      if (skipped > 0xFF) {        // больше байта: молчали дольше 63 ч
        skipped = 0xFF;
        LOG_OBJECT.println(F("[Journal] gap clamped to 255 windows (>63 h silence)"));
      }
    }
    s_packGap[s_packN] = (uint8_t)skipped;
    s_packWin[s_packN] = s_lastBucket;
#endif
    s_packBuf[s_packN] = (uint8_t)s_lastValue;  // последнее значение окна
    s_packN++;
#if JUR_2_UART
    s_lastRecordedWindow = s_lastBucket;
#endif
    if (s_packN >= JOURNAL_SAMPLES_PER_WORD) {
      s_packN = 0;
      commitWord();                // 4 образца = слово -> в журнал
    }
    s_lastBucket = bucket;
  }
  s_lastValue = halfC;             // кандидат в «последнее значение окна»
}

// ============================================================
//  Диагностика: сколько слов записано с последнего «стирания»
// ============================================================
uint16_t journalWordCount() {
  return s_wordCount;
}

// Диагностика: сколько образцов ждёт упаковки (0..3)
uint8_t journalPackedCount() {
  return s_packN;
}

// Доступ к слову журнала по индексу (выгрузка по UART, будущий
// сервис чтения). Вне диапазона — модель стёртой флеш (0xFFFFFFFF).
uint32_t journalWordAt(uint16_t idx) {
  return (idx < JOURNAL_WORDS) ? journalWords[idx] : 0xFFFFFFFFUL;
}

// Образец буфера упаковки (0..3). Используется экраном графика
// для «хвоста» текущего часа и тестами. Вне диапазона — 0.
int8_t journalPackedAt(uint8_t k) {
  return (k < JOURNAL_SAMPLES_PER_WORD) ? (int8_t)s_packBuf[k] : 0;
}

#if JUR_2_UART
// Якорь времени: окно (epoch/900) первого образца текущего цикла
// заполнения. До первого замера после init — 0.
uint32_t journalFirstWindow() {
  return s_firstWindow;
}

// Пропуск окон ПЕРЕД образцом idx. Поток образцов: сначала
// закоммиченные в слова, затем буфер упаковки. Вне диапазона — 0.
uint8_t journalGapAt(uint32_t idx) {
  uint32_t wordSamples = (uint32_t)s_wordCount * JOURNAL_SAMPLES_PER_WORD;
  if (idx < wordSamples) {
    return s_gap[idx];
  }
  if (idx < wordSamples + s_packN) {
    return s_packGap[idx - wordSamples];
  }
  return 0;
}

// ============================================================
//  Выгрузка журнала в LOG_OBJECT дампом CSV.
//  Формат: строки-заголовки с '#', затем по строке на образец:
//      <RTC-эпоха начала окна>,<температура °C>
//  Время восстанавливается от якоря по карте пропусков; хвост
//  буфера упаковки печатается следом за словами. Вызов — из
//  ACTIVE (блокирующий): на время дампа конвейер замеров не
//  работает, экран не обновляется (GLM.ino показывает кадр
//  «CSV dump...» и после дампа продлевает активную минуту).
//  Возвращает true — дамп дошёл до конца; false — хост пропал
//  посреди выгрузки (проверка logPortAlive() каждые
//  JOURNAL_DUMP_ALIVE_EVERY строк; печать при мёртвом хосте может
//  блокироваться навечно в ряде версий ядра — дамп лучше прервать).
// ============================================================
bool journalDumpCsv() {
  const uint16_t words  = s_wordCount;
  const uint8_t  packed = s_packN;
  const uint32_t wordSamples = (uint32_t)words * JOURNAL_SAMPLES_PER_WORD;
  const uint32_t total = wordSamples + packed;

  uint32_t t0 = millis();
  LOG_OBJECT.println(F("# GLM temperature journal dump"));
  LOG_OBJECT.print(F("# samples="));
  LOG_OBJECT.print(total);
  LOG_OBJECT.print(F(" (words="));
  LOG_OBJECT.print(words);
  LOG_OBJECT.print(F(", packed="));
  LOG_OBJECT.print(packed);
  LOG_OBJECT.print(F(", first_window="));
  LOG_OBJECT.println(s_firstWindow);
  LOG_OBJECT.println(F("# time,value"));
  LOG_OBJECT.println(F("# time = RTC epoch [s] of 15-min window start; value = temperature [C]"));

  if (total == 0) {
    LOG_OBJECT.println(F("# journal is empty"));
  } else {
    uint32_t win = s_firstWindow;   // окно текущего образца
    for (uint32_t j = 0; j < total; j++) {
      // Живость порта — каждые JOURNAL_DUMP_ALIVE_EVERY строк.
      // Пропажа хоста (кабель выдернут, монитор закрыт и ядро
      // сбросило соединение) останавливает дамп, не доводя до
      // вечного блока внутри print().
      if ((j % JOURNAL_DUMP_ALIVE_EVERY) == 0 && !logPortAlive()) {
        LOG_OBJECT.println(F("# dump aborted: host lost"));
        safeSerialFlush(200);       // что успело — то успело
        LOG_OBJECT.print(F("[Journal] dump ABORTED after "));
        LOG_OBJECT.print(j);
        LOG_OBJECT.print(F(" of "));
        LOG_OBJECT.print(total);
        LOG_OBJECT.print(F(" lines in "));
        LOG_OBJECT.print(millis() - t0);
        LOG_OBJECT.println(F(" ms"));
        return false;
      }
      if (j > 0) {
        // +1 соседнее окно + все окна, пропущенные перед образцом
        win += 1UL + ((j < wordSamples) ? s_gap[j]
                                        : s_packGap[j - wordSamples]);
      }
      int8_t v = (j < wordSamples)
          ? (int8_t)(uint8_t)(journalWords[j >> 2] >> (8 * (j & 3)))
          : (int8_t)s_packBuf[j - wordSamples];
      LOG_OBJECT.print(win * JOURNAL_PERIOD_S);
      LOG_OBJECT.print(',');
      LOG_OBJECT.println(v * 0.5f, 1);
    }
  }

  safeSerialFlush(200);             // хвост дампа — в провод
  LOG_OBJECT.print(F("[Journal] dump done: "));
  LOG_OBJECT.print(total);
  LOG_OBJECT.print(F(" lines in "));
  LOG_OBJECT.print(millis() - t0);
  LOG_OBJECT.println(F(" ms"));
  return true;
}
#endif // JUR_2_UART
