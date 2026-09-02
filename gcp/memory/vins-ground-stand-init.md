---
name: vins-ground-stand-init
description: "«ODO -- после долгого стояния на земле» (2026-08-28) — две причины: all_image_frame эстиматора растёт без предела в INITIAL (init O(N³) → 109-167 с на попытку), плюс штормы cv::Exception в трекере (findFundamentalMat на неподвижных точках, лотерея 0.3%/набор)"
metadata:
  type: project
---

Диагноз по bag'ам `docker/sim/output/joystick/odom_gets_borken/` (odom_broken_1/2 — арм
на 226/170 с bag-времени, `/odometry`=0 сообщений; odom_ok — арм на 18 с, init через 3.8 с).

**1. Эстиматор (корень).** В `Estimator::slideWindow()` ветка MARGIN_SECOND_NEW НЕ чистит
`all_image_frame` — на неподвижном дроне все кадры не-ключевые, карта растёт 10 кадров/с,
~40 КБ/кадр (замерено живьём: RSS vins_estimator +404 КБ/с, 761 МБ через 44 мин на земле).
При первой параллаксе `initialStructure()` → `visualInitialAlign()` → `LinearAlignment`/
`RefineGravity` строят плотную (3N+4)² матрицу и LDLT — бенч Eigen в контейнере p1317_nav:
N=185 → 0 с; N=1650 (broken_2) → 22 с на решение, ~109 с на попытку (1 Linear + 4 Refine);
N=1900 (broken_1) → 33 с / ~167 с. Полёты длились 53-64 с → ни одна попытка не завершилась.
Через 45 мин стояния N≈25000 → матрица 45 ГБ → bad_alloc.

**2. Трекер (усилитель).** `cv::findFundamentalMat(FM_RANSAC)` в `rejectWithF` на ДВУХ
ОДИНАКОВЫХ наборах точек (кадры на земле бит-в-бит идентичны, LK даёт сдвиг ровно 0)
бросает `matrix.cpp:766 rowRange assert` (OpenCV 4.10; run7Point → solveCubic=-1 →
rowRange(0,-3)); воспроизведено в cv2 контейнера: ~1/300 случайных наборов. Набор точек
на земле заморожен → «лотерея» решается детерминированно → шторм исключений на каждом
кадре 20-150 с, пока сцена не изменится (1.4% пикселей раз в десятки секунд). try/catch
форка (4c826bd) глотает исключение ПОСЛЕ `last_image_time=t` и ДО `pub_count++` →
`/feature` молчит без discontinuity-сброса, копится долг freq-контроля → после шторма
30 Гц до выплаты (ok: 7 с дыры / 3.5 с burst; broken_1: 48 с / 20 с). «Гонка размера на
холодном старте» из коммитов de17c71/4c826bd — на самом деле этот же баг (стартовые
кадры одинаковые).

**Why:** без этого ЛЮБОЙ сценарий «долго стоим на земле с поднятым VINS» (реальный борт с
ручным `vins_m`, реплеи с паузой) даст ODO -- на весь полёт; симптом выглядит как «VINS
не инитится», а это O(N³) и память.
**Фиксы СДЕЛАНЫ и ДОКАЗАНЫ ПОЛЁТОМ 2026-08-28 (коммиты e4b71ea в 13.17, 1ed92c7 в форке):** (а) лётная нода
`bootstrap_arch2` шлёт `/restart` на фронте armed (`config.vins_restart_arm`,
`BS_VINS_RESTART_ARM`, дефолт 1; freefly_lv экспортирует явно); (б) форк, ветка
1317_debug: `rejectWithF` пропускает findFundamentalMat при max-сдвиге точек <1e-3 px
и ловит cv::Exception внутри; `processImage` сбрасывает состояние (clearState +
setParameter) при `all_image_frame` >300 в INITIAL (MAX_INIT_FRAMES). Проверка:
4.5 мин на земле — RSS 41→42 МБ, исключений 0, /feature 10 Гц; полёт после стояния —
ODO появился, LOITER работает (подтверждение пилота).
**How to apply:** для старых стеков/реплеев без фикса — перед полётом свежий
`make restart-all` (стек, простоявший >10 мин, VINS не поднимет).
Связано: [[freefly-phase-stats]] (долг freq-контроля), [[vins-solver-fix]].
