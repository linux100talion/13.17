# Регуляторы (ESC) реального борта — настройки

`xcross_hv3.ixi` — снимок настроек всех четырёх ESC (Multiple-ESC Setup file BLHeliSuite32),
снят 2026-09-24 после правки Beacon Delay. Текст, переносится через буфер обмена.

**Железо.** Flycolor X-Cross HV3 60A 5-12S, раскладка `Flycolor_X_Cross_HV3_G071`, прошивка
BLHeli_32 **rev 31.101** — предрелизная («pre-release rev31.x prototype sample»): свежий
BLHeliSuite32 32.10 её не берёт и обновить до релиза не даёт. Работает только
**BLHeliSuite32Test 31.10.0.1** (Windows), выложенный продавцом Rotorama на странице этих ESC:
`https://www.rotorama.cz/cms/assets/docs/1ca0fca3d3bb1efff638b28b816b243b/26765-1/blhelisuite32-31.10.0.1-1.zip`
(sha256 `d5a1b62e177ffd7fd5edf102ab53552c7d28809cfd7c52c57294d3387b701ae0`). **Копия лежит здесь же** —
`blhelisuite32-31.10.0.1.zip` (BLHeli_32 закрыт с 2024, ссылка магазина может пропасть). У самой Flycolor
официальной ссылки не нашли. Программа крутится в ВМ Win11 на ноуте.

**Что меняли.** Beacon Delay: 10 мин → **Infinite** (`Eep_Pgm_Beacon_Delay=0` в файле) — маяк
BLHeli_32 пищал моторами после 10 мин нулевого газа на столе. Перед полётами решить, нужен ли
маяк для поиска упавшего борта (вернуть 10 мин или меньше). Остальное — как пришло с завода.
Направление всех четырёх — 1 (штатное); реверс мотора 4 делает полётник по DShot
(`SERVO_BLH_RVMASK,8`), в ESC не трогать.

## Как снять заново

1. Пропеллеры снять, батарею подключить. USB полётника — в ноут, в ВМ: Devices → USB →
   «StellarH7V2» (фильтр ВМ цепляет его сам при новом подключении, пока Win11 запущена).
2. BLHeliSuite32Test: интерфейс «BLHeli32 Bootloader (Betaflight/Cleanflight)», COM полётника,
   Connect → Read Setup (все 4 ESC). Проброс к ESC разрешает `SERVO_BLH_AUTO=1`.
3. Save Setup в `.ixi` → открыть в Notepad → Ctrl+A, Ctrl+C → на ноуте сохранить сюда
   (`xclip -o -selection clipboard > distro/doc/HW/esc/xcross_hv3.ixi`).

## Как восстановить

1. На ноуте: `xclip -selection clipboard < distro/doc/HW/esc/xcross_hv3.ixi` → в Win11 вставить в
   Notepad → Save As `xcross_hv3.ixi` (тип «All files»).
2. BLHeliSuite32Test → Connect → Load Setup из файла → Write Setup → Read Setup для проверки.
3. ТОЛЬКО на эти же ESC с той же прошивкой (Layout `Flycolor_X_Cross_HV3_G071`, rev 31.101).
   На другой раскладке/прошивке файл запишет чушь.
4. Disconnect → USB обратно в Orin; полётник сам выходит из проброса через 10 с
   (`SERVO_BLH_TMOUT,10`); mavlink-router на Orin после выдернутого USB — `reset-failed` + `start`.
