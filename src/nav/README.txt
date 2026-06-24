═══════════════════════════════════════════════════════════════════════════
  src/nav — пакет nav_pkg (нейросети навигации NN1/NN2) + офлайн-тулзы
═══════════════════════════════════════════════════════════════════════════

Раскладка и роли нод подробно — в корневом CLAUDE.md (раздел «OpenHD-оверлей и
nav_pkg»). Кратко:
  nav_pkg/nn1/  — NN1, якорная локализация: nn1_anchor (SuperPoint+LightGlue) +
                  ray_tracer (засечка по ориентиру → поправка VINS).
  nav_pkg/nn2/  — NN2, топокарта: nn2_scene (DINOv2 + FAISS) + relocalizer.
  nav_pkg/openhd_streamer.py — даунлинк в OpenHD с оверлеем детекций.
  tools/nn1, tools/nn2_scene, tools/nn2_route — офлайн-скрипты (сборка баз/карт,
                  обучение, оценка).

Запуск: ros2 launch nav_pkg nav.launch.py use_sim_time:=true (камеру/VINS не
поднимает; в симуляции включается из src/sim/sim_nav.launch.py).

───────────────────────────────────────────────────────────────────────────
  GPU vs CPU — где NN1/NN2 упираются в отсутствие GPU
───────────────────────────────────────────────────────────────────────────
(контекст: ветка nn2_c3_cpu — GPU-less прогон на боксе без NVIDIA, пока T4 в
дефиците. Ядро gazebo→SITL→VINS от GPU отвязано; ниже — про nn-сторону.)

ЯДРО gazebo→SITL→VINS в GPU-стену НЕ упирается
  nav.launch оно не поднимает. VINS — Ceres+LK (CPU), камера имеет CPU drop-in
  (camera_node_cpu), Gazebo — софтовый llvmpipe. Барьер тут — fps софтрендера
  (perf-гейт), а не отсутствие GPU.

СТЕНА №1 — тривиальная, ловится сразу при старте nn-нод
  Дефолты нод захардкожены на CUDA:
    nav_pkg/nn1/nn1_anchor.py:35  declare_parameter("device", "cuda")
                                  → anchor_matcher: LightGlue(...).to("cuda")
    nav_pkg/nn2/nn2_scene.py:47   declare_parameter("device", "cuda")
                                  → SceneEncoder: DINOv2 .to("cuda")
  На машине без GPU это НЕМЕДЛЕННЫЙ краш на загрузке модели (torch: no CUDA
  device). Но это НЕ настоящая стена — везде это ROS-параметр / --device,
  лечится device:=cpu. Образец правильного поведения уже есть:
    tools/nn2_route/train_route_coords.py:120
      dev = device if (device != "cuda" or torch.cuda.is_available()) else "cpu"
  TODO (если гонять nn на CPU-боксе): завести такой же авто-фолбэк cuda→cpu в
  nn1_anchor / nn2_scene, чтобы ноды не падали и не требовали ручных device:=cpu.
  Офлайн-тулзы (build_scene_map, eval_isometry, visualize_fiber) — argparse
  --device default="cuda", достаточно передать --device cpu.

СТЕНА №2 — настоящая: пропускная способность, а не «не запустится»
  Torch и FAISS на CPU РАБОТАЮТ, вопрос в скорости (×10–50):
    • nn2 / DINOv2 ViT-S/14: ~0.2–1 с/кадр на CPU. Живой инференс @3 с —
      терпимо. Сборка карты из bag (build_scene_map по сотням-тысячам кадров)
      и обучение топографа на реальных bag'ах — минуты→десятки минут.
    • nn1 / SuperPoint+LightGlue: ~0.5–2 с/матч на CPU, цель ~1 Гц → на грани;
      против многокадровой reference-базы — мимо каденса.
    • FAISS — уже faiss-cpu, поиск по карте микросекунды: стены нет в принципе.
    • Геометрия (nn1/geo.py, ray_tracer, nn2/metric_decode — чистый numpy) —
      CPU-native, стены нет.

ВЫВОД
  Жёсткой «не поедет» стены у NN1/NN2 нет — у всего есть CPU-путь.
    • Мгновенная мелочь: дефолт device="cuda" → краш (фикс параметром/фолбэком).
    • Мягкая throughput-стена: для СМОУК-ТЕСТА (нода поднялась, топики идут,
      FAISS отвечает, матчинг/трейсинг считают верно) CPU-бокса хватает; для
      ОБУЧЕНИЯ топографа на реальных bag'ах и REAL-TIME релокализации — нет,
      вот за этим возвращаемся на T4.
═══════════════════════════════════════════════════════════════════════════
