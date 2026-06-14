# Attribution — Military fortress world

Эти ассеты (`mili_tech/`) взяты из открытого репозитория:

**engcang/gazebo_maps** — https://github.com/engcang/gazebo_maps
файл `mili_tech.tar.xz` (карта "Military fortress").

## Источник / цитирование

Карта использовалась в работе (см. README репозитория):

> Development of a 3D Mapping System including Object Position for UAV with a
> RGB-D camera in an Unknown and GNSS-denied Environment, *2020 IEMEK*.
> Center for Research Officers for National Defense (RoND), KAIST.

Демонстрация (VINS-Fusion + YOLO v3 tiny): https://youtu.be/5t-6g7UWA7o

## Что изменено для проекта 13.17

Ассеты вендорены в реп как есть, со следующими правками под Gazebo Harmonic:
- `mili_map/model.sdf` — удалена битая ссылка `model://home` (модели нет в архиве)
- `grass_plane/model.sdf` — Ogre material script → PBR
- `digital_wall/model.sdf` — Ogre material script → PBR
- Сам мир портирован в `../mili_fortress.sdf` (оригинальный `mili.world`
  остаётся здесь для справки, но под Harmonic не используется).

Лицензия — согласно upstream-репозиторию (открытый, бесплатный). При
публикации результатов цитировать работу выше.
