#!/bin/bash
# Длинная выдержка (может дать небольшой смаз при резких маневрах), высокий гейн
v4l2-ctl -d /dev/video0 -c exposure=5250
v4l2-ctl -d /dev/video0 -c analogue_gain=1200