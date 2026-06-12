#!/bin/bash
# Короткая выдержка, минимальный гейн
v4l2-ctl -d /dev/video0 -c exposure=800
v4l2-ctl -d /dev/video0 -c analogue_gain=100