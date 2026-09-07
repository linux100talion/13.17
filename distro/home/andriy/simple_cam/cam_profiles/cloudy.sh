#!/bin/bash
v4l2-ctl -d /dev/video0 -c exposure=2500
v4l2-ctl -d /dev/video0 -c analogue_gain=300