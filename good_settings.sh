#!/bin/bash
v4l2-ctl -d /dev/video$1 \
--set-ctrl brightness=100 \
--set-ctrl contrast=100 \
--set-ctrl saturation=100 \
--set-ctrl white_balance_temperature=4000 \
--set-ctrl white_balance_temperature_auto=0 \
--set-ctrl power_line_frequency=2 \
--set-ctrl sharpness=24 \
--set-ctrl backlight_compensation=0 \
--set-ctrl exposure_auto=1 \
--set-ctrl exposure_absolute=25 \
--set-ctrl gain=0 \
--set-ctrl focus_absolute=0 \
--set-ctrl focus_auto=0 \