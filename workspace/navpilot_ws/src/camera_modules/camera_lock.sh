#!/bin/zsh
DEV=/dev/video0

# Hard-lock zoom
v4l2-ctl -d $DEV -c zoom_absolute=0

# Re-assert focus (OBSBOT sometimes couples these)
v4l2-ctl -d $DEV -c focus_automatic_continuous=0
v4l2-ctl -d $DEV -c focus_absolute=20
