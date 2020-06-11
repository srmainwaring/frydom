#!/usr/bin/env bash

gource -s 0.05 -1024x768 -o - | ffmpeg -y -r 60 -f image2pipe -vcodec ppm -i - -vcodec libx264 -preset ultrafast -crf 1 -threads 0 -bf 0 frydom_dev.mp4
