#!/usr/bin/env python
#  -*- coding: utf-8 -*-
# ==========================================================================
# FRyDoM - frydom-ce.org
# 
# Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
# All rights reserved.
# 
# Use of this source code is governed by a GPLv3 license that can be found
# in the LICENSE file of FRyDoM.
# 
# ==========================================================================

import sys
from distutils.spawn import find_executable
from subprocess import call

ffmpeg = find_executable('ffmpeg')


def main():
    if sys.argv[1] is not None:
        filename = str(sys.argv[1])
    else:
        filename = "frydom_video.mp4"

    call([ffmpeg,
          "-framerate", "10",
          "-i", "screenshot%5d.bmp",
          "-c:v", "libx264",
          "-preset", "slow",
          "-crf", "21",
          filename
          ])


if __name__ == '__main__':
    main()
