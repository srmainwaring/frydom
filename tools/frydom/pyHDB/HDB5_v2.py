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
"""
    Module to create a hydrodynamic database for FRyDoM.
"""

import os
from math import *
import numpy as np

from bem_reader_v2 import *
from pyHDB import *

class HDB5(object):

    """
        Class HDB5 for dealing with *.h5 files.
    """

    def __init__(self):

        """
            Constructor of the class HDB5.
        """

        self._hdb5 = pyHDB()

        return

    def nemoh_reader(self, input_directory='.', nb_faces_by_wavelength=None):

        """This function reads the *.cal file and stores the data.

        Parameters
        ----------
        input_directory : string, optional
            Path to directory of *.cal file.
        nb_faces_by_wavelength : float, optional
            Number of panels per wave length.
        """

        if not os.path.isabs(input_directory):
            input_directory = os.path.abspath(input_directory)

        # Verifying there is the Nemoh.cal file inside input_directory
        nemoh_cal_file = os.path.join(input_directory, 'Nemoh.cal')
        if not os.path.isfile(nemoh_cal_file):
            raise AssertionError('Folder %s seems not to be a Nemoh calculation folder as '
                                 'we did not find Nemoh.cal' % input_directory)

        print('========================')
        print('Reading Nemoh results...')
        print('========================')

        if nb_faces_by_wavelength is None:
            nb_faces_by_wavelength = 10

        # Reading *.cal.
        NemohReader(self._hdb5,cal_file=nemoh_cal_file, test=True, nb_face_by_wave_length=nb_faces_by_wavelength)

        print('-------> Nemoh data successfully loaded from "%s"' % input_directory)

    def _initialize(self):

        """This function updates and improve the hydrodynamic database (computation of RK and diffraction loads, impulse response functions, interpolation, etc.)."""