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

import numpy as np

inf = float('inf')  # Definition of infinity for depth

class pyHDB():
    """
        Class for storing the hydrodynamique database.
    """

    def __init__(self):

        # Environmental data.
        self.rho_water = 0
        self.grav = 0
        self.depth = -1
        self.x_wave_measure = 0.
        self.y_wave_measure = 0.

        # Wave frequencies.
        self.nb_wave_freq = 0
        self.min_wave_freq = 0.
        self.max_wave_freq = 0.
        self.wave_freq = np.array([])
        self._iwcut = None

        # Wave directions.
        self.nb_wave_dir = 0
        self.min_wave_dir = 0.
        self.max_wave_dir = 0.
        self.wave_dir = np.array([])

        # Kochin parameters.
        self.has_kochin = False
        self.nb_dir_kochin = 0
        self.min_dir_kochin = 0.
        self.max_dir_kochin = 0.
        self.wave_dir_Kochin = np.array([])

        # Bodies.
        self.nb_bodies = 0
        self.bodies = []

        # Version.
        self.version = 2.0

    def set_wave_frequencies(self):
        """Frequency array of BEM computations in rad/s

        Returns
        -------
        np.ndarray
        """

        self.wave_freq = np.linspace(self.min_wave_freq, self.max_wave_freq, self.nb_wave_freq, dtype=np.float)

    @property
    def omega(self):
        """Frequency array of BEM computations in rad/s

        Returns
        -------
        np.ndarray
        """
        if self._iwcut is None:
            return self.wave_freq
        else:
            return self.wave_freq[:self._iwcut]

    def set_wave_directions(self):
        """Frequency array of BEM computations in rad/s

        Returns
        -------
        np.ndarray
        """

        self.wave_dir = np.radians(np.linspace(self.min_wave_dir, self.max_wave_dir, self.nb_wave_dir, dtype=np.float))

    @property
    def wave_dir(self):
        """Wave direction angles array of BEM computations in radians

        Returns
        -------
        np.ndarray
            angles array in radians.
        """
        return self.wave_dir

    def append(self, body):

        """This function adds a body.

        Parameter
        ----------
        BodyDB : body
            Hydrodynamic body.
        """

        self.bodies.append(body)

    @property
    def nb_forces(self):

        """ This function computes the sum of the force modes for all bodies.
        Returns
        -------
        int
            Number of force modes for all bodies.
        """

        n = 0
        for body in self.bodies:
            n = n + int(body.Force_mask.sum())

        return n

    @property
    def nb_motions(self):

        """ This function computes the sum of the motion modes for all bodies.
        Returns
        -------
        int
            Number of motion modes for all bodies.
        """

        n = 0
        for body in self.bodies:
            n = n + int(body.Motion_mask.sum())

        return n