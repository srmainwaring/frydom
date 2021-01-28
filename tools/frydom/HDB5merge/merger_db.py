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

import frydom.HDB5tool.pyHDB as pyHDB

class Merger(object):
    """
        Class for merging two pyHDB of the same problem, only the wave freqency ranges are different.
    """

    def __init__(self, pyHDB_1, pyHDB_2):

        """
            Constructor of the class Merger.
        """

        # By definition, self._pyHDB_1 has the lowest frequency range and self._pyHDB_2 the highest one.
        if(pyHDB_1.max_wave_freq < pyHDB_2.min_wave_freq):
            self._pyHDB_1 = pyHDB_1
            self._pyHDB_2 = pyHDB_2
        elif(pyHDB_2.max_wave_freq < pyHDB_1.min_wave_freq):
            self._pyHDB_1 = pyHDB_2
            self._pyHDB_2 = pyHDB_1
        else: # Check that the two wave frequency ranges are disjoint.
            print("Wave frequency ranges are not disjoint.")
            print("This case is not taken into account by hdb5merge.")
            print("Range of the first *.hdb5: [%f , %f]" % (pyHDB_1.min_wave_freq, pyHDB_1.max_wave_freq))
            print("Range of the second *.hdb5: [%f , %f]" % (pyHDB_2.min_wave_freq, pyHDB_2.max_wave_freq))
            exit()

    def merge_version(self, pyHDB_out):

        """
            This method merges the version of the two pyHDB.
        """

        pyHDB_out.version = self._pyHDB_1.version_max

    def merge_environment(self, pyHDB_out):

        """
            This method merges the environmental data of the two pyHDB.
        """

        # Gravity acceleration.
        assert (self._pyHDB_1.grav == self._pyHDB_2.grav)
        pyHDB_out.grav = self._pyHDB_1.grav

        # Water density.
        assert (self._pyHDB_1.rho_water == self._pyHDB_2.rho_water)
        pyHDB_out.rho_water = self._pyHDB_1.rho_water

        # Normalisation length.
        assert (self._pyHDB_1.normalization_length == self._pyHDB_2.normalization_length)
        pyHDB_out.normalization_length = self._pyHDB_1.normalization_length

        # Water depth.
        assert (self._pyHDB_1.depth == self._pyHDB_2.depth)
        pyHDB_out.depth = self._pyHDB_1.depth

        # Number of bodies.
        assert (self._pyHDB_1.nb_bodies == self._pyHDB_2.nb_bodies)
        pyHDB_out.nb_bodies = self._pyHDB_1.nb_bodies

        # Solver.
        assert (self._pyHDB_1.solver == self._pyHDB_2.solver)
        pyHDB_out.solver = self._pyHDB_1.solver

    def merge(self):

        """
            This method merges two pyHDB and returns the merged pyHDB.
        """

        # Initialization of the merged pyHDB.
        pyHDB_out = pyHDB.pyHDB()

        # Version.
        self.merge_version(pyHDB_out)

        # Environnement.
        self.merge_environnement(pyHDB_out)

        return


