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
    Module to load a HDB5 file.
"""

import numpy as np
import h5py

from meshmagick.mesh import Mesh

from body_db_v2 import *

class HDB5reader():
    """
        Class for reading HDB5 file..
    """

    def __init__(self, pyHDB, hdb5_file):
        """ Constructor of the class NemohReader.

         Parameters
        ----------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        hdb5_file : string
            Path to the hdb5 file to load.
        """

        with h5py.File(hdb5_file, 'r') as reader:
            print ("")
            print list(reader.keys())

            # Version.
            self.read_version(reader, pyHDB)

            # Environment.
            self.read_environment(reader, pyHDB)

            # Discretization.
            self.read_discretization(reader, pyHDB)

            # Bodies.
            self.read_bodies(reader, pyHDB)

            # Wave drift coefficients.
            # set_wave_directions_Kochin

    def read_environment(self, reader, pyHDB):
        """This function reads the environmental data of the *.hdb5 file.

        Parameter
        ---------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        # Date.
        self.Creation_data_hdf5file = np.array(reader['CreationDate']) # Date of creation of the hdf5file.

        # Gravity acceleration.
        pyHDB.grav = np.array(reader['GravityAcc'])

        # Water density.
        pyHDB.rho_water = np.array(reader['WaterDensity'])

        # Normalisation length.
        pyHDB.normalization_length = np.array(reader['NormalizationLength'])

        # Water depth.
        pyHDB.depth = np.array(reader['WaterDepth'])

        # Number of bodies.
        pyHDB.nb_bodies = np.array(reader['NbBody'])

    def read_discretization(self, reader, pyHDB):
        """This function reads the discretization parameters of the *.hdb5 file.

        Parameter
        ---------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        discretization_path = "/Discretizations"

        # Frequency discretization.

        frequential_path = discretization_path + "/Frequency"

        pyHDB.nb_wave_freq = np.array(reader[frequential_path + "/NbFrequencies"])
        pyHDB.min_wave_freq = np.array(reader[frequential_path + "/MinFrequency"])
        pyHDB.max_wave_freq = np.array(reader[frequential_path + "/MaxFrequency"])

        print pyHDB.max_wave_freq

        pyHDB.set_wave_directions() # Definition of beta.

        # print pyHDB.nb_wave_freq
        # print pyHDB.min_wave_freq
        # print pyHDB.max_wave_freq
        # print pyHDB.wave_dir

        # Wave direction discretization.

        wave_direction_path = discretization_path + "/WaveDirections"

        pyHDB.nb_wave_dir = np.array(reader[wave_direction_path + "/NbWaveDirections"])
        pyHDB.min_wave_dir = np.radians(np.array(reader[wave_direction_path + "/MinAngle"]))
        pyHDB.max_wave_dir = np.radians(np.array(reader[wave_direction_path + "/MaxAngle"]))
        pyHDB.set_wave_frequencies() # Definition of omega.

        # Time sample.

        time_path = discretization_path + "/Time"

        pyHDB.nb_time_samples = np.array(reader[time_path + "/NbTimeSample"])
        final_time = np.array(reader[time_path + "/FinalTime"])
        pyHDB.dt = np.array(reader[time_path + "/TimeStep"])
        pyHDB.time = np.arange(start=0., stop=final_time + pyHDB.dt, step=pyHDB.dt) # Definition of time.

    def read_mesh(self, reader, mesh_path):

        """This function reads the mesh quantities of the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        mesh_path : string
            Path to the mesh folder.
        """

        # Mesh data.
        nb_vertices = np.array(reader[mesh_path + "/NbVertices"])
        vertices = np.array(reader[mesh_path + "/Vertices"])
        nb_faces = np.array(reader[mesh_path + "/NbFaces"])
        faces = np.array(reader[mesh_path + "/Faces"])
        mesh = Mesh(vertices, faces)

        # Verification of mesh information consistency
        assert nb_vertices == mesh.nb_vertices
        assert nb_faces == mesh.nb_faces

        return mesh

    def read_mode(self, reader, body, ForceOrMotion, body_modes_path):
        """This function reads the force and motion modes of the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        ForceOrMotion : int
            0 for Force, 1 for Motion.
        body_modes_path : string
            Path to body modes.
        """

        for iforce in range(0,6):
            if (ForceOrMotion == 0):  # Force.
                mode_path = body_modes_path + "/ForceModes/Mode_%u" % iforce
            else:  # Motion.
                mode_path = body_modes_path + "/MotionModes/Mode_%u" % iforce

            if (iforce >= 3):
                body.point = np.array(reader[mode_path + "/Point"])

    def read_mask(self, reader, body, mask_path):
        """This function reads the Force and Motion masks into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        mask_path : string, optional
            Path to the masks.
        """

        body.Motion_mask = np.array(reader[mask_path + "/MotionMask"])
        body.Force_mask = np.array(reader[mask_path + "/ForceMask"])

    def read_excitation(self, reader, pyHDB, body, excitation_path):

        """This function reads the diffraction and Froude-Krylov loads into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        body : BodyDB.
            Body.
        excitation_path : string, optional
            Path to excitation loads.
        """

        # Froude-Krylov loads;

        fk_path = excitation_path + "/FroudeKrylov"

        for idir in range(0, pyHDB.nb_wave_dir):

            wave_dir_path = fk_path + "/Angle_%u" % idir

            # Check of the wave direction.
            # print pyHDB.wave_dir
            # assert pyHDB.wave_dir[idir] == np.array(reader[wave_dir_path + "/Angle"])

    def read_bodies(self, reader, pyHDB):
        """This function reads the body data of the *.hdb5 file.

        Parameters
        ----------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        for ibody in xrange(pyHDB.nb_bodies):

            body_path = '/Bodies/Body_%u' % ibody

            # Index of the body.
            id = np.array(reader[body_path + "/ID"])

            # Mesh.
            mesh = self.read_mesh(reader, body_path + "/Mesh")

            # Body definition.
            body = BodyDB(id, pyHDB.nb_bodies, pyHDB.nb_wave_freq, pyHDB.nb_wave_dir, mesh)

            # Force modes.
            self.read_mode(reader, body, 0, body_path + "/Modes")

            # Motion modes.
            self.read_mode(reader, body, 1, body_path + "/Modes")

            # Masks.
            self.read_mask(reader, body, body_path + "/Mask")

            # Diffraction and Froude-Krylov loads.
            self.read_excitation(reader, pyHDB, body, body_path + "/Excitation")

            # Add body to pyHDB.
            pyHDB.append(body)

    def read_version(self, reader, pyHDB):
        """This function reads the version of the *.hdb5 file.

        Parameter
        ---------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        # Version.
        try:
            pyHDB.version = np.array(reader['Version'])
        except:
            pyHDB.version = 1.0













































