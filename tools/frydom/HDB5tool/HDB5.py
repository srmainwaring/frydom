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
import numpy as np

from meshmagick.mmio import write_OBJ

import frydom.HDB5tool.bem_reader as bem_reader
import frydom.HDB5tool.HDB5_reader as HDB5_reader
import frydom.HDB5tool.pyHDB as pyHDB
from frydom.HDB5tool.discretization_db import DiscretizationDB
import frydom.HDB5tool.plot_db as plot_db

class HDB5(object):

    """
        Class HDB5 for dealing with *.h5 files.
    """

    def __init__(self):

        """
            Constructor of the class HDB5.
        """

        # HDB.
        self._pyHDB = pyHDB.pyHDB()

        # Discretization parameters.
        self._discretization = DiscretizationDB()

        # Initialization parameter.
        self._is_initialized = False

        return

    @property
    def pyHDB(self):

        """This function returns the hdb.

        Returns
        -------
        BodyDB
        """

        return self._pyHDB

    @property
    def body(self):

        """This function returns all the bodies.

        Returns
        -------
        BodyDB
        """

        return self._pyHDB.bodies

    @property
    def discretization(self):

        """This function returns the parameters of the discretization.

        Returns
        -------
        DiscretizationDB
        """

        return self._discretization

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

        print("")
        print('========================')
        print('Reading Nemoh results...')
        print('========================')

        if nb_faces_by_wavelength is None:
            nb_faces_by_wavelength = 10

        # Reading *.cal.
        bem_reader.NemohReader(self._pyHDB,cal_file=nemoh_cal_file, test=True, nb_face_by_wave_length=nb_faces_by_wavelength)

        print('-------> Nemoh data successfully loaded from "%s"' % input_directory)
        print("")

    def _initialize(self):

        """This function updates the hydrodynamic database (computation of FK and diffraction loads, impulse response functions, interpolations, etc.)."""

        # Computing Froude-Krylov loads.
        self._pyHDB.Eval_Froude_Krylov_loads()

        # Initialization of the discretization object.
        self._discretization.initialize(self._pyHDB)

        # Time.
        self._pyHDB.time = self.discretization.time
        self._pyHDB.dt = self.discretization.delta_time
        self._pyHDB.nb_time_samples = self.discretization.nb_time_sample

        # Impule response functions for radiation damping.
        self._pyHDB.eval_impulse_response_function()

        # Infinite masses.
        self._pyHDB.eval_infinite_added_mass()

        # Impule response functions proportional to the forward speed without x-derivatives.
        self._pyHDB.eval_impulse_response_function_Ku()

        if(self._pyHDB._has_x_derivatives):

            # Impule response functions proportional to the forward speed with x-derivatives.
            self._pyHDB.eval_impulse_response_function_Ku_x_derivative()

            # Impule response functions proportional to the square of the forward speed.
            self._pyHDB.eval_impulse_response_function_Ku2()

        # Interpolations with respect to the wave directions and the wave frequencies.
        # self._pyHDB.interpolation(self.discretization)

        # Initialization done.
        self._is_initialized = True

    @property
    def wave_drift(self):

        """This function gives the wave drift data of the body.

        Returns
        -------
        WaveDriftDB
            Wave drift data of the body.
        """

        return self._pyHDB._wave_drift

    @property
    def wave_drift_force(self):

        """This function gives the wave drift data of the body.

        Returns
        -------
        WaveDriftDB
            Wave drift data of the body.
        """

        return self._pyHDB.Wave_drift_force

    @property
    def omega(self):
        """Frequency array of BEM computations in rad/s

        Returns
        -------
        np.ndarray
        """
        return self._pyHDB.omega

    @property
    def wave_dir(self):
        """Wave direction angles array of BEM computations in radians

        Returns
        -------
        np.ndarray
            angles array in radians.
        """
        return self._pyHDB.wave_dir

    def symmetry_HDB(self):

        """This function symmetrizes the HDB."""

        # Updating the wave directions.
        self._pyHDB._initialize_wave_dir()

    def Plot_Diffraction(self, ibody, iforce, iwave = 0):
        """This functions plots the diffraction loads."""

        # Data.
        data = self._pyHDB.bodies[ibody].Diffraction[iforce, :, iwave]

        # Wave direction.
        beta = self._pyHDB.wave_dir[iwave]

        # Plot.
        plot_db.plot_loads(data, self._pyHDB.wave_freq, 0, ibody, iforce, beta, False)

    def Plot_Diffraction_x_derivative(self, ibody, iforce, iwave = 0):
        """This functions plots the x-derivative of the diffraction loads."""

        # Data.
        data = self._pyHDB.bodies[ibody].Diffraction_x_derivative[iforce, :, iwave]

        # Wave direction.
        beta = self._pyHDB.wave_dir[iwave]

        # Plot.
        plot_db.plot_loads(data, self._pyHDB.wave_freq, 0, ibody, iforce, beta, True)

    def Plot_Froude_Krylov(self, ibody, iforce, iwave = 0, **kwargs):
        """This functions plots the Froude-Krylov loads."""

        # Data.
        data = self._pyHDB.bodies[ibody].Froude_Krylov[iforce, :, iwave]

        # Wave direction.
        beta = self._pyHDB.wave_dir[iwave]

        # Plots.
        plot_db.plot_loads(data, self._pyHDB.wave_freq, 1, ibody, iforce, beta, False, **kwargs)

    def Plot_Froude_Krylov_x_derivative(self, ibody, iforce, iwave = 0, **kwargs):
        """This functions plots the x-derivative of the Froude-Krylov loads."""

        # Data.
        data = self._pyHDB.bodies[ibody].Froude_Krylov_x_derivative[iforce, :, iwave]

        # Wave direction.
        beta = self._pyHDB.wave_dir[iwave]

        # Plots.
        plot_db.plot_loads(data, self._pyHDB.wave_freq, 1, ibody, iforce, beta, True, **kwargs)

    def Plot_Excitation(self, ibody, iforce, iwave = 0, **kwargs):
        """This functions plots the excitation loads."""

        # Data.
        data = self._pyHDB.bodies[ibody].Diffraction[iforce, :, iwave] + self._pyHDB.bodies[ibody].Froude_Krylov[iforce, :, iwave]

        # Wave direction.
        beta = self._pyHDB.wave_dir[iwave]

        # Plots.
        plot_db.plot_loads(data, self._pyHDB.wave_freq, 2, ibody, iforce, beta, False, **kwargs)

    def Plot_Excitation_x_derivative(self, ibody, iforce, iwave = 0, **kwargs):
        """This functions plots the x-derivative of the excitation loads."""

        # Data.
        data = self._pyHDB.bodies[ibody].Diffraction_x_derivative[iforce, :, iwave] + self._pyHDB.bodies[ibody].Froude_Krylov_x_derivative[iforce, :, iwave]

        # Wave direction.
        beta = self._pyHDB.wave_dir[iwave]

        # Plots.
        plot_db.plot_loads(data, self._pyHDB.wave_freq, 2, ibody, iforce, beta, True, **kwargs)

    def Plot_Radiation_coeff(self, ibody_force, iforce, ibody_motion, idof):
        """This functions plots the added mass and damping coefficients."""

        # Data.
        body_force = self._pyHDB.bodies[ibody_force]

        # If w_min is too far from w = 0, w = 0 is not displayed.
        display_zero_frequency = False
        if body_force.Zero_Added_mass is not None and self._pyHDB.wave_freq[0] < 1.:
            display_zero_frequency = True

        if display_zero_frequency is True:
            data = np.zeros((self._pyHDB.nb_wave_freq + 2, 2), dtype=np.float)  # 2 for added mass and damping coefficients, +2 for both the infinite and zero-frequency added mass.

            # Added mass.
            data[0, 0] = body_force.Zero_Added_mass[iforce, 6 * ibody_motion + idof]
            data[1:self._pyHDB.nb_wave_freq+1, 0] = body_force.Added_mass[iforce, 6 * ibody_motion + idof, :]
            data[self._pyHDB.nb_wave_freq+1, 0] = body_force.Inf_Added_mass[iforce, 6 * ibody_motion + idof]

            # Damping.
            data[0, 1] = 0.
            data[1:self._pyHDB.nb_wave_freq+1, 1] = body_force.Damping[iforce, 6 * ibody_motion + idof, :]

            # Wave frequency.
            w = np.zeros((self._pyHDB.wave_freq.shape[0] + 1))
            w[0] = 0
            w[1:] = self._pyHDB.wave_freq
        else:
            data = np.zeros((self._pyHDB.nb_wave_freq+1,2), dtype = np.float) # 2 for added mass and damping coefficients, +1 for the infinite-frequency added mass.

            # Added mass.
            data[0:self._pyHDB.nb_wave_freq, 0] = body_force.Added_mass[iforce, 6 * ibody_motion + idof, :]
            data[self._pyHDB.nb_wave_freq, 0] = body_force.Inf_Added_mass[iforce, 6 * ibody_motion + idof]

            # Damping.
            data[0:self._pyHDB.nb_wave_freq, 1] = body_force.Damping[iforce, 6 * ibody_motion + idof, :]

            # Wave frequency.
            w = self._pyHDB.wave_freq

        # Plots.
        plot_db.plot_AB(data, w, ibody_force, iforce, ibody_motion, idof, False)

    def Plot_Radiation_coeff_x_derivative(self, ibody_force, iforce, ibody_motion, idof):
        """This functions plots the x-derivative of the added mass and damping coefficients."""

        if(self._pyHDB._has_x_derivatives):

            # Data.
            body_force = self._pyHDB.bodies[ibody_force]
            data = np.zeros((self._pyHDB.nb_wave_freq + 2, 2), dtype=np.float)  # 2 for added mass and damping coefficients, +2 for both the infinite and zero-frequency added mass.

            # x-derivative of the added mass.
            data[0, 0] = body_force.Zero_Added_mass_x_derivative[iforce, 6 * ibody_motion + idof]
            data[1:self._pyHDB.nb_wave_freq+1, 0] = body_force.Added_mass_x_derivative[iforce, 6 * ibody_motion + idof, :]
            data[self._pyHDB.nb_wave_freq+1, 0] = body_force.Inf_Added_mass_x_derivative[iforce, 6 * ibody_motion + idof]

            # x-derivative of the Damping.
            data[0, 1] = 0.
            data[1:self._pyHDB.nb_wave_freq+1, 1] = body_force.Damping_x_derivative[iforce, 6 * ibody_motion + idof, :]

            # Wave frequency.
            w = np.zeros((self._pyHDB.wave_freq.shape[0] + 1))
            w[0] = 0
            w[1:] = self._pyHDB.wave_freq

            # Plots.
            plot_db.plot_AB(data, w, ibody_force, iforce, ibody_motion, idof, True)

    def Plot_IRF(self, ibody_force, iforce, ibody_motion, idof):
        """This function plots the impulse response functions without forward speed."""

        # Data.
        data = self._pyHDB.bodies[ibody_force].irf[iforce, 6 * ibody_motion + idof, :]

        # Time.
        time = self._pyHDB.time

        # Plots.
        plot_db.plot_irf(data, time, 0, ibody_force, iforce, ibody_motion, idof)

    def Plot_IRF_speed(self, ibody_force, iforce, ibody_motion, idof):
        """This function plots the impulse response functions with forward speed."""

        # Data.
        data = self._pyHDB.bodies[ibody_force].irf_ku[iforce, 6 * ibody_motion + idof , :]

        # Time.
        time = self._pyHDB.time

        # Plots.
        plot_db.plot_irf(data, time, 2, ibody_force, iforce, ibody_motion, idof)

        if(self._pyHDB._has_x_derivatives):

            if(self._pyHDB.bodies[ibody_force].irf_ku_x_derivative is not None):
                # Data.
                data = self._pyHDB.bodies[ibody_force].irf_ku_x_derivative[iforce, 6 * ibody_motion + idof, :]

                # Plots.
                plot_db.plot_irf(data, time, 1, ibody_force, iforce, ibody_motion, idof)

            if(self._pyHDB.bodies[ibody_force].irf_ku2 is not None):
                # Data.
                data = self._pyHDB.bodies[ibody_force].irf_ku2[iforce, 6 * ibody_motion + idof, :]

                # Plots.
                plot_db.plot_irf(data, time, 3, ibody_force, iforce, ibody_motion, idof)

    def Plot_Mesh(self, ibody = -1):
        """This function plots a mesh."""

        if(self._pyHDB.bodies[ibody].mesh is not None):

            # Data.
            mesh = self._pyHDB.bodies[ibody].mesh

            # From the body frame to the global frame.
            if(self._pyHDB.bodies[ibody].horizontal_position is not None):
                horizontal_position = self._pyHDB.bodies[ibody].horizontal_position
                mesh.rotate([0., 0., horizontal_position[2]])
                mesh.translate([horizontal_position[0], horizontal_position[1], 0.])

            # Plot.
            plot_db.Meshmagick_viewer(mesh)

    def Plot_Meshes(self):
        """This function plots all meshes."""

        if (self._pyHDB.bodies[0].mesh is not None): # If one mesh is present, other ones should also be.

            # Data.
            mesh = self._pyHDB.bodies[0].mesh
            if (self._pyHDB.bodies[0].horizontal_position is not None):
                horizontal_position = self._pyHDB.bodies[0].horizontal_position
                mesh.rotate([0., 0., horizontal_position[2]])
                mesh.translate([horizontal_position[0], horizontal_position[1], 0.])

            MultibodyMesh = mesh # Initialization by using the first body which always exists because they are several bodies.
            for id in range(1, self._pyHDB.nb_bodies): # Loop over all bodies except the first one.

                mesh = self._pyHDB.bodies[id].mesh

                # From the body frame to the global frame.
                if (self._pyHDB.bodies[id].horizontal_position is not None):
                    horizontal_position = self._pyHDB.bodies[id].horizontal_position
                    mesh.rotate([0., 0., horizontal_position[2]])
                    mesh.translate([horizontal_position[0], horizontal_position[1], 0.])

                # Merging.
                MultibodyMesh += mesh

            # Plot.
            plot_db.Meshmagick_viewer(MultibodyMesh)

    def Write_Mesh(self, ibody = -1):
        """This method writes a mesh."""

        if (self._pyHDB.bodies[ibody].mesh is not None):

            # Data.
            body = self._pyHDB.bodies[ibody]
            mesh = body.mesh

            # From the body frame to the global frame.
            if (self._pyHDB.bodies[ibody].horizontal_position is not None):
                horizontal_position = self._pyHDB.bodies[ibody].horizontal_position
                mesh.rotate([0., 0., horizontal_position[2]])
                mesh.translate([horizontal_position[0], horizontal_position[1], 0.])

            # Write.
            if(body.name is not None):
                write_OBJ(body.name + ".obj", mesh.vertices, mesh.faces)
            else:
                write_OBJ("Body_" + str(id + 1) + ".obj", mesh.vertices, mesh.faces)

    def Write_Meshes(self):
        """This method writes all meshes."""

        for ibody in range(0, self._pyHDB.nb_bodies):  # Loop over all bodies except the first one.

            if (self._pyHDB.bodies[ibody].mesh is not None):  # If one mesh is present, other ones should also be.

                # Data.
                body = self._pyHDB.bodies[ibody]
                mesh = body.mesh

                # From the body frame to the global frame.
                if (self._pyHDB.bodies[ibody].horizontal_position is not None):
                    horizontal_position = self._pyHDB.bodies[ibody].horizontal_position
                    mesh.rotate([0., 0., horizontal_position[2]])
                    mesh.translate([horizontal_position[0], horizontal_position[1], 0.])

                # Write.
                if (body.name is not None):
                    write_OBJ(body.name + ".obj", mesh.vertices, mesh.faces)
                else:
                    write_OBJ("Body_" + str(ibody + 1) + ".obj", mesh.vertices, mesh.faces)

    def Cutoff_scaling_IRF(self, tc, ibody_force, iforce, ibody_motion, idof, auto_apply=False):
        """This function applies a filter to the impule response functions without forward speed and plot the result.

        Parameters
        ----------
        float : tc.
            Cutting time.
        ibody_force : int.
            Index of the body where the radiation force is applied.
        int : i_force.
            Index of the index of the force of the current body.
        int : i_body_motion.
            Index of the body.
        int : i_dof.
            Index of the dof of the moving body.
        Bool : auto_apply, optional.
            Automatic application of the filtering, not if flase (default).
       """

        # Data.
        data = self._pyHDB.bodies[ibody_force].irf[iforce, 6 * ibody_motion + idof, :]

        # Time.
        time = self._pyHDB.time

        # Coeff.
        try:
            coeff = np.exp(-9.*time*time / (tc*tc))
        except:
            coeff = np.zeros(time.size)

        # Application of the filer.
        if auto_apply:
            bool = True
        else:
            # Plot.
            plot_db.plot_filering(data, time, 0, coeff, ibody_force, iforce, ibody_motion, idof)

            # input returns the empty string for "enter".
            yes = {'yes', 'y', 'ye', ''}
            no = {'no', 'n'}

            choice = input("Apply scaling (y/n) ? ").lower()
            if choice in yes:
                bool = True
            elif choice in no:
                bool = False
            else:
                sys.stdout.write("Please respond with 'yes' or 'no'")

        if bool:
            self._pyHDB.bodies[ibody_force].irf[iforce, 6 * ibody_motion + idof, :] *= coeff

    def Cutoff_scaling_IRF_speed(self, tc, ibody_force, iforce, ibody_motion, idof, auto_apply=False):
        """This function applies a filter to the impule response functions with forward speed and plot the result.

        Parameters
        ----------
        float : tc.
            Cutting time.
        ibody_force : int.
            Index of the body where the radiation force is applied.
        int : i_force.
            Index of the index of the force of the current body.
        int : i_body_motion.
            Index of the body.
        int : i_dof.
            Index of the dof of the moving body.
        Bool : auto_apply, optional.
            Automatic application of the filtering, not if flase (default).
       """

        # Data.
        data = self._pyHDB.bodies[ibody_force].irf_ku[iforce, 6 * ibody_motion + idof, :]

        # Time.
        time = self._pyHDB.time

        # Coeff.
        try:
            coeff = np.exp(-9.*time*time / (tc*tc))
        except:
            coeff = np.zeros(time.size)

        # Application of the filer.
        if auto_apply:
            bool = True
        else:
            # Plot.
            plot_db.plot_filering(data, time, 1, coeff, ibody_force, iforce, ibody_motion, idof)

            # input returns the empty string for "enter".
            yes = {'yes', 'y', 'ye', ''}
            no = {'no', 'n'}

            choice = input("Apply scaling (y/n) ? ").lower()
            if choice in yes:
                bool = True
            elif choice in no:
                bool = False
            else:
                sys.stdout.write("Please respond with 'yes' or 'no'")

        if bool:
            self._pyHDB.bodies[ibody_force].irf[iforce, 6 * ibody_motion + iforce, :] *= coeff

    def Update_radiation_mask(self):
        """This function asks the user to define the radiation coefficient which should be zeroed and update the radiation mask accordingly."""

        for ibody_force in range(0, self._pyHDB.nb_bodies):
            for ibody_motion in range(0, self._pyHDB.nb_bodies):

                # data.
                data = np.zeros((6, 6, self._pyHDB.nb_wave_freq), dtype=np.float)
                for iforce in range(0, 6):
                    for idof in range(0, 6):
                        for iw in range(0, self._pyHDB.nb_wave_freq):
                            data[iforce, idof, iw] = np.linalg.norm(self._pyHDB.bodies[ibody_force].Damping[iforce, 6 * ibody_motion + idof, iw]
                                        + 1j * self._pyHDB.wave_freq[iw] * (self._pyHDB.bodies[ibody_force].Added_mass[iforce, 6 * ibody_motion + idof, iw]
                                        - self._pyHDB.bodies[ibody_force].Inf_Added_mass[iforce, 6 * ibody_motion + idof]))

                # Plot.
                plot_db.plot_AB_array(data, self._pyHDB.wave_freq, ibody_force, ibody_motion, self._pyHDB, False)

    def Update_radiation_mask_x_derivatives(self):
        """This function asks the user to define the radiation coefficient which should be zeroed and update the radiation mask accordingly."""

        if (self._pyHDB._has_x_derivatives):

            for ibody_force in range(0, self._pyHDB.nb_bodies):
                for ibody_motion in range(0, self._pyHDB.nb_bodies):

                    # data.
                    data = np.zeros((6, 6, self._pyHDB.nb_wave_freq), dtype=np.float)
                    for iforce in range(0, 6):
                        for idof in range(0, 6):
                            for iw in range(0, self._pyHDB.nb_wave_freq):
                                data[iforce, idof, iw] = np.linalg.norm(self._pyHDB.bodies[ibody_force].Damping_x_derivative[iforce, 6 * ibody_motion + idof, iw]
                                            + 1j * self._pyHDB.wave_freq[iw] * (self._pyHDB.bodies[ibody_force].Added_mass_x_derivative[iforce, 6 * ibody_motion + idof, iw]
                                            - self._pyHDB.bodies[ibody_force].Inf_Added_mass_x_derivative[iforce, 6 * ibody_motion + idof]))

                    # Plot.
                    plot_db.plot_AB_array(data, self._pyHDB.wave_freq, ibody_force, ibody_motion, self._pyHDB, True)

        else:
            print("No x-derivative.")

    def Plot_irf_array(self):
        """This method plots the impulse response functions per body."""

        for ibody_force in range(0, self._pyHDB.nb_bodies):
            for ibody_motion in range(0, self._pyHDB.nb_bodies):

                # Time.
                time = self._pyHDB.time

                # Data.
                data = np.zeros((6, 6, self._pyHDB.nb_time_samples), dtype=np.float)
                for iforce in range(0, 6):
                    for idof in range(0, 6):
                        data[iforce, idof, :] = self._pyHDB.bodies[ibody_force].irf[iforce, 6 * ibody_motion + idof, :]
                # Plot.
                plot_db.plot_irf_array(data, time, ibody_force, ibody_motion, 0)

    def Plot_irf_speed_array(self):
        """This method plots the impulse response functions per body."""

        for ibody_force in range(0, self._pyHDB.nb_bodies):
            for ibody_motion in range(0, self._pyHDB.nb_bodies):

                # Time.
                time = self._pyHDB.time

                if(self._pyHDB.bodies[ibody_force].irf_ku is not None):
                    # Data.
                    data = np.zeros((6, 6, self._pyHDB.nb_time_samples), dtype=np.float)
                    for iforce in range(0, 6):
                        for idof in range(0, 6):
                            data[iforce, idof, :] = self._pyHDB.bodies[ibody_force].irf_ku[iforce, 6 * ibody_motion + idof, :]
                    # Plot.
                    plot_db.plot_irf_array(data, time, ibody_force, ibody_motion, 1)

                if (self._pyHDB._has_x_derivatives):

                    if(self._pyHDB.bodies[ibody_force].irf_ku_x_derivative is not None):
                        # Data.
                        data = np.zeros((6, 6, self._pyHDB.nb_time_samples), dtype=np.float)
                        for iforce in range(0, 6):
                            for idof in range(0, 6):
                                data[iforce, idof, :] = self._pyHDB.bodies[ibody_force].irf_ku_x_derivative[iforce, 6 * ibody_motion + idof, :]

                        # Plot.
                        plot_db.plot_irf_array(data, time, ibody_force, ibody_motion, 2)

                    if(self._pyHDB.bodies[ibody_force].irf_ku2 is not None):
                        # Data.
                        data = np.zeros((6, 6, self._pyHDB.nb_time_samples), dtype=np.float)
                        for iforce in range(0, 6):
                            for idof in range(0, 6):
                                data[iforce, idof, :] = self._pyHDB.bodies[ibody_force].irf_ku2[iforce,6 * ibody_motion + idof, :]

                        # Plot.
                        plot_db.plot_irf_array(data, time, ibody_force, ibody_motion, 3)

    def export_hdb5(self, output_file = None):
        """This function writes the hydrodynamic database into a *.hdb5 file.

        Parameter
        ---------
        output_file : string, optional
            Name of the hdb5 output file.
        """

        if not self._is_initialized:

            print('========================')
            print('Intialize HDB5 database...')
            print('========================')

            self._initialize()

        print('========================')
        print('Writing HDB5 database...')
        print('========================')

        if output_file is None:
            hdb5_file = os.path.abspath('frydom.hdb5')
        else:
            # Verifying that the output file has the extension .hdb5.
            root, ext = os.path.splitext(output_file)
            if not ext == '.hdb5':
                raise IOError('Please register the output file with a .hdb5 extension.')

            hdb5_file = output_file

            if not os.path.isabs(output_file):
                hdb5_file = os.path.abspath(hdb5_file)

        # Writing all the data from _pyHDB.
        try:
            self._pyHDB.write_hdb5(hdb5_file)
        except IOError:
            raise IOError('Problem in writing HDB5 file at location %s' % hdb5_file)

        print('')
        print('-------> "%s" has been written.' % hdb5_file)
        print('')

    def read_hdb5(self, input_file = None):
        """This function loads a *.hdb5 file.

        Parameter
        ---------
        input_file : string, optional
            Name of the hdb5 input file.
        """

        if(input_file is None):
            hdb5_file = os.path.abspath('frydom.hdb5')
        else:
            # Verifying that the output file has the extension .hdb5.
            root, ext = os.path.splitext(input_file)
            if not ext == '.hdb5':
                raise IOError('Please register the input file with a .hdb5 extension.')

            hdb5_file = input_file

            if not os.path.isabs(input_file):
                hdb5_file = os.path.abspath(hdb5_file)

        # Reading all the data from .hdb5 and creating a _pyHDB object.
        try:
            HDB5_reader.HDB5reader(self._pyHDB, hdb5_file)
        except IOError:
            raise IOError('Problem in reading HDB5 file at location %s' % hdb5_file)

        print('')
        print('-------> "%s" has been loaded.' % hdb5_file)
        print('')

    def write_info(self, input_file):
        """This method writes information about the hdb5."""

        self._pyHDB.write_info(input_file)
