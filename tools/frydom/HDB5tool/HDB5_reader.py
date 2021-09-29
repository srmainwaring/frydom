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

import frydom.HDB5tool.body_db as body_db
import frydom.HDB5tool.PoleResidue as PoleResidue
from frydom.HDB5tool.pyHDB import inf

class HDB5reader():
    """
        Class for reading HDB5 file.
    """

    def __init__(self, pyHDB, hdb5_file):
        """ Constructor of the class NemohReader.

         Parameters
        -----------
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        hdb5_file : string
            Path to the hdb5 file to load.
        """

        with h5py.File(hdb5_file, 'r') as reader:

            # Version.
            self.read_version(reader, pyHDB)

            # Date.
            self.Creation_data_hdf5file = np.array(reader['CreationDate'])  # Date of creation of the hdf5file.

            # Environment.
            self.read_environment(reader, pyHDB)

            # Discretization.
            if (pyHDB.version <= 2.0):
                self.read_discretization_v2(reader, pyHDB)
            else:
                self.read_discretization_v3(reader, pyHDB)

            # Symmetries.
            self.read_symmetries(reader, pyHDB)

            # Vector fitting.
            self.read_VF(reader, pyHDB, "/VectorFitting") # Always before HDBRreader for setting has_VF.

            # Bodies
            if(pyHDB.version == 1.0):
                HDB5reader_v1(reader, pyHDB)
            else:
                HDB5reader_v2(reader, pyHDB)

            # Wave field.
            self.read_wave_field(reader, pyHDB, "/WaveField")

            # Wave drift coefficients.
            self.read_wave_drift(reader, pyHDB, "/WaveDrift")

            # Expert numerical parameters.
            self.read_numerical_parameters(reader, pyHDB, "/ExpertParameters")

            # pyHDB parameters.
            pyHDB._has_infinite_added_mass = True
            pyHDB._has_froude_krylov = True

    def read_environment(self, reader, pyHDB):
        """This function reads the environmental data of the *.hdb5 file.

        Parameter
        ---------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        # Gravity acceleration.
        pyHDB.grav = np.array(reader['GravityAcc'])

        # Water density.
        pyHDB.rho_water = np.array(reader['WaterDensity'])

        # Normalisation length.
        pyHDB.normalization_length = np.array(reader['NormalizationLength'])

        # Water depth.
        pyHDB.depth = np.array(reader['WaterDepth'])
        if pyHDB.depth == 0.:  # Infinite water depth.
            pyHDB.depth = inf

        # Number of bodies.
        pyHDB.nb_bodies = np.array(reader['NbBody'])

        # Solver.
        try:
            pyHDB.solver = str(np.array(reader['Solver']))
        except:
            pyHDB.solver = "Nemoh"

        # Fix problem of convertion between bytes and string when using h5py.
        if(pyHDB.solver[0:2] == "b'" and pyHDB.solver[-1] == "'"):
            pyHDB.solver = pyHDB.solver[2:-1]

        # Commit hash.
        if (pyHDB.solver == "Helios"):
            try:
                pyHDB.commit_hash = str(np.array(reader['NormalizedCommitHash']))
                if (pyHDB.commit_hash[0:2] == "b'" and pyHDB.commit_hash[-1] == "'"):
                    pyHDB.commit_hash = pyHDB.commit_hash[2:-1]
            except:
                pass

    def read_discretization_v2(self, reader, pyHDB):
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
        pyHDB.set_wave_frequencies()  # Definition of omega.

        # Wave direction discretization.

        wave_direction_path = discretization_path + "/WaveDirections"

        pyHDB.nb_wave_dir = np.array(reader[wave_direction_path + "/NbWaveDirections"])
        pyHDB.min_wave_dir = np.array(reader[wave_direction_path + "/MinAngle"]) # Deg.
        pyHDB.max_wave_dir = np.array(reader[wave_direction_path + "/MaxAngle"]) # Deg.
        pyHDB.set_wave_directions() # Definition of beta in rad.

        # Time sample.

        time_path = discretization_path + "/Time"

        pyHDB.nb_time_samples = np.array(reader[time_path + "/NbTimeSample"])
        final_time = np.array(reader[time_path + "/FinalTime"])
        try:
            pyHDB.dt = np.array(reader[time_path + "/TimeStep"])
        except:
            pyHDB.dt = final_time / (pyHDB.nb_time_samples - 1)
        pyHDB.time = np.linspace(start=0., stop=final_time, num=pyHDB.nb_time_samples)

    def read_discretization_v3(self, reader, pyHDB):
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

        wave_frequency = np.array(reader[frequential_path])
        pyHDB.nb_wave_freq = wave_frequency.shape[0]
        pyHDB.min_wave_freq = wave_frequency[0]
        pyHDB.max_wave_freq = wave_frequency[-1]
        pyHDB.wave_freq = wave_frequency

        # Wave direction discretization.

        wave_direction_path = discretization_path + "/WaveDirection"

        wave_dir = np.array(reader[wave_direction_path])
        pyHDB.nb_wave_dir = wave_dir.shape[0]
        pyHDB.min_wave_dir = wave_dir[0] # Deg.
        pyHDB.max_wave_dir = wave_dir[-1] # Deg.
        pyHDB.set_wave_directions() # Definition of beta in rad.

        # Time sample.

        time_path = discretization_path + "/Time"

        time = np.array(reader[time_path])
        pyHDB.nb_time_samples = time.shape[0]
        final_time = time[-1]
        try:
            pyHDB.dt = np.array(reader[time_path + "/TimeStep"])
        except:
            if(pyHDB.nb_time_samples != 1):
                pyHDB.dt = final_time / (pyHDB.nb_time_samples - 1)
            else:
                pyHDB.dt = 0
        pyHDB.time = np.linspace(start=0., stop=final_time, num=pyHDB.nb_time_samples)

    def read_symmetries(self, reader, pyHDB):
        """This function reads the symmetry parameters of the *.hdb5 file.

        Parameter
        ---------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        symmetry_path = "/Symmetries"
        try:
            pyHDB.bottom_sym = np.array(reader[symmetry_path + "/Bottom"])
            pyHDB.xoz_sym = np.array(reader[symmetry_path + "/xOz"])
            pyHDB.yoz_sym = np.array(reader[symmetry_path + "/yOz"])
        except:
            pass

    def read_mesh(self, reader, pyHDB, mesh_path):

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

        # Only triangle meshes are considered in Helios but Meshmagick only considered quandrangular meshes.
        # The last column is duplicated.
        if (faces.shape[1] == 3):
            faces = np.insert(faces, -1, faces[:, 2], axis=1)

        # Meshmagick mesh.
        if(nb_vertices != 0 and nb_faces != 0):
            mesh = Mesh(vertices, faces)

        # Verification of mesh information consistency
        assert nb_vertices == mesh.nb_vertices
        assert nb_faces == mesh.nb_faces

        return mesh

    def read_mask(self, reader, body, mask_path):
        """This function reads the Force and Motion masks into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        mask_path : string
            Path to the masks.
        """

        # Motion mask.
        try:
            body.Motion_mask = np.array(reader[mask_path + "/MotionMask"])
        except:
            body.Motion_mask = np.ones(6, dtype = np.int)

        # Force mask.
        try:
            body.Force_mask = np.array(reader[mask_path + "/ForceMask"])
        except:
            body.Force_mask = np.ones(6, dtype = np.int)

    def read_hydrostatic(self, reader, body, hydrostatic_path):

        """This function reads the hydrostatic stiffness matrix into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        hydrostatic_path : string
            Path to hydrostatic stiffness matrix.
        """

        try:
            reader[hydrostatic_path + "/StiffnessMatrix"]
            body.activate_hydrostatic()
            body.hydrostatic.matrix = np.array(reader[hydrostatic_path + "/StiffnessMatrix"])
        except:
            pass

    def read_wave_field(self, reader, pyHDB, wave_field_path):
        """This function reads the wave field data of the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        wave_field_path : string, optional
            Path to wave field data.
        """

        try:
            reader[wave_field_path]
            pyHDB.has_wave_field = True

        except:
            pass

    def read_kochin(self, reader, pyHDB, kochin_path):
        """"This method reads the Kochin functions and their derivatives of the *. hdb5 file."""

        # Angular discretization.
        pyHDB.min_angle_kochin = 0.
        pyHDB.max_angle_kochin = 360.
        pyHDB.nb_angle_kochin = int((360. / pyHDB.kochin_step) + 1.)
        pyHDB.angle_kochin = np.radians(np.linspace(pyHDB.min_angle_kochin, pyHDB.max_angle_kochin, pyHDB.nb_angle_kochin, dtype=np.float))

        # Wave directions for diffraction Kochin functions.
        # Do not use pyHDB.set_wave_directions_Kochin() because in case of symmetry, the wave directions for the hdb is
        # different from the wave direction for the Kochin functions (which are not symmetrised).
        pyHDB.nb_dir_kochin = len(reader[kochin_path + "/Diffraction"].keys())
        pyHDB.min_dir_kochin = np.array(reader[kochin_path + "/Diffraction/Angle_0/Angle"]) # deg.
        pyHDB.max_dir_kochin = np.array(reader[kochin_path + "/Diffraction/Angle_"+str(pyHDB.nb_dir_kochin - 1)+"/Angle"]) # deg.
        pyHDB.wave_dir_kochin = np.radians(np.linspace(pyHDB.min_dir_kochin, pyHDB.max_dir_kochin, pyHDB.nb_dir_kochin, dtype=np.float))

        # Parameters.
        ntheta = pyHDB.nb_angle_kochin
        nw = pyHDB.nb_wave_freq
        nbeta = pyHDB.nb_dir_kochin
        nbodies = pyHDB.nb_bodies

        # Diffraction Kochin functions and their derivatives.
        pyHDB.kochin_diffraction = np.zeros((nbeta, nw, ntheta), dtype=np.complex)
        if (pyHDB.solver == "Helios"):  # No derivative with Nemoh.
            pyHDB.kochin_diffraction_derivative = np.zeros((nbeta, nw, ntheta), dtype=np.complex)
        for iwave in range(nbeta):

            # Real part - Function.
            diffraction_function_real = np.array(reader[kochin_path + "/Diffraction/Angle_"+str(iwave)+"/Function/RealPart"])
            assert(diffraction_function_real.shape[0] == ntheta)
            assert(diffraction_function_real.shape[1] == nw)

            # Imaginary part - Function.
            diffraction_function_imag = np.array(reader[kochin_path + "/Diffraction/Angle_" + str(iwave) + "/Function/ImagPart"])
            assert(diffraction_function_imag.shape[0] == ntheta)
            assert(diffraction_function_imag.shape[1] == nw)

            if (pyHDB.solver == "Helios"):  # No derivative with Nemoh.
                # Real part - Derivative.
                diffraction_derivative_real = np.array(reader[kochin_path + "/Diffraction/Angle_" + str(iwave) + "/Derivative/RealPart"])
                assert(diffraction_derivative_real.shape[0] == ntheta)
                assert(diffraction_derivative_real.shape[1] == nw)

                # Imaginary part - Derivative.
                diffraction_derivative_imag = np.array(reader[kochin_path + "/Diffraction/Angle_" + str(iwave) + "/Derivative/ImagPart"])
                assert(diffraction_derivative_imag.shape[0] == ntheta)
                assert(diffraction_derivative_imag.shape[1] == nw)

            # Setting.
            for ifreq in range(nw):
                pyHDB.kochin_diffraction[iwave, ifreq, :] = diffraction_function_real[:,ifreq] + 1j * diffraction_function_imag[:, ifreq]
                if (pyHDB.solver == "Helios"):  # No derivative with Nemoh.
                    pyHDB.kochin_diffraction_derivative[iwave, ifreq, :] = diffraction_derivative_real[:,ifreq] + 1j * diffraction_derivative_imag[:, ifreq]

        # Radiation Kochin functions and their derivatives.
        pyHDB.kochin_radiation = np.zeros((6 * nbodies, nw, ntheta), dtype=np.complex)
        if (pyHDB.solver == "Helios"):  # No derivative with Nemoh.
            pyHDB.kochin_radiation_derivative = np.zeros((6*nbodies, nw, ntheta), dtype=np.complex)
        for body in pyHDB.bodies:
            for imotion in range(0, 6):

                # Real part - Function.
                radiation_function_real = np.array(reader[kochin_path + "/Radiation/Body_"+str(body.i_body)+"/DOF_" + str(imotion) + "/Function/RealPart"])
                assert (radiation_function_real.shape[0] == ntheta)
                assert (radiation_function_real.shape[1] == nw)

                # Imaginary part - Function.
                radiation_function_imag = np.array(reader[kochin_path + "/Radiation/Body_"+str(body.i_body)+"/DOF_" + str(imotion) + "/Function/ImagPart"])
                assert (radiation_function_imag.shape[0] == ntheta)
                assert (radiation_function_imag.shape[1] == nw)

                if (pyHDB.solver == "Helios"): # No derivative with Nemoh.
                    # Real part - Derivative.
                    radiation_derivative_real = np.array(reader[kochin_path + "/Radiation/Body_"+str(body.i_body)+"/DOF_" + str(imotion) + "/Derivative/RealPart"])
                    assert (radiation_derivative_real.shape[0] == ntheta)
                    assert (radiation_derivative_real.shape[1] == nw)

                    # Imaginary part - Derivative.
                    radiation_derivative_imag = np.array(reader[kochin_path + "/Radiation/Body_"+str(body.i_body)+"/DOF_" + str(imotion) + "/Derivative/ImagPart"])
                    assert (radiation_derivative_imag.shape[0] == ntheta)
                    assert (radiation_derivative_imag.shape[1] == nw)

                # Setting.
                for ifreq in range(nw):
                    pyHDB.kochin_radiation[6 * body.i_body + imotion, ifreq, :] = radiation_function_real[:, ifreq] + 1j * radiation_function_imag[:, ifreq]
                    if (pyHDB.solver == "Helios"): # No derivative with Nemoh.
                        pyHDB.kochin_radiation_derivative[6 * body.i_body + imotion, ifreq, :] = radiation_derivative_real[:, ifreq] + 1j * radiation_derivative_imag[:, ifreq]

        pyHDB.has_kochin = True

    def read_wave_drift(self, reader, pyHDB, wave_drift_path):
        """This function reads the wave drift loads of the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        wave_drift_path : string, optional
            Path to wave drift loads.
        """

        try:
            reader[wave_drift_path]
            pyHDB.Wave_drift_force = np.zeros((6, pyHDB.nb_wave_freq, pyHDB.nb_wave_dir), dtype=np.float)

            # sym_x.
            if(int(np.array(reader[wave_drift_path + "/sym_x"])) == 0):
                pyHDB.sym_x = False
            else:
                pyHDB.sym_x = True

            # sym_y
            if (int(np.array(reader[wave_drift_path + "/sym_y"])) == 0):
                pyHDB.sym_y = False
            else:
                pyHDB.sym_y = True

            # Kochin function angular step.
            try:
                pyHDB.kochin_step = np.array(reader[wave_drift_path + "/KochinStep"])
            except:
                try:
                    pyHDB.kochin_step = np.array(reader[wave_drift_path + "/Kochin/KochinStep"])
                except:
                    pass

            # Kochin functions and their derivatives.
            try:
                self.read_kochin(reader, pyHDB, wave_drift_path + "/Kochin")
            except:
                pass

            # Modes.
            for mode in ["/surge", "/sway", "/heave", "/roll", "/pitch", "/yaw"]:
                try:
                    reader[wave_drift_path + mode]

                    # Loop over the wave directions.
                    for ibeta in range(0, pyHDB.nb_wave_dir):

                        # Path.
                        if(pyHDB.version <= 2.0):
                            heading_path = wave_drift_path + mode + "/heading_%u" % ibeta
                        else:
                            heading_path = wave_drift_path + mode + "/angle_%u" % ibeta

                        # Check wave direction.
                        if(pyHDB.version <= 2.0):
                            assert(abs(pyHDB.wave_dir[ibeta] - np.array(reader[heading_path + "/heading"])) < pow(10,-5))
                        else:
                            assert (abs(pyHDB.wave_dir[ibeta] - np.array(reader[heading_path + "/angle"]) * np.pi / 180.) < pow(10, -5))

                        # Wave drift coefficients.
                        if(mode == "/surge"):
                            pyHDB.Wave_drift_force[0, :, ibeta] = np.array(list(reader[heading_path + "/data"]))
                        elif(mode == "/sway"):
                            pyHDB.Wave_drift_force[1, :, ibeta] = np.array(list(reader[heading_path + "/data"]))
                        elif(mode == "/heave"):
                            pyHDB.Wave_drift_force[2, :, ibeta] = np.array(list(reader[heading_path + "/data"]))
                        elif(mode == "/roll"):
                            pyHDB.Wave_drift_force[3, :, ibeta] = np.array(list(reader[heading_path + "/data"]))
                        elif(mode == "/pitch"):
                            pyHDB.Wave_drift_force[4, :, ibeta] = np.array(list(reader[heading_path + "/data"]))
                        else: # Yaw.
                            pyHDB.Wave_drift_force[5, :, ibeta] = np.array(list(reader[heading_path + "/data"]))
                except:
                    pass
            pyHDB.has_Drift = True
        except:
            pass

    def read_VF(self, reader, pyHDB, VF_path):
        """This function reads the vector fitting parameters of the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        VF_path : string, optional
            Path to VF parameters.
        """

        try:
            reader[VF_path]
            pyHDB.max_order = np.array(reader[VF_path + "/MaxOrder"])
            pyHDB.relaxed = np.array(reader[VF_path + "/Relaxed"])
            pyHDB.tolerance = np.array(reader[VF_path + "/Tolerance"])
            pyHDB.has_VF = True

        except:
            pass

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

    def read_numerical_parameters(self, reader, pyHDB, num_param_path):
        """This methid reads the expert numerical parameters of the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        num_param_path : string, optional
            Path to expert numerical parameters.
        """

        try:
            reader[num_param_path]
            pyHDB.surface_integration_order = np.array(reader[num_param_path + "/SurfaceIntegrationOrder"])
            pyHDB.green_function = str(np.array(reader[num_param_path + "/GreenFunction"]))

            # Fix problem of convertion between bytes and string when using h5py.
            if (pyHDB.green_function[0:2] == "b'" and pyHDB.green_function[-1] == "'"):
                pyHDB.green_function = pyHDB.green_function[2:-1]

            pyHDB.crmax = np.array(reader[num_param_path + "/Crmax"])
            pyHDB.wave_reference_point_x = np.array(reader[num_param_path + "/WaveReferencePoint/x"])
            pyHDB.wave_reference_point_y = np.array(reader[num_param_path + "/WaveReferencePoint/y"])
            pyHDB.has_expert_parameters = True

        except:
            pass

class HDB5reader_v2(HDB5reader):
    """
        Class for reading HDB5 file of version 2.
    """

    def __init__(self, reader, pyHDB):
        """ HDB5 reader when version = 2.

         Parameters
        -----------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        # Bodies.
        self.read_bodies(reader, pyHDB)

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
        excitation_path : string
            Path to excitation loads.
        """

        # Test of the presence of the x-derivative of the excitation loads.
        try:
            reader[excitation_path + "/FroudeKrylovXDerivative/Angle_0/RealCoeffs"] # Read for cheking if the folder is present or not.
            pyHDB._has_x_derivatives = True
            nw = pyHDB.nb_wave_freq
            nbeta = pyHDB.nb_wave_dir
            body.Froude_Krylov_x_derivative = np.zeros((6, nw, nbeta), dtype=np.complex)
            body.Diffraction_x_derivative = np.zeros((6, nw, nbeta), dtype=np.complex)
            body.Added_mass_x_derivative = np.zeros((6, 6 * pyHDB.nb_bodies, nw), dtype=np.float)
            body.Inf_Added_mass_x_derivative = np.zeros((6, 6 * pyHDB.nb_bodies), dtype=np.float)
            body.Zero_Added_mass_x_derivative = np.zeros((6, 6 * pyHDB.nb_bodies), dtype=np.float)
            body.Damping_x_derivative = np.zeros((6, 6 * pyHDB.nb_bodies, nw), dtype=np.float)
        except:
            pass

        # Froude-Krylov loads.

        fk_path = excitation_path + "/FroudeKrylov"

        for idir in range(0, pyHDB.nb_wave_dir):
            wave_dir_path = fk_path + "/Angle_%u" % idir

            # Check of the wave direction.
            assert(abs(pyHDB.wave_dir[idir] - np.radians(np.array(reader[wave_dir_path + "/Angle"]))) < pow(10,-5))

            # Real parts.
            body.Froude_Krylov[:, :, idir].real = np.array(reader[wave_dir_path + "/RealCoeffs"])

            # Imaginary parts.
            body.Froude_Krylov[:, :, idir].imag = np.array(reader[wave_dir_path + "/ImagCoeffs"])

        # x-derivative of the Froude-Krylov loads.
        if(pyHDB._has_x_derivatives):
            fk_x_derivative_path = excitation_path + "/FroudeKrylovXDerivative"
            for idir in range(0, pyHDB.nb_wave_dir):
                wave_dir_path = fk_x_derivative_path + "/Angle_%u" % idir

                # Check of the wave direction.
                assert (abs(pyHDB.wave_dir[idir] - np.radians(np.array(reader[wave_dir_path + "/Angle"]))) < pow(10, -5))

                # Real parts.
                body.Froude_Krylov_x_derivative[:, :, idir].real = np.array(reader[wave_dir_path + "/RealCoeffs"])

                # Imaginary parts.
                body.Froude_Krylov_x_derivative[:, :, idir].imag = np.array(reader[wave_dir_path + "/ImagCoeffs"])

        # Diffraction loads.

        diffraction_path = excitation_path + "/Diffraction"

        for idir in range(0, pyHDB.nb_wave_dir):
            wave_dir_path = diffraction_path + "/Angle_%u" % idir

            # Check of the wave direction.
            assert(abs(pyHDB.wave_dir[idir] - np.radians(np.array(reader[wave_dir_path + "/Angle"]))) < pow(10, -5))

            # Real parts.
            body.Diffraction[:, :, idir].real = np.array(reader[wave_dir_path + "/RealCoeffs"])

            # Imaginary parts.
            body.Diffraction[:, :, idir].imag = np.array(reader[wave_dir_path + "/ImagCoeffs"])

        # x-derivative of the diffraction loads.
        if (pyHDB._has_x_derivatives):
            diffraction_x_derivative_path = excitation_path + "/DiffractionXDerivative"
            for idir in range(0, pyHDB.nb_wave_dir):
                wave_dir_path = diffraction_x_derivative_path + "/Angle_%u" % idir

                # Check of the wave direction.
                assert (abs(pyHDB.wave_dir[idir] - np.radians(np.array(reader[wave_dir_path + "/Angle"]))) < pow(10, -5))

                # Real parts.
                body.Diffraction_x_derivative[:, :, idir].real = np.array(reader[wave_dir_path + "/RealCoeffs"])

                # Imaginary parts.
                body.Diffraction_x_derivative[:, :, idir].imag = np.array(reader[wave_dir_path + "/ImagCoeffs"])

    def read_radiation(self, reader, pyHDB, body, radiation_path):

        """This function reads the added mass and damping coefficients and the impulse response functions with and without forward speed of the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        body : BodyDB.
            Body.
        radiation_path : string
            Path to radiation loads.
        """

        # Initializations.
        body.Inf_Added_mass = np.zeros((6, 6 * pyHDB.nb_bodies), dtype=np.float)
        try:
            reader[radiation_path + "/BodyMotion_0/ZeroFreqAddedMass"] # Read for cheking if the folder is present or not.
            body.Zero_Added_mass = np.zeros((6, 6 * pyHDB.nb_bodies), dtype=np.float)
        except:
            pass

        try:
            reader[radiation_path + "/BodyMotion_0/ImpulseResponseFunctionK/DOF_0"] # Read for cheking if the folder is present or not.
            body.irf = np.zeros((6, 6 * pyHDB.nb_bodies, pyHDB.nb_time_samples), dtype=np.float)
            body.irf_ku = np.zeros((6, 6 * pyHDB.nb_bodies, pyHDB.nb_time_samples), dtype=np.float)
        except:
            pass

        try:
            reader[radiation_path + "/BodyMotion_0/ImpulseResponseFunctionKUXDerivative/DOF_0"] # Read for cheking if the folder is present or not.
            body.irf_ku_x_derivative = np.zeros((6, 6 * pyHDB.nb_bodies, pyHDB.nb_time_samples), dtype=np.float)
            body.irf_ku2 = np.zeros((6, 6 * pyHDB.nb_bodies, pyHDB.nb_time_samples), dtype=np.float)
        except:
            pass

        if(pyHDB.has_VF):
            body.poles_residues = []

        for j in range(pyHDB.nb_bodies): # Body motion.

            # Paths.
            radiation_body_motion_path = radiation_path + "/BodyMotion_%u" % j
            added_mass_path = radiation_body_motion_path + "/AddedMass"
            added_mass_x_derivative_path = radiation_body_motion_path + "/AddedMassXDerivative"
            radiation_damping_path = radiation_body_motion_path + "/RadiationDamping"
            radiation_damping_x_derivative_path = radiation_body_motion_path + "/RadiationDampingXDerivative"
            irf_path = radiation_body_motion_path + "/ImpulseResponseFunctionK"
            irf_ku_path = radiation_body_motion_path + "/ImpulseResponseFunctionKU"
            irf_ku_x_derivative_path = radiation_body_motion_path + "/ImpulseResponseFunctionKUXDerivative"
            irf_ku2_path = radiation_body_motion_path + "/ImpulseResponseFunctionKU2"
            modal_path = radiation_body_motion_path + "/Modal"

            # Infinite-frequency added mass.
            body.Inf_Added_mass[:, 6 * j:6 * (j + 1)] = np.array(reader[radiation_body_motion_path + "/InfiniteAddedMass"])

            # x-derivative of the infinite-frequency added mass.
            if (pyHDB._has_x_derivatives):
                body.Inf_Added_mass_x_derivative[:, 6 * j:6 * (j + 1)] = np.array(reader[radiation_body_motion_path + "/InfiniteAddedMassXDerivative"])

            # Zero-frequency added mass.
            try:
                body.Zero_Added_mass[:, 6 * j:6 * (j + 1)] = np.array(reader[radiation_body_motion_path + "/ZeroFreqAddedMass"])
            except:
                pass

            # x-derivative of the zero-frequency added mass.
            if (pyHDB._has_x_derivatives):
                try:
                    body.Zero_Added_mass_x_derivative[:, 6 * j:6 * (j + 1)] = np.array(reader[radiation_body_motion_path + "/ZeroFreqAddedMassXDerivative"])
                except:
                    pass

            # Radiation mask.
            try:
                body.Radiation_mask[:, 6 * j:6 * (j + 1)] = np.array(reader[radiation_body_motion_path + "/RadiationMask"])
            except:
                pass

            for imotion in range(0, 6):

                # Added mass.
                body.Added_mass[:, 6 * j + imotion, :] = np.array(reader[added_mass_path + "/DOF_%u" % imotion])

                # x-derivative of the added mass.
                if (pyHDB._has_x_derivatives):
                    body.Added_mass_x_derivative[:, 6 * j + imotion, :] = np.array(reader[added_mass_x_derivative_path + "/DOF_%u" % imotion])

                # Damping.
                body.Damping[:, 6 * j + imotion, :] = np.array(reader[radiation_damping_path + "/DOF_%u" % imotion])

                # x-derivative of the damping.
                if (pyHDB._has_x_derivatives):
                    body.Damping_x_derivative[:, 6 * j + imotion, :] = np.array(reader[radiation_damping_x_derivative_path + "/DOF_%u" % imotion])

                # Impulse response functions without forward speed.
                if(body.irf is not None):
                    body.irf[:, 6 * j + imotion, :] = np.array(reader[irf_path + "/DOF_%u" % imotion])

                # Impulse response functions proportional to the forward speed without x-derivatives.
                if(body.irf_ku is not None):
                    body.irf_ku[:, 6 * j + imotion, :] = np.array(reader[irf_ku_path + "/DOF_%u" % imotion])

                # Impulse response functions proportional to the forward speed with x-derivatives.
                if (body.irf_ku_x_derivative is not None):
                    body.irf_ku_x_derivative[:, 6 * j + imotion, :] = np.array(reader[irf_ku_x_derivative_path + "/DOF_%u" % imotion])

                # Impulse response functions proportional to the square of the forward speed.
                if (body.irf_ku2 is not None):
                    body.irf_ku2[:, 6 * j + imotion, :] = np.array(reader[irf_ku2_path + "/DOF_%u" % imotion])

                # Poles and residues.
                if(pyHDB.has_VF):
                    for iforce in range(0, 6):
                        modal_coef_path = modal_path + "/DOF_%u/FORCE_%u" % (imotion, iforce)
                        PR = PoleResidue.PoleResidue()

                        # Real poles and residues.
                        try:
                            real_poles = np.array(reader[modal_coef_path + "/RealPoles"])
                            real_residues = np.array(reader[modal_coef_path + "/RealResidues"])
                            PR.add_real_pole_residue(real_poles, real_residues)
                        except:
                            pass

                        # Complex poles and residues.
                        try:

                            # Poles.
                            cc_poles_path = modal_coef_path + "/ComplexPoles"
                            cc_poles_Re = np.array(reader[cc_poles_path + "/RealCoeff"])
                            cc_poles_Im = np.array(reader[cc_poles_path + "/ImagCoeff"])
                            cc_poles = cc_poles_Re + 1j * cc_poles_Im

                            # Residues.
                            cc_residues_path = modal_coef_path + "/ComplexResidues"
                            cc_residues_Re = np.array(reader[cc_residues_path + "/RealCoeff"])
                            cc_residues_Im = np.array(reader[cc_residues_path + "/ImagCoeff"])
                            cc_residues = cc_residues_Re + 1j * cc_residues_Im

                            PR.add_cc_pole_residue(cc_poles, cc_residues)
                        except:
                            pass

                        body.poles_residues.append(PR)

    def read_mass_matrix(self, reader, body, inertia_path):

        """This function reads the mass matrix into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        inertia_path : string
            Path to inertia matrix.
        """

        try:
            reader[inertia_path + "/InertiaMatrix"]
            body.activate_inertia()
            body.inertia.matrix = np.array(reader[inertia_path + "/InertiaMatrix"])
        except:
            pass

    def read_mooring_matrix(self, reader, body, mooring_path):

        """This function reads the mooring matrix into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        mooring_path : string
            Path to mooring matrix.
        """

        try:
            reader[mooring_path + "/MooringMatrix"]
            body.activate_mooring()
            body.mooring = np.array(reader[mooring_path + "/MooringMatrix"])
        except:
            pass

    def read_extra_linear_damping_matrix(self, reader, body, extra_linear_damping_path):

        """This function reads the extra linear damping matrix into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        body : BodyDB.
            Body.
        extra_linear_damping_path : string
            Path to extra linear damping matrix.
        """

        try:
            reader[extra_linear_damping_path + "/DampingMatrix"]
            body.activate_extra_damping()
            body.extra_damping = np.array(reader[extra_linear_damping_path + "/DampingMatrix"])
        except:
            pass


    def read_RAO(self, reader, pyHDB, body, RAO_path):

        """This function reads the RAO into the *.hdb5 file.

        Parameters
        ----------
        reader : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        body : BodyDB.
            Body.
        excitation_path : string
            Path to excitation loads.
        """

        # Definition.
        body.RAO = np.zeros((6, pyHDB.nb_wave_freq, pyHDB.nb_wave_dir), dtype=np.complex)

        try:

            for idir in range(0, pyHDB.nb_wave_dir):
                wave_dir_path = RAO_path + "/Angle_%u" % idir

                # Check of the wave direction.
                assert(abs(pyHDB.wave_dir[idir] - np.radians(np.array(reader[wave_dir_path + "/Angle"])) < pow(10,-5)))

                # Amplitude.
                Abs_RAO = np.array(reader[wave_dir_path + "/Amplitude"])

                # Phase.
                if reader[wave_dir_path + "/Phase"].attrs['Unit'] == 'rad':
                    Phase_RAO = np.array(reader[wave_dir_path + "/Phase"])
                else:
                    Phase_RAO = np.radians(np.array(reader[wave_dir_path + "/Phase"])) # Conversion to rad.

                # RAO.
                body.RAO[:, :, idir] = Abs_RAO * np.exp(1j * Phase_RAO)
                pyHDB.has_RAO = True  # Written for each body but it does not matter.

        except:
            pass

    def read_bodies(self, reader, pyHDB):
        """This function reads the body data of the *.hdb5 file.

        Parameters
        ----------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        for ibody in range(0, pyHDB.nb_bodies):
            body_path = '/Bodies/Body_%u' % ibody

            # Index of the body.
            id = np.array(reader[body_path + "/ID"])
            assert ibody == id

            # Mesh.
            read_mesh = False
            try:
                mesh = self.read_mesh(reader, pyHDB, body_path + "/Mesh")
                read_mesh = True
            except:
                pass

            # Body definition.
            if(read_mesh):
                body = body_db.BodyDB(id, pyHDB.nb_bodies, pyHDB.nb_wave_freq, pyHDB.nb_wave_dir, mesh)
            else:
                body = body_db.BodyDB(id, pyHDB.nb_bodies, pyHDB.nb_wave_freq, pyHDB.nb_wave_dir)

            # Body name (body mesh name until version 2).
            try:
                body.name = str(np.array(reader[body_path + "/BodyName"]))

                # Fix problem of convertion between bytes and string when using h5py.
                if (body.name[0:2] == "b'" and body.name[-1] == "'"):
                    body.name = body.name[2:-1]
            except:
                pass

            # Horizontal position in world.
            try:
                x = np.array(reader[body_path + "/HorizontalPosition/x"])  # m.
                y = np.array(reader[body_path + "/HorizontalPosition/y"])  # m.
                psi = np.array(reader[body_path + "/HorizontalPosition/psi"])  # deg.
                body.horizontal_position = np.zeros(3)
                body.horizontal_position[0] = x  # m.
                body.horizontal_position[1] = y  # m.
                body.horizontal_position[2] = psi  # deg.
            except:
                pass

            # Computation point in body frame.
            try:
                body.computation_point = np.array(reader[body_path + "/BodyPosition"])
            except:
                try:
                    body.computation_point = np.array(reader[body_path + "/ComputationPoint"])
                except:
                    pass

            # Masks.
            self.read_mask(reader, body, body_path + "/Mask")

            # Diffraction and Froude-Krylov loads.
            self.read_excitation(reader, pyHDB, body, body_path + "/Excitation") # Must be before read_radiation.

            # Added mass and damping coefficients, impulse response functions and poles and residues of the VF.
            self.read_radiation(reader, pyHDB, body, body_path + "/Radiation")

            # Hydrostatics.
            self.read_hydrostatic(reader, body, body_path + "/Hydrostatic")

            # Mass matrix.
            self.read_mass_matrix(reader, body, body_path + "/Inertia")

            # Mooring matrix.
            self.read_mooring_matrix(reader, body, body_path + "/Mooring")

            # Extra linear damping matrix.
            self.read_extra_linear_damping_matrix(reader, body, body_path + "/LinearDamping")

            # RAO.
            self.read_RAO(reader, pyHDB, body, body_path + "/RAO")

            # Add body to pyHDB.
            pyHDB.append(body)

class HDB5reader_v1(HDB5reader):
    """
        Class for reading HDB5 file of version 1.
    """

    def __init__(self, reader, pyHDB):
        """ HDB5 reader when version = 2.

         Parameters
        -----------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        # Bodies.
        self.read_bodies(reader, pyHDB)

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
        excitation_path : string
            Path to excitation loads.
        """

        # Froude-Krylov loads;

        fk_path = excitation_path + "/FroudeKrylov"

        for idir in range(0, pyHDB.nb_wave_dir):
            wave_dir_path = fk_path + "/Angle_%u" % idir

            # Check of the wave direction.
            assert(abs(pyHDB.wave_dir[idir] - np.radians(np.array(reader[wave_dir_path + "/Angle"]))) < pow(10,-5))

            # Real parts.
            data = np.array(reader[wave_dir_path + "/RealCoeffs"])
            irow = 0
            for iforce in range(0, 6):
                if (body.Force_mask[iforce] == 1):
                    body.Froude_Krylov[iforce, :, idir].real = data[irow, :]
                    irow = irow + 1

            # Imaginary parts.
            data = np.array(reader[wave_dir_path + "/ImagCoeffs"])
            irow = 0
            for iforce in range(0, 6):
                if (body.Force_mask[iforce] == 1):
                    body.Froude_Krylov[iforce, :, idir].imag  = data[irow, :]
                    irow = irow + 1

        # Diffraction loads.

        diffraction_path = excitation_path + "/Diffraction"

        for idir in range(0, pyHDB.nb_wave_dir):
            wave_dir_path = diffraction_path + "/Angle_%u" % idir

            # Check of the wave direction.
            assert(abs(pyHDB.wave_dir[idir] - np.radians(np.array(reader[wave_dir_path + "/Angle"]))) < pow(10,-5))

            # Real parts.
            data = np.array(reader[wave_dir_path + "/RealCoeffs"])
            irow = 0
            for iforce in range(0, 6):
                if (body.Force_mask[iforce] == 1):
                    body.Diffraction[iforce, :, idir].real = data[irow, :]
                    irow = irow + 1

            # Imaginary parts.
            data = np.array(reader[wave_dir_path + "/ImagCoeffs"])
            irow = 0
            for iforce in range(0, 6):
                if (body.Force_mask[iforce] == 1):
                    body.Diffraction[iforce, :, idir].imag = data[irow, :]
                    irow = irow + 1

    def read_radiation(self, reader, pyHDB, body, radiation_path):

        """This function reads the added mass and damping coefficients and the impulse response functions with and without forward speed of the *.hdb5 file.

        Parameters
        ----------
        Writer : string
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        body : BodyDB.
            Body.
        radiation_path : string
            Path to radiation loads.
        """

        # Initializations.
        body.Inf_Added_mass = np.zeros((6, 6 * pyHDB.nb_bodies), dtype=np.float)
        body.irf = np.zeros((6, 6 * pyHDB.nb_bodies, pyHDB.nb_time_samples), dtype=np.float)
        body.irf_ku = np.zeros((6, 6 * pyHDB.nb_bodies, pyHDB.nb_time_samples), dtype=np.float)

        for j in range(pyHDB.nb_bodies):

            body_j = pyHDB.bodies[j]

            # Paths.
            radiation_body_motion_path = radiation_path + "/BodyMotion_%u" % j
            added_mass_path = radiation_body_motion_path + "/AddedMass"
            radiation_damping_path = radiation_body_motion_path + "/RadiationDamping"
            irf_path = radiation_body_motion_path + "/ImpulseResponseFunctionK"
            irf_ku_path = radiation_body_motion_path + "/ImpulseResponseFunctionKU"

            # Infinite added mass.
            data = np.array(reader[radiation_body_motion_path + "/InfiniteAddedMass"])
            irow = 0
            for iforce in range(0, 6):
                if (body.Force_mask[iforce] == 1): # Force activated.
                    icolumn = 0
                    for imotion in range(0,6):
                        if (body_j.Motion_mask[imotion]): # Dof of body_j activated.
                            body.Inf_Added_mass[iforce, 6 * body_j.i_body + imotion] = data[irow, icolumn]
                            icolumn = icolumn + 1
                    irow = irow + 1

            # Added mass.
            icolumn = 0
            for imotion in range(0, 6):
                if (body_j.Motion_mask[imotion]): # Dof of body_j activated.
                    data = np.array(reader[added_mass_path + "/DOF_%u" % icolumn])
                    irow = 0
                    for iforce in range(0, 6):
                        if (body.Force_mask[iforce] == 1):  # Force activated.
                            body.Added_mass[iforce, 6 * j + imotion, :] = data[irow, :]
                            irow = irow + 1
                    icolumn = icolumn + 1

            # Damping.
            icolumn = 0
            for imotion in range(0, 6):
                if (body_j.Motion_mask[imotion]):  # Dof of body_j activated.
                    data = np.array(reader[radiation_damping_path + "/DOF_%u" % icolumn])
                    irow = 0
                    for iforce in range(0, 6):
                        if (body.Force_mask[iforce] == 1):  # Force activated.
                            body.Damping[iforce, 6 * j + imotion, :] = data[irow, :]
                            irow = irow + 1
                    icolumn = icolumn + 1

            # Impulse response functions without forward speed.
            icolumn = 0
            for imotion in range(0, 6):
                if (body_j.Motion_mask[imotion]):  # Dof of body_j activated.
                    data = np.array(reader[irf_path + "/DOF_%u" % icolumn])
                    irow = 0
                    for iforce in range(0, 6):
                        if (body.Force_mask[iforce] == 1):  # Force activated.
                            body.irf[iforce, 6 * j + imotion, :] = data[irow, :]
                            irow = irow + 1
                    icolumn = icolumn + 1

            # Impulse response functions with forward speed.
            icolumn = 0
            for imotion in range(0, 6):
                if (body_j.Motion_mask[imotion]):  # Dof of body_j activated.
                    try:
                        data = np.array(reader[irf_ku_path + "/DOF_%u" % icolumn])
                        irow = 0
                        for iforce in range(0, 6):
                            if (body.Force_mask[iforce] == 1):  # Force activated.
                                body.irf_ku[iforce, 6 * j + imotion, :] = data[irow, :]
                                irow = irow + 1
                        icolumn = icolumn + 1
                    except:
                        pass

    def read_bodies(self, reader, pyHDB):
        """This function reads the body data of the *.hdb5 file.

        Parameters
        ----------
        reader : string.
            *.hdb5 file.
        pyHDB : object
            pyHDB object for storing the hydrodynamic database.
        """

        for ibody in range(pyHDB.nb_bodies):
            body_path = '/Bodies/Body_%u' % ibody

            # Index of the body.
            id = np.array(reader[body_path + "/ID"])
            assert ibody == id

            # Mesh.
            mesh = self.read_mesh(reader, pyHDB, body_path + "/Mesh")

            # Body definition.
            body = body_db.BodyDB(id, pyHDB.nb_bodies, pyHDB.nb_wave_freq, pyHDB.nb_wave_dir, mesh)

            # Body name.
            try:
                body.name = str(np.array(reader[body_path + "/BodyName"]))

                # Fix problem of convertion between bytes and string when using h5py.
                if (body.name[0:2] == "b'" and body.name[-1] == "'"):
                    body.name = body.name[2:-1]

            except:
                pass

            # Horizontal position in world and computation point in body frame
            try:
                position = np.array(reader[body_path + "/BodyPosition"])
                body.horizontal_position[0] = position[0]
                body.horizontal_position[1] = position[1]
                body.horizontal_position[2] = 0
                body.computation_point = np.zeros(3)
            except:
                pass

            # Masks.
            self.read_mask(reader, body, body_path + "/Mask")

            # # Force modes.
            # self.read_mode(reader, body, 0, body_path + "/Modes")
            #
            # # Motion modes.
            # self.read_mode(reader, body, 1, body_path + "/Modes")

            # Add body to pyHDB.
            pyHDB.append(body)

        for body in pyHDB.bodies:
            body_path = '/Bodies/Body_%u' % body.i_body

            # Diffraction and Froude-Krylov loads.
            self.read_excitation(reader, pyHDB, body, body_path + "/Excitation")

            # Added mass and damping coefficients and impulse response functions.
            self.read_radiation(reader, pyHDB, body, body_path + "/Radiation")

            # Hydrostatics.
            self.read_hydrostatic(reader, body, body_path + "/Hydrostatic")
