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
    ArgParse module of hdb5tool.
"""

import os
import argparse

import frydom.HDB5tool.HDB5 as H5T

try:
    import argcomplete

    acok = True
except:
    acok = False


def creation_parser_CE():
    parser = argparse.ArgumentParser(
        description="""  --  HDB5tool  --
            A Python module and a command line utility to handle HDB5 files.\n\n  Example of use:\n\n  hdb5tool --help""",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    return parser


def get_parser(parser):

    # Path to Nemoh.cal.
    parser.add_argument('--path_to_nemoh_cal', '-cal', action="store", nargs=1, metavar='Arg', help="""
                Path to the folder including the file Nemoh.cal.""")

    # Discretization - Wave directions.
    parser.add_argument('--discretization_waves', '-dw', '-dbeta',
                        action="store", nargs=1, metavar='Arg', help="""
                Integer for the new discretization of the wave directions.""")

    # Discretization - Wave frequencies.
    parser.add_argument('--discretization_frequencies', '-dis_freq', '-df', action="store", nargs=1, metavar='Arg', help="""
                Integer for the new discretization of the wave frequencies.""")

    # Discretization - Final time.
    parser.add_argument('--final_time_irf', '-ft', action="store", nargs=1, metavar='Arg', help="""
                Final time for the computation of the impulse response functions.""")

    # Discretization - Time step.
    parser.add_argument('--time_step_irf', '-dt', action="store", nargs=1, metavar='Arg', help="""
                    Time step for the computation of the impulse response functions.""")

    # Body - Activate hydrostatics (useless).
    parser.add_argument('--activate_hydrostatics', '-activate_hs',
                        nargs='+', metavar='Arg', action="append", help="""
                Activate hydrostatics for the body of index given in argument.""")

    # Body - Hydrostatic matrix.
    parser.add_argument('--hydrostatics', '-hst',
                        metavar=('id', 'k33', 'k44', 'k55', 'k34', 'k35', 'k45'), nargs=7, type=float, action="append",
                        help="""
    #             Hydrostatic coefficients (K33, K44, K55, K34, K35, K45) for the body of index given in first argument.""")

    # Body - Activate inertia (useless).
    parser.add_argument('--activate_inertia', '-activate_i', action="append", nargs='+', metavar='Arg', help="""
                Activate inertia for the body of index given in argument.""")

    # Body - Inertia matrix (inertias and mass).
    parser.add_argument('--inertia', '-i', nargs=8, metavar=('id', 'mass', 'i44', 'i55', 'i66', 'i45', 'i46', 'i56'),
                        type=float, action="append", help="""
                Inertia coefficients and mass (Mass, I44, I55, I66, I45, I46, I56) for the body of index given in first argument.""")

    # Body - Inertia matrix (inertia only).
    parser.add_argument('--inertia_only', '-io', nargs=7, metavar=('id', 'i44', 'i55', 'i66', 'i45', 'i46', 'i56'),
                        type=float, action="append", help="""
                Inertia coefficients only (I44, I55, I66, I45, I46, I56) for the body of index given in first argument.""")

    # Body - Mass.
    parser.add_argument('--mass', '-m', nargs=2, metavar=('id', 'mass'), action="append", help="""
                Mass of the body of index given in argument.""")

    # Filtering impulse response functions.
    parser.add_argument('--cutoff_irf', '-coirf', nargs=5,
                        metavar=('tc', 'ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append", help="""
                Application of the filter with a cutoff time tc to the impulse response functions of ibody_force along iforce for a motion of ibody_motion along idof and plot the irf.""")

    # Filtering ALL impulse response functions.
    parser.add_argument('--cutoff_irf_all', '-coirf_all', nargs=1, metavar=('tc'), action="store", help="""
                    Application of the filter with a cutoff time tc to ALL impulse response functions.""")

    # Filtering impulse response functions with forward speed.
    parser.add_argument('--cutoff_irf_speed', '-coirf_speed', nargs=5,
                        metavar=('tc', 'ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append", help="""
                Application of the filter with a cutoff time tc to the impulse response functions with forward speed of ibody_force along iforce for a motion of ibody_motion along idof and plot the irf.""")

    # Filtering ALL impulse response functions with forward speed.
    parser.add_argument('--cutoff_irf_all_speed', '-coirf_all_speed', nargs=1, metavar=('tc'),
                        action="append", help="""
                    Application of the filter with a cutoff time tc to ALL impulse response functions with forward speed .""")

    # No symmetrization of the HDB.
    parser.add_argument('--sym_hdb', '-sym', action="store_true", help="""
                Symmetrization of the HDB.""")

    # Update the radiation mask.
    parser.add_argument('--radiation_mask', '-rm', action="store_true", help="""
                    Update the radiation mask of all bodies.""")

    # Update the radiation mask from the x-derivatives.
    parser.add_argument('--radiation_mask_x_derivatives', '-rmx', action="store_true", help="""
                        Update the radiation mask of all bodies from the x-derivatives.""")

    # Writing the hdb5 output file.
    parser.add_argument('--write', '-w', action="store", help="""
                Writing the hdb5 output file with the given name.""")

    # Plot the added mass and damping coefficients.
    parser.add_argument('--plot_radiation', '-pab', nargs=4,
                        metavar=('ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append", help="""
                Plot the added mass and damping coefficients of ibody_force along iforce for a motion of ibody_motion along idof.""")

    # Plot the x-derivative of the added mass and damping coefficients.
    parser.add_argument('--plot_radiation_derivative', '-pabx', nargs=4,
                        metavar=('ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append", help="""
                    Plot the x-derivative of the added mass and damping coefficients of ibody_force along iforce for a motion of ibody_motion along idof.""")

    # Plot the diffraction loads.
    parser.add_argument('--plot_diffraction', '-pd', '-pdiff', nargs=3,
                        metavar=('ibody', 'iforce', 'iwave'), action="append", help="""
                Plot the diffraction loads of ibody along iforce for iwave.""")

    # Plot the x-derivative of the diffraction loads.
    parser.add_argument('--plot_diffraction_derivative', '-pdx', '-pdiffx', nargs=3,
                        metavar=('ibody', 'iforce', 'iwave'), action="append", help="""
                    Plot the x-derivative of the diffraction loads of ibody along iforce for iwave.""")

    # Plot the Froude-Krylov loads.
    parser.add_argument('--plot_froude_krylov', '-pfk', nargs=3,
                        metavar=('ibody', 'iforce', 'iwave'), action="append", help="""
                Plot the Froude-Krylov loads of ibody along iforce for iwave.""")

    # Plot the x-derivative of the Froude-Krylov loads.
    parser.add_argument('--plot_froude_krylov_derivative', '-pfkx', nargs=3,
                        metavar=('ibody', 'iforce', 'iwave'), action="append", help="""
                    Plot the x-derivative of the Froude-Krylov loads of ibody along iforce for iwave.""")

    # Plot the excitation loads.
    parser.add_argument('--plot_excitation', '-pe', '-pexc', nargs=3,
                        metavar=('ibody', 'iforce', 'iwave'), action="append", help="""
                Plot the excitation loads of ibody along iforce for iwave.""")

    # Plot the x-derivative of the excitation loads.
    parser.add_argument('--plot_excitation_derivative', '-pex', '-pexcx', nargs=3,
                        metavar=('ibody', 'iforce', 'iwave'), action="append", help="""
                    Plot the x-derivative of the excitation loads of ibody along iforce for iwave.""")

    # Plot the IRF.
    parser.add_argument('--plot_irf', '-pirf', nargs=4,
                        metavar=('ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append", help="""
                Plot the impulse response functions of ibody_force along iforce for a motion of ibody_motion along idof.""")

    # Plot the IRF per body.
    parser.add_argument('--plot_irf_array', '-pirfa', action="store_true", help="""
                        Plot the impulse reponse functions of ibody_force for a motion of ibody_motion.""")

    # Plot the IRF speed.
    parser.add_argument('--plot_irf_speed', '-pirfs', '-pirfku', '-pirfx', nargs=4,
                        metavar=('ibody_force', 'iforce', 'ibody_motion', 'idof'), action="append", help="""
                Plot the impulse response functions with speed velocity of ibody_force along iforce for a motion of ibody_motion along idof.""")

    # Plot the IRF with speed per body.
    parser.add_argument('--plot_irf_speed_array', '-pirfsa', '-pirfkua', '-pirfxa', action="store_true", help="""
                            Plot the impulse reponse functions with speed velocity of ibody_force for a motion of ibody_motion.""")

    # Plot the mesh.
    parser.add_argument('--plot_mesh', '-pm', nargs = 1, metavar=('ibody'), action="append", help="""
                Plot the mesh of ibody.""")

    # Plot meshes.
    parser.add_argument('--plot_meshes', '-pms', action="store_true", help="""
                    Plot all the meshes in the same time.""")

    # Reading a hdb5 file.
    parser.add_argument('--read', '-r', action="store", help="""
                Reading a hdb5 file with the given name.""")

    # Initialization of the hdb.
    parser.add_argument('--initialization', '-init', action="store_true", help="""
                Initialization of the hydrodynamic database: computation of the Froude-Krylov loads, IRF, etc.""")

    # Information about the hdb5 file.
    parser.add_argument('--info', '-info', action="store_true", help="""
                    Information about the hdb5 file""")

    # Plot the mesh.
    parser.add_argument('--write_mesh', '-wm', nargs=1, metavar=('ibody'), action="append", help="""
                    Write the mesh of ibody in a *.obj file.""")

    # Plot meshes.
    parser.add_argument('--write_meshes', '-wms', action="store_true", help="""
                        Write all meshes in a single *.obj file.""")

    return parser


def Read_cal_hdb5(args):
    """This function reads the input files of a frequency-domain tool or a *.HDB5 file."""

    # BEM reader.
    if (args.path_to_nemoh_cal is not None): # Nemoh.
        database = H5T.HDB5()
        database.nemoh_reader(args.path_to_nemoh_cal[0])
        database._pyHDB.solver = "Nemoh"

    # Reading a hdb5 file.
    if (args.read is not None):
        database = H5T.HDB5()
        database.read_hdb5(args.read)
        database._is_initialized = True # No initialization except if asked.

    if (args.path_to_nemoh_cal is None and args.read is None):
        print("No input file has been defined.")
        print("Please give a Nemoh.cal file (-cal) or a .hdb5 file (-r) as input.")
        exit()
    elif (args.path_to_nemoh_cal is not None and args.read is not None):
        print("Only one input file may be defined.")
        print("Please choose between given a Nemoh.cal file (-cal) and a .hdb5 file (-r) as input.")
        exit()
    else:
        has_single_input = True

    return database


def get_Arg_part_1_CE(args, database):
    """This function makes all the fundamental computations with the hydrodynamic database:
    computation of IRF, infinite added masses, rediscretization, etc."""

    # Discretization - Wave directions.
    if (args.discretization_waves is not None):
        if (int(args.discretization_waves[0]) <= 1):
            print("The number of the wave direction discretization must be higher or equal to 2.")
            exit()
        database.discretization.nb_wave_directions = int(args.discretization_waves[0])

    # Discretization - Wave frequencies.
    if (args.discretization_frequencies is not None):
        if (int(args.discretization_frequencies[0]) <= 1):
            print("The number of the wave frequency discretization must be higher or equal to 2.")
            exit()
        database.discretization.nb_frequencies = int(args.discretization_frequencies[0])

    # Discretization - Final time for IRF.
    if (args.final_time_irf is not None):
        database.discretization._final_time = float(args.final_time_irf[0])

    # Discretization - Time step for IRF.
    if (args.time_step_irf is not None):
        database.discretization._delta_time = float(args.time_step_irf[0])

    # Initialize pyHDB.
    if (args.path_to_nemoh_cal is not None or args.initialization is True):  # _initialize is automatically called when a .cal is read.
        database._initialize()

    # Body - Active hydrostatics (useless).
    if (args.activate_hydrostatics is not None):
        nb_activation_hydrostatics = len(args.activate_hydrostatics)
        for id in range(0, nb_activation_hydrostatics):
            database.body[int(args.activate_hydrostatics[id]) - 1].activate_hydrostatic()

    # Body - Hydrostatic matrix.
    if (args.hydrostatics is not None):
        nb_hydrostatics = len(args.hydrostatics)

        for j in range(0, nb_hydrostatics):
            database.body[int(args.hydrostatics[j][0]) - 1].activate_hydrostatic()
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k33 = float(args.hydrostatics[j][1])
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k44 = float(args.hydrostatics[j][2])
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k55 = float(args.hydrostatics[j][3])
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k34 = float(args.hydrostatics[j][4])
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k35 = float(args.hydrostatics[j][5])
            database.body[int(args.hydrostatics[j][0]) - 1].hydrostatic.k45 = float(args.hydrostatics[j][6])

    # Body - Active inertia (useless).
    if (args.activate_inertia is not None):
        nb_activation_inertia = len(args.activate_inertia)
        for id in range(0, nb_activation_inertia):
            database.body[int(args.activate_inertia[id]) - 1].activate_inertia()

    # Body - Inertia matrix.
    if (args.inertia is not None):
        nb_inertia = len(args.inertia)
        for j in range(0, nb_inertia):
            database.body[int(args.inertia[j][0]) - 1].activate_inertia()
            database.body[int(args.inertia[j][0]) - 1].inertia.mass = float(args.inertia[j][1])
            database.body[int(args.inertia[j][0]) - 1].inertia.I44 = float(args.inertia[j][2])
            database.body[int(args.inertia[j][0]) - 1].inertia.I55 = float(args.inertia[j][3])
            database.body[int(args.inertia[j][0]) - 1].inertia.I66 = float(args.inertia[j][4])
            database.body[int(args.inertia[j][0]) - 1].inertia.I45 = float(args.inertia[j][5])
            database.body[int(args.inertia[j][0]) - 1].inertia.I46 = float(args.inertia[j][6])
            database.body[int(args.inertia[j][0]) - 1].inertia.I56 = float(args.inertia[j][7])

    # Body - Inertia matrix.
    if (args.inertia_only is not None):
        nb_inertia_only = len(args.inertia_only)
        for j in range(0, nb_inertia_only):
            database.body[int(args.inertia_only[j][0]) - 1].activate_inertia()
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I44 = float(args.inertia_only[j][1])
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I55 = float(args.inertia_only[j][2])
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I66 = float(args.inertia_only[j][3])
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I45 = float(args.inertia_only[j][4])
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I46 = float(args.inertia_only[j][5])
            database.body[int(args.inertia_only[j][0]) - 1].inertia.I56 = float(args.inertia_only[j][6])

    # Body - Mass.
    if (args.mass is not None):
        nb_mass = len(args.mass)
        for j in range(0, nb_mass):
            database.body[int(args.mass[j][0]) - 1].activate_inertia()
            database.body[int(args.mass[j][0]) - 1].inertia.mass = float(args.mass[j][1])

    return database

def get_Arg_part_2_CE(args, database):
    """This function makes the symmetry of the HDB, filters the IRF, the and updates the radiation mask."""

    # Filtering impulse response functions.
    if (args.cutoff_irf is not None):
        nb_cut_off_irf = len(args.cutoff_irf)
        for j in range(0, nb_cut_off_irf):
            database.Cutoff_scaling_IRF(tc=float(args.cutoff_irf[j][0]), ibody_force=int(args.cutoff_irf[j][1]) - 1,
                                        iforce=int(args.cutoff_irf[j][2]) - 1,
                                        ibody_motion=int(args.cutoff_irf[j][3]) - 1,
                                        idof=int(args.cutoff_irf[j][4]) - 1)

    # Filtering ALL impulse response functions.
    if (args.cutoff_irf_all is not None):
        for body_force in database.body:
            for iforce in range(0, 6):
                for body_motion in database.body:
                    for idof in range(0, 6):
                        database.Cutoff_scaling_IRF(tc=float(args.cutoff_irf_all[0]), ibody_force=body_force.i_body,
                                                    iforce=iforce, ibody_motion=body_motion.i_body,
                                                    idof=idof, auto_apply=True)

    # Filtering impulse response functions with forward speed.
    if (args.cutoff_irf_speed is not None):
        nb_cut_off_irf_speed = len(args.cutoff_irf_speed)
        for j in range(0, nb_cut_off_irf_speed):
            database.Cutoff_scaling_IRF_speed(tc=float(args.cutoff_irf_speed[j][0]),
                                              ibody_force=int(args.cutoff_irf_speed[j][1]) - 1,
                                              iforce=int(args.cutoff_irf_speed[j][2]) - 1,
                                              ibody_motion=int(args.cutoff_irf_speed[j][3]) - 1,
                                              idof=int(args.cutoff_irf_speed[j][4]) - 1)

    # Filtering ALL impulse response functions with forward speed.
    if (args.cutoff_irf_all_speed is not None):
        for body_force in database.body:
            for iforce in range(0, 6):
                for body_motion in database.body:
                    for idof in range(0, 6):
                        database.Cutoff_scaling_IRF_speed(tc=float(args.cutoff_irf_all[0]),
                                                          ibody_force=body_force.i_body, iforce=iforce,
                                                          ibody_motion=body_motion.i_body, idof=idof, auto_apply=True)

    # Symmetry of the HDB.
    if (args.sym_hdb is True):
        database.symmetry_HDB()

    # Radiation mask.
    if (args.radiation_mask is True):
        database.Update_radiation_mask()
    if (args.radiation_mask_x_derivatives is True):
        database.Update_radiation_mask_x_derivatives()

    return database

def get_Arg_part_3_CE(args, database):
    """ This function plots all the physical quantities: added mass, damping, loads and IRF."""

    # Plot mesh.
    if (args.plot_mesh is not None):
        nb_plots_mesh = len(args.plot_mesh)
        for j in range(0, nb_plots_mesh):
            database.Plot_Mesh(ibody=int(args.plot_mesh[j][0]) - 1)

    # Plot meshes.
    if (args.plot_meshes is True):
        database.Plot_Meshes()

    # Write mesh.
    if (args.write_mesh is not None):
        nb_write_mesh = len(args.write_mesh)
        for j in range(0, nb_write_mesh):
            database.Write_Mesh(ibody=int(args.write_mesh[j][0]) - 1)

    # Write meshes.
    if (args.write_meshes is True):
        database.Write_Meshes()

    # Plot the added mass and damping coefficients.
    if (args.plot_radiation is not None):
        nb_plots_radiation = len(args.plot_radiation)
        for j in range(0, nb_plots_radiation):
            database.Plot_Radiation_coeff(ibody_force=int(args.plot_radiation[j][0]) - 1,
                                          iforce=int(args.plot_radiation[j][1]) - 1,
                                          ibody_motion=int(args.plot_radiation[j][2]) - 1,
                                          idof=int(args.plot_radiation[j][3]) - 1)

    # Plot the x-derivative of the added mass and damping coefficients.
    if (args.plot_radiation_derivative is not None):
        nb_plots_radiation_derivative = len(args.plot_radiation_derivative)
        for j in range(0, nb_plots_radiation_derivative):
            database.Plot_Radiation_coeff_x_derivative(ibody_force=int(args.plot_radiation_derivative[j][0]) - 1,
                                                       iforce=int(args.plot_radiation_derivative[j][1]) - 1,
                                                       ibody_motion=int(args.plot_radiation_derivative[j][2]) - 1,
                                                       idof=int(args.plot_radiation_derivative[j][3]) - 1)

    # Plot the diffraction loads.
    if (args.plot_diffraction is not None):
        nb_plots_diffraction = len(args.plot_diffraction)
        for j in range(0, nb_plots_diffraction):
            database.Plot_Diffraction(ibody=int(args.plot_diffraction[j][0]) - 1,
                                      iforce=int(args.plot_diffraction[j][1]) - 1,
                                      iwave=int(args.plot_diffraction[j][2]) - 1)

    # Plot the x-derivative of the diffraction loads.
    if (args.plot_diffraction_derivative is not None):
        nb_plots_diffraction_derivative = len(args.plot_diffraction_derivative)
        for j in range(0, nb_plots_diffraction_derivative):
            database.Plot_Diffraction_x_derivative(ibody=int(args.plot_diffraction_derivative[j][0]) - 1,
                                                   iforce=int(args.plot_diffraction_derivative[j][1]) - 1,
                                                   iwave=int(args.plot_diffraction_derivative[j][2]) - 1)

    # Plot the Froude-Krylov loads.
    if (args.plot_froude_krylov is not None):
        nb_plots_froude_krylov = len(args.plot_froude_krylov)
        for j in range(0, nb_plots_froude_krylov):
            database.Plot_Froude_Krylov(ibody=int(args.plot_froude_krylov[j][0]) - 1,
                                        iforce=int(args.plot_froude_krylov[j][1]) - 1,
                                        iwave=int(args.plot_froude_krylov[j][2]) - 1)

    # Plot the Froude-Krylov loads.
    if (args.plot_froude_krylov_derivative is not None):
        nb_plots_froude_krylov_derivative = len(args.plot_froude_krylov_derivative)
        for j in range(0, nb_plots_froude_krylov_derivative):
            database.Plot_Froude_Krylov_x_derivative(ibody=int(args.plot_froude_krylov_derivative[j][0]) - 1,
                                                     iforce=int(args.plot_froude_krylov_derivative[j][1]) - 1,
                                                     iwave=int(args.plot_froude_krylov_derivative[j][2]) - 1)

    # Plot the excitation loads.
    if (args.plot_excitation is not None):
        nb_plots_excitation = len(args.plot_excitation)
        for j in range(0, nb_plots_excitation):
            database.Plot_Excitation(ibody=int(args.plot_excitation[j][0]) - 1,
                                     iforce=int(args.plot_excitation[j][1]) - 1,
                                     iwave=int(args.plot_excitation[j][2]) - 1)

    # Plot the x-derivative of the excitation loads.
    if (args.plot_excitation_derivative is not None):
        nb_plots_excitation_derivative = len(args.plot_excitation_derivative)
        for j in range(0, nb_plots_excitation_derivative):
            database.Plot_Excitation_x_derivative(ibody=int(args.plot_excitation_derivative[j][0]) - 1,
                                                  iforce=int(args.plot_excitation_derivative[j][1]) - 1,
                                                  iwave=int(args.plot_excitation_derivative[j][2]) - 1)

    # Plot the impulse response functions.
    if (args.plot_irf is not None):
        nb_plots_irf = len(args.plot_irf)
        for j in range(0, nb_plots_irf):
            database.Plot_IRF(ibody_force=int(args.plot_irf[j][0]) - 1, iforce=int(args.plot_irf[j][1]) - 1,
                              ibody_motion=int(args.plot_irf[j][2]) - 1,
                              idof=int(args.plot_irf[j][3]) - 1)

    # Plot the impulse response functions per body.
    if (args.plot_irf_array is True):
        database.Plot_irf_array()

    # Plot the impulse response function with speed velocity.
    if (args.plot_irf_speed is not None):
        nb_plots_irf_speed = len(args.plot_irf_speed)
        for j in range(0, nb_plots_irf_speed):
            database.Plot_IRF_speed(ibody_force=int(args.plot_irf_speed[j][0]) - 1,
                                    iforce=int(args.plot_irf_speed[j][1]) - 1,
                                    ibody_motion=int(args.plot_irf_speed[j][2]) - 1,
                                    idof=int(args.plot_irf_speed[j][3]) - 1)

    # Plot the impulse response functions with speed velocity per body.
    if (args.plot_irf_speed_array is True):
        database.Plot_irf_speed_array()

    return database


def get_Arg_part_4_CE(args, database):
    """This function writes the *.hdb5 output file."""

    # Information about the hdb5.
    if (args.info is True):
        if (args.path_to_nemoh_cal is not None):
            database.write_info(None)
        if (args.read is not None):
            database.write_info(args.read)

    # Writing the hdb5 output file.
    if (args.write is not None):
        database.export_hdb5(args.write)

    return database


def main():
    ####################################################################################################################
    #                                                   Parser
    ####################################################################################################################

    parser = creation_parser_CE()
    parser = get_parser(parser)

    if acok:
        argcomplete.autocomplete(parser)

    args, unknown = parser.parse_known_args()

    ####################################################################################################################
    #                               Selection of an input file: Nemoh file or hdb5 file
    ####################################################################################################################

    database = Read_cal_hdb5(args)

    ####################################################################################################################
    #                                               Reading arguments
    ####################################################################################################################

    # 1st set of arguments - FryDoM CE - Computation of IRF, infinite added masses, rediscretization, filering, etc.
    database = get_Arg_part_1_CE(args, database)

    # 2nd set of arguments - FRyDoM CE - Symmetry of the HDB, filterering the IRF and updating of the radiation mask.
    database = get_Arg_part_2_CE(args, database)

    # 3rd set of arguments - FRyDoM CE - Plots of A, B, Fdiff, Ffk, Fexc., IRF.
    database = get_Arg_part_3_CE(args, database)

    # 4th set of arguments - FRyDoM CE - Information about the hdb and writing of the *.hdb5 output file.
    database = get_Arg_part_4_CE(args, database)


if __name__ == '__main__':
    main()
