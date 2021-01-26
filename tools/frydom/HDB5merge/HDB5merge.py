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
    ArgParse module of hdb5merge.
"""

import os
import argparse

import frydom.HDB5tool.HDB5 as H5T

try:
    import argcomplete

    acok = True
except:
    acok = False

def creation_parser():
    parser = argparse.ArgumentParser(
        description="""  --  HDB5merge  --
            A Python module and a command line utility to merge HDB5 files.\n\n  Example of use:\n\n  hdb5merge --help""",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    return parser

def get_parser(parser):

    # Merge two hdb5 files into a single one.
    parser.add_argument('--merge', '-m', nargs=2, action="store", help="""Reading a hdb5 file with the given name.""")

    # Writing the hdb5 output file.
    parser.add_argument('--write', '-w', action="store", help="""Writing the hdb5 output file with the given name.""")

    return parser

def Read_hdb5(args):
    """This function reads the input *.HDB5 files."""

    # Reading a hdb5 file.
    if (args.merge is not None):

        # First hdb5 input file.
        database_1 = H5T.HDB5()
        database_1.read_hdb5(args.merge[0])

        # Second hdb5 input file.
        database_2 = H5T.HDB5()
        database_2.read_hdb5(args.merge[1])

    return database_1, database_2

def main():
    ####################################################################################################################
    #                                                   Parser
    ####################################################################################################################

    parser = creation_parser()
    parser = get_parser(parser)

    if acok:
        argcomplete.autocomplete(parser)

    args, unknown = parser.parse_known_args()

    ####################################################################################################################
    #                                        *.hdb5 input files to merge
    ####################################################################################################################

    database_1, database_2 = Read_hdb5(args)

if __name__ == '__main__':
    main()
