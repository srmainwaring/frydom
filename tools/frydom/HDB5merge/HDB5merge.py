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

    return parser

def main():
    ####################################################################################################################
    #                                                   Parser
    ####################################################################################################################

    parser = creation_parser()
    parser = get_parser(parser)

    if acok:
        argcomplete.autocomplete(parser)

    args, unknown = parser.parse_known_args()

if __name__ == '__main__':
    main()
