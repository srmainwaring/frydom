# -*- coding: utf-8 -*-

import argparse
import csv
import json

import matplotlib.pyplot as plt
import numpy as np
from pools.units import Unit


def create_parser():
    parser = argparse.ArgumentParser(
        description=""" Scripts for the conversion of wind and current polar coefficients, from .csv to .json""",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    return parser


def get_parser(parser):
    parser.add_argument('--input', '-i', type=str,
                        help="Input csv file path")

    parser.add_argument('--output', '-o', type=str,
                        help="Output json file path")

    # parser.add_argument('--verbose', '-v', action='store_true', help="verbosity")

    parser.add_argument('--plot', '-p', action='store_true', help="plot polar curves")

    return parser


def ReadCSVHeader(filename, sep=";"):
    header = dict()

    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile, delimiter=sep)

        for field in reader.fieldnames:
            header[field] = []

        line = next(reader)
        try:
            val = float(line[header[0]])
        except:
            for key in line.keys():
                header[key].append(line[key])

        return header


def ReadCSV(filename, fields=None, sep=";"):
    """ Read the csv file and create a dictionary of the data.
    The first line of the csv file corresponding to the header of the column.
    Next lines must contains the data (float number). If additional lines are
    include in the header they must be mentioned with the n_extra argument.

    :param filename: name of the csv file
    :param sep: separator of the csv file
    :param n_extra: number of extra header line in the csv file
    :return: dictionary
    """

    # Read header and return dict
    header = ReadCSVHeader(filename, sep=sep)

    n_header = 1 + len(header[next(iter(header))])

    mydict = dict()
    if fields:
        for field in fields:
            mydict[field] = np.array([])
    else:
        for field in header:
            mydict[field] = np.array([])

    keys = list(header.keys())
    cols = [keys.index(field) for field in mydict.keys()]

    data = np.genfromtxt(filename, delimiter=sep, skip_header=n_header, usecols=cols)

    data = data.reshape((int(data.size / len(cols)), len(cols)))

    i = 0
    for key in mydict.keys():
        mydict[key] = data[:, i]
        i += 1

    # If no unit in csv file
    if n_header == 1:
        for key in header.keys():
            header[key].append("-")

    return mydict, header


def convert_units_dict(data, header, variables, units):
    for i, key in enumerate(variables):
        unit_key = Unit(header[key][0])
        scale = unit_key.scale_to_other(units[i])
        data[key] *= scale
        header[key][0] = units[i]

    return data, header


def check_conventions(data):
    cx_0 = data['cx'][np.where(data['angle'] == 0)]
    cy_90 = data['cy'][np.where(data['angle'] == 90)]

    if cx_0 >= 0 and cy_90 >= 0:
        return 'NWU', 'GOTO'
    if cx_0 <= 0 and cy_90 <= 0:
        return 'NWU', 'COMEFROM'
    if cx_0 <= 0 and cy_90 >= 0:
        return 'NED', 'COMEFROM'
    if cx_0 >= 0 and cy_90 <= 0:
        return 'NED', 'GOTO'


def plot(data):
    fig, ax1 = plt.subplots()

    plt.grid(which='both')
    ax2 = ax1.twinx()
    ax1.plot(data['angle'], data['cx'], 'g-', label='cx')
    ax1.plot(data['angle'], data['cy'], 'r-', label='cy')
    ax2.plot(data['angle'], data['cn'], 'b-', label='cn')

    ax1.set_xlabel('angles (deg)')
    ax1.set_ylabel('cx, cy', color='g')
    ax2.set_ylabel('cn', color='b')
    fig.legend()

    plt.show()


def main():
    parser = create_parser()
    parser = get_parser(parser)

    args, unknown = parser.parse_known_args()

    variables = ['angle', 'cx', 'cy', 'cn']
    data, header = ReadCSV(args.input, variables, sep=';')
    # not adimensionalized by the lateral or front area
    units = ['deg', 'N.s^2/m^2', 'N.s^2/m^2', 'N.s^2/m']
    data, header = convert_units_dict(data, header, variables, units)

    frame_convention, direction_convention = check_conventions(data)

    if args.plot:
        plot(data)

    json_dict = {'angles': data['angle'].tolist(),
                 'cx': data['cx'].tolist(),
                 'cy': data['cy'].tolist(),
                 'cn': data['cn'].tolist(),
                 'unit': 'DEG',
                 'FRAME_CONVENTION': frame_convention,
                 'DIRECTION_CONVENTION': direction_convention
                 }
    if args.output:
        with open(args.output, "w") as corr_file:
            json.dump(json_dict, corr_file, indent=2)
    # else:
    #     print(json_dict)


if __name__ == '__main__':
    main()
