#!/usr/bin/env python
#  -*- coding: utf-8 -*-

from subprocess import Popen, PIPE
import numpy as np
import scipy.fftpack
import matplotlib.pyplot as plt
import time

# This Python file runs DTMB5512 bench of the FRyDoM project.

def GetMode(time, yfunc, tmin, tlength, fe):
    """ Compute the amplitude of a specific mode of the signal

    :param time: time array
    :param yfunc: data
    :param tmin: starting time of the FFT analysis
    :param tlength: length of the FFT windows
    :param fe: time sample
    :return:
    """

    dt = (time[1]-time[0])
    tlength = int(tlength*fe) / fe

    x_temp = time[time > tmin - 0.5*dt]
    y_temp = yfunc[time > tmin - 0.5*dt]

    x = x_temp[x_temp < tmin + tlength + 0.5*dt]
    y = y_temp[x_temp < tmin + tlength + 0.5*dt]

    N = x.size
    fs = 1. / dt

    yf = scipy.fftpack.fft(y)
    xf = np.linspace(0.0, fs, int(N/2) ) / 2.

    dxf = xf[1]-xf[0]

    fftAbs = 2. / N * np.abs(yf[:N // 2])

    fftAbs_temp = fftAbs[xf > fe - 5.*dxf]
    xf_temp = xf[xf > fe - 5.*dxf]
    fftAbs_temp = fftAbs_temp[xf_temp < fe + 5.*dxf]
    mode_1 = np.max(fftAbs_temp)

    #print(" C0 : %16.4f, C1 : %16.4f" % (fftAbs[0], mode_1))

    return fftAbs[0], mode_1

# Input parameters.
Froude = [0, 0.19, 0.28, 0.34, 0.41]
amplitude = [0.00317, 0.00341, 0.00368, 0.00398, 0.00431, 0.00470, 0.00513, 0.00564, 0.00621, 0.00688, 0.00767, 0.00860,
             0.00971, 0.01104, 0.01268, 0.01470, 0.01725, 0.02051, 0.02474, 0.03028, 0.03751, 0.04692, 0.05924]
period = [0.714, 0.741, 0.769, 0.800, 0.833, 0.870, 0.909, 0.952, 1.000, 1.053, 1.111, 1.176, 1.250, 1.333, 1.429,
          1.538, 1.667, 1.818, 2.000, 2.222, 2.500, 2.857, 3.333]
lambda_over_Lpp = [0.260, 0.281, 0.303, 0.328, 0.356, 0.387, 0.423, 0.465, 0.512, 0.568, 0.632, 0.709, 0.800, 0.910,
                   1.045, 1.212, 1.423, 1.691, 2.040, 2.497, 3.093, 3.869, 4.884]
name = "bench_DTMB5512"
zcog = 0.03 # Initial vertical position (m).
gravity = 9.80665 # gravity constant (m/s2).
Lpp = 3.048 # Ship length (m).

# FFT parameters.
tstart = 30 # s.
tlength = 20 # s.

# Experimental data.
data_exp = []
data_exp.append(np.genfromtxt("../../data/ce/bench/DTMB5512/2008_IRVINE_Fr_0.csv", delimiter=';', skip_header=2))
data_exp.append(np.genfromtxt("../../data/ce/bench/DTMB5512/2008_IRVINE_Fr_0.19.csv", delimiter=';', skip_header=2))
data_exp.append(np.genfromtxt("../../data/ce/bench/DTMB5512/2008_IRVINE_Fr_0.28.csv", delimiter=';', skip_header=2))
data_exp.append(np.genfromtxt("../../data/ce/bench/DTMB5512/2008_IRVINE_Fr_0.34.csv", delimiter=';', skip_header=2))
data_exp.append(np.genfromtxt("../../data/ce/bench/DTMB5512/2008_IRVINE_Fr_0.41.csv", delimiter=';', skip_header=2))

# Storage of the RAO amplitude.
RAO = np.zeros((len(period), len(Froude), 3, 3)) # First 3 for encounter frequency (0), heave (1) and pitch (2); Second 3 for no forward speed model (0), the simple (1) and extended (2) forward speed models.

# Loop over the Froude numbers.
for iFr in range(0, len(Froude)):
    Fr = Froude[iFr]
    forward_speed = Fr * np.sqrt(gravity * Lpp)
    # Loop over the wave periods.
    for iT in range(0, len(period)):
        T = period[iT]
        A = amplitude[iT]
        k = 2. * np.pi / (lambda_over_Lpp[iT] * Lpp) # Wave number.
        steepness = k * A # Wave steepness.
        fe = (1. / T) + (forward_speed / (Lpp * lambda_over_Lpp[iT]))  # Encounter frequency (Hz).

        # Loop over the forward speed effect models.
        for imodel in range(0, 3):

            if(imodel == 1):
                print("Fr = " + str(Fr) + " - Amplitude (m) = " + str(A) + " - Period (s) = " + str(T) + " - Simple forward speed model")
            elif(imodel == 2):
                print("Fr = " + str(Fr) + " - Amplitude (m) = " + str(A) + " - Period (s) = " + str(T) + " - Extended forward speed model")
            else:
                print("Fr = " + str(Fr) + " - Amplitude (m) = " + str(A) + " - Period (s) = " + str(T) + " - No forward speed model")

            # Run one case of the bench.
            args = ["../../cmake-build-release/bin/"+str(name), str(Fr), str(A), str(T), str(imodel), str(name)]
            t_begin = time.time()
            process = Popen(args, stdin=PIPE, stdout=PIPE, stderr=PIPE)
            process.communicate()
            # print("%s (pid=%s)" % (' '.join(args), process.pid))
            t_end = time.time()
            print("CPU time (s): " + str(t_end - t_begin))

            # Reading of the FRyDoM output data.
            log_folder = "../../cmake-build-release/logs/"+str(name)+"_Fr_%.6f_Amplitude_%.6f_Period_%.6f" % (Fr, A, T)
            if(imodel == 1):
                log_folder += "_Simple_forward_speed_model"
            elif(imodel == 2):
                log_folder += "_Extended_forward_speed_model"
            else:
                log_folder += "_No_forward_speed_model"
            data = np.genfromtxt(log_folder + "/FRYDOM_"+str(name)+"/BODIES/BODY_DTMB/BodyFrBody.csv", delimiter=';', skip_header=2)

            # Computation of the RAO in heave and pitch from FFT.
            constant_heave, RAO_heave = GetMode(data[:, 0], data[:, 6] - zcog, tstart, tlength, fe)
            constant_pitch, RAO_pitch = GetMode(data[:, 0], data[:, 8] * np.pi / 180, tstart, tlength, fe)

            # RAO nondimensionalization.
            RAO_heave /= A
            RAO_pitch /= steepness

            # Storing.
            RAO[iT, iFr, 0, imodel] = fe
            RAO[iT, iFr, 1, imodel] = RAO_heave
            RAO[iT, iFr, 2, imodel] = RAO_pitch

    # Plots.

    # Heave
    plt.figure(figsize=(6, 6))
    plt.plot(RAO[:, iFr, 0, 0], RAO[:, iFr, 1, 0], color="r", linestyle='-', label="FRyDoM - No forward speed model")
    plt.plot(RAO[: , iFr, 0, 1], RAO[: , iFr, 1, 1], color="b", linestyle='-', label="FRyDoM - Simple forward speed model")
    plt.plot(RAO[:, iFr, 0, 2], RAO[:, iFr, 1, 2], color="g", linestyle='--', label="FRyDoM - Extended forward speed model")
    plt.plot(data_exp[iFr][:, 0], data_exp[iFr][:, 1], color="k", linestyle='', marker = '+', label="Experimental data [Irvine2008]")
    plt.ylabel(r'$|RAO_{z}|$ (-)')
    plt.xlabel(r"$f_e$ (Hz)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("Bench_DTMB_RAO_Heave_Fr_"+str(Fr)+".png", dpi = 100)
    plt.close()

    # Heave
    plt.figure(figsize=(6, 6))
    plt.plot(RAO[:, iFr, 0, 0], RAO[:, iFr, 2, 0], color="r", linestyle='-', label="FRyDoM - No forward speed model")
    plt.plot(RAO[:, iFr, 0, 1], RAO[:, iFr, 2, 1], color="b", linestyle='-', label="FRyDoM - Simple forward speed model")
    plt.plot(RAO[:, iFr, 0, 2], RAO[:, iFr, 2, 2], color="g", linestyle='--', label="FRyDoM - Extended forward speed model")
    plt.plot(data_exp[iFr][:, 0], data_exp[iFr][:, 2], color="k", linestyle='', marker='+', label="Experimental data [Irvine2008]")
    plt.ylabel(r'$|RAO_{\theta}|$ (-)')
    plt.xlabel(r"$f_e$ (Hz)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("Bench_DTMB_RAO_Pitch_Fr_" + str(Fr) + ".png", dpi=100)
    plt.close()