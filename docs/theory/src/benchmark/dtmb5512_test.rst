.. dtmb5512_test:

DTMB5512 surface combatant in regular head waves
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This benchmark corresponds to the simulation of the surface combatant DTMB5512 with constant forward speed submitted to
regular head waves. The DTMB5512 is a 3.048m geosym of the DTMB5415 which has been chosen by the International Towing
Tank Conference (ITTC) as an international benchmark for CFD validation. Two configurations are considered, captive
motion and free motion in pitch and heave (3DOF). Description of this test case and comparison to experimental results
are summarized in the following.

Description of the test case
----------------------------

In this simulation, the 1:46.6 scale is considered which corresponds to the laboratory scale of the DTMB 5512 geometry
used in the experimental campaigned made by the Iowa Institute of Hydraulic Research (IIHR) ([Gui2001]_, [Irvine2008]_). The hull model with no
appendage, no propeller and no rudder is considered. The CAD model and data are freely available on the IIHR website. The main characteristics of the
vessel are summarized in the following table. The x-axis is pointing forward to the vessel and z-axis is pointing upward
as represented in the following figure.

.. _fig_DTMB5512_geometry:
.. figure:: _static/DTMB5512_configuration.png
    :align: center
    :alt: DTMB5512 geometry

    Description of the test case.

============================== ================ =====================
Scale                          :math:`-`        46.6
Length (Lpp)                   :math:`m`        3.048
Beam                           :math:`m`        0.405
Draft                          :math:`m`        0.132
Wetted surface area            :math:`m^2`      1.371
Longitudinal center of gravity :math:`m`        1.536
Vertical center of gravity     :math:`m`        0.162
Block coefficient              :math:`-`        0.506
============================== ================ =====================

Non-zero hydrostatic coefficients are presented in the next table.

===================== ================ ====================
:math:`K_{33}`        :math:`N/m`      :math:`9680`
:math:`K_{44}`        :math:`N.m`      :math:`34.6`
:math:`K_{55}`        :math:`N.m`      :math:`5420`
:math:`K_{35}`        :math:`N`        :math:`1250`
===================== ================ ====================

The non-zero inertial properties of the scaled DTMB are listed in the table below. They come from Table 3-1 of [Yoon2009]_.

===================== ================ ==================================================
Center of gravity     :math:`m`        :math:`\begin{pmatrix} 0 & 0 & 0.03 \end{pmatrix}`        
Displacement          :math:`kg`       86
:math:`I_{xx}`        :math:`kg.m^2`   1.98
:math:`I_{yy}`        :math:`kg.m^2`   53.88
:math:`I_{zz}`        :math:`kg.m^2`   49.99
===================== ================ ==================================================

The DTMB5512 is mounted on a carriage with constant forward speed as represented in next figure.
A `video <https://www.youtube.com/watch?v=yUbBE2nytg0>`_ is also available for the semi captive test.

.. _fig_simulation_picture:
.. figure:: _static/DTMB5512_simulation_picture.png
    :align: center
    :alt: DTMB5512 simulation
    :scale: 50%

For the captive test , the DTMB5512 model is fixed on the carriage whereas for pitch and heave motion test the DTMB5512
model is free to move in heave and pitch. Four different speed are considered which correspond to a low (0.19),
medium (0.28), mid-high (0.34) and high (0.41) froude numbers.

Integration of steady forces due to forward speed
-------------------------------------------------

Resistance force (ITTC57)
..........................

Due to the forward speed the vessel is submitted to a resistance force in surge motion and additional heave and pitch steady force. These forces are determined from the experimental results in steady motion with no waves.

Resistance force is computed according to the ITTC57 standard with a hull factor :math:`k=0.03`. Based on experimental results, a residual coefficient :math:`C_R` is estimated for the different vessel speed.

================ =====================
Froude Number         Residual coeff.
================ =====================
0.19             :math:`5,37.10^{-4}`
0.24             :math:`9,07.10^{-4}`
0.34             :math:`1,68.10^{-3}`
0.41             :math:`4,02.10^{-3}`
================ =====================

User defined forces
...................

Based on the experimental results with forward speed and no waves, steady forces for pitch :math:`M_{y,user}` and :math:`F_{z,user}` are defined:

.. math::
    F_{z,user} = -12.3 u^3 - 2.9 u^2 \\
    M_{y,user} = 4.3 u^6 - 9.1 u^5 - 9.7 u^4 + 34.2 u^3 - 22.7 u^2

where :math:`u` is the steady forward speed of the vessel.

Wave drift force
................

For regular wave, the wave drift force is estimated from the following formula:

.. math::
    F_{wd} = A^2 . C_{wd}(\omega)

where :math:`A` is the wave amplitude, :math:`\omega` is the wave frequency and :math:`C_{wd}` the mean wave drift coefficient.

In this test case, the mean wave drift coefficient :math:`C_{wd}` is estimated from experimental results and added resistance given by :

.. math::
    C_{T,ad} = \bar{C_T} - C_{T,st}

where :math:`\bar{C_T}` is the mean unsteady resistance and :math:`C_{T,st}` is the steady resistance in calm water.

Estimation of the mean wave drift coefficient in surge :math:`C_{wd, x}` for various wave frequencies is represented in :numref:`fig_wave_drift_coeff`

.. _fig_wave_drift_coeff:
.. figure:: _static/Cwd.png
    :align: center
    :alt: Mean Wave drift
    :scale: 50 %

    Estimation of the mean wave drift coefficient depending on wave frequencies.


Adimentionalization of the forces
---------------------------------

To compare forces and moment applied on the vessel to the experimental results, the following adimentionalization is applied:

.. math::
    C_t = \frac{F_x}{0.5 \rho U^2 S} \\
    C_h = \frac{F_z}{0.5 \rho U^2 S} \\
    C_m = \frac{M_y}{0.5 \rho U^2 L S}

Following [ref], harmonic decomposition of the forces and moment are applied as follows:

.. math::
    X_F(t) = \frac{X_0}{2} + \sum_n X_n cos(2 \pi n f_e t)

where :math:`X_0` is the constant part and :math:`X_n` the :math:`n^{th}` harmonic coefficient of the function :math:`X_F`.


Captive test results
--------------------

The regular wave field propagates in negative x-direction. Four different wave amplitudes are considered corresponding to small (0.025), small-median (0.05, 0.075) and median (0.1) steepness. The wave period is equal to 0.22 seconds.

The zeroth and first harmonic coefficients of :math:`C_T`, :math:`C_H` and :math:`C_M` are compared to experimental results [Gui2002]_ in :numref:`fig_zeroth_coeff` and :numref:`fig_first_harmonic`.

.. _fig_zeroth_coeff:
.. figure:: _static/zero_harmonic_plot.png
    :align: center
    :alt: Zeroth coefficient
    :scale: 50%

    Comparison of the zeroth coefficients from FRyDoM (continuous lines) with experimental results (discontinuous lines).

.. _fig_first_harmonic:
.. figure:: _static/first_harmonic_plot.png
    :align: center
    :alt: First coefficient
    :scale: 50%

    Comparison of the first harmonic coefficient from FRyDoM (continuous lines) with experimental results (discontinuous lines).


Pitch and Heave Motion test results
-----------------------------------

The Response Amplitude Operator (RAO) of the DTMB5512 for pitch and heave motions are compared to experimental results provided by [Irvine2008]_. Five different Froude numbers are considered and are equal to 0, 0.19, 0.28, 0.34 and 0.41. Amplitudes are obtained from a Fourier transformation of the heave and pitch motions. Heave amplitudes are nondimensionalized by the incident wave amplitude while pitch amplitudes are divided by the wave steepess.

Comparisons are displayed in :numref:`fig_heave_motion` in heave and in :numref:`fig_pitch_motion` in pitch. A very good agreement is obtained in heave for every Froude number. Regarding the pitch motion, the zero forward speed case shows a very good match. For a nonzero Froude number, differences are observed and a shift of the peak of amplitude appears. A more accurate forward speed model could improve these results.

.. _fig_heave_motion:
.. figure:: _static/Bench_DTMB_RAO_Heave.png
    :align: center
    :alt: Heave motion

    Comparison of the nondimensionalized heave RAO from FRyDoM (red) with experimental data (black) with respect to encounter frequency for different Froude numbers (0.0, 0.19, 0.28, 0.34, 0.41 - from left to right and top to bottom).

.. _fig_pitch_motion:
.. figure:: _static/Bench_DTMB_RAO_Pitch.png
    :align: center
    :alt: Pitch motion

    Comparison of the pitch RAO from FRyDoM (red) with experimental data (black) with respect to encounter frequency for different Froude numbers (0.0, 0.19, 0.28, 0.34, 0.41 - from left to right and top to bottom).

References
----------

.. [Gui2001] L. Gui, J. Longo, B. Metcalf, J. Shao, F. Stern, "Forces, moment, and wave pattern for surface combatant in regular head waves. Part 1 : Measurment systems and uncertainty assessment", Experiments in Fluids, Vol 31, 2001, pp 674-680.

.. [Gui2002] L. Gui, J. Longo, B. Metcalf, J. Shao, F. Stern, "Forces, moment, and wave pattern for surface combatant in regular head waves. Part 2 : Measurment results and discussions", Experiments in Fluids, Vol 32, 2002, pp 27-36.

.. [Irvine2008] M. Irvine, J. Longo, F. Stern, "Pitch and Heave Tets Uncertainty assessment for a surface combatant in regular head waves", Journal Ship Research, Vol 52, No 2, June 2008, pp 146-163.

.. [Yoon2009] H. Yoon, "Phase-averaged stereo-PIV flow field and force/moment/motion measurements for surface combatant in PMM maneuvers", PhD thesis, University of Iowa, 2009.
