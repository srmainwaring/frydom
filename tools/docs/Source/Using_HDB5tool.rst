Using HDB5tool
==============

.. note::

    * So far, only the ouput files of **Nemoh** may be read.
    * The command line options may be combined.
    * Indexes present in arguments start from 1 (for instance, the first body has an index of 1, the surge too).
    * Physical quantities are expressed at the center of gravity of each body.
    * Negative numbers written with a scientific notation are not read by the command line argument parser, please use a decimal notation in this case. Thus, write -1000 instead of -10e3.

.. contents:: Content
    :local:
    :backlinks: top

.. highlight:: bash

Getting help
------------

You can get command line help by issuing the following command::

    >$ hdb5tool -h

Reading an input file
---------------------

**HDB5tool** can read two types of input files:
 - The output files of **Nemoh**;
 - A *.hdb5* file.

From the **Nemoh** output files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Reading the output files of Nemoh is achieved by writing the path to the *folder* including the Nemoh.cal input file of **Nemoh**::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder

.. note::

    As a folder is asked, no extension is expected. You can use a relative or absolute path.

From a *.hdb5* file
~~~~~~~~~~~~~~~~~~~

If a *.hdb5* file already exists, it can be downloaded by the following command::

    >$ hdb5tool -r path/to/hdb5/file

The extension of the file is necessary.

.. note::

    You can use a relative or absolute path. 

.. note::

    In the following, the commands are given based on the reading of the **Nemoh** output files (``-cal path/to/Nemoh/cal``) but it can be replaces by the reading of an *.hdb5* file (``-r path/to/hdb5``).

Rediscretization of the hydrodynamic database
---------------------------------------------

Wave directions
~~~~~~~~~~~~~~~

The update of the wave direction discretization is obtained by using the following command::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -dw 50

Here, the hydrodynamic database will be rediscretized for 50 wave directions.

.. note::

    The new discretization for the wave directions is achieved before symmetrizing the hydrodynamic database.

Wave frequencies
~~~~~~~~~~~~~~~~

The update of the wave frequency discretization is obtained by writing::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -df 60

Here, the hydrodynamic database will be rediscretized for 60 wave frequencies.

Impulse response functions
--------------------------

Final time
~~~~~~~~~~

To set the final time for evaluating the impulse response functions with and without forward speed (here :math:`40` \\(s\\)), use::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -ft 40

.. note::

    By default, the final time is:

    :math:`T_f = \dfrac{2\pi}{2d\omega} = \dfrac{\pi}{d\omega}`

    where :math:`d\omega` represents the wave frequency step.

Time step
~~~~~~~~~

To set the time step for evaluating the impulse response functions with and without forward speed (here :math:`0.01` \\(s\\)), use::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -dt 0.01

.. note::

    By default, the time step is :math:`0.008` \\(s\\).

Cutoff scaling function
~~~~~~~~~~~~~~~~~~~~~~~

For smoothing the impulse response functions, the following cutoff scaling function may be applied:

.. math::
    c(t) = \exp\left(-\dfrac{3t}{t_c}\right)^2

where :math:`t_c` is the cutoff time.

The command to set the cutoff time for impulse response function without forward speed of the body *ibody_force* along the force *iforce* for a motion of *ibody_motion* along the degree of freedom *idof* is::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -coirf tc ibody_force iforce ibody_motion idof

For example, for a cuttoff time of :math:`10` \\(s\\) of the impulse response function :math:`K_{33}` of the first body::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -coirf 10 1 3 1 3

Then, if you want to apply this cutoff scaling function and update the impulse response function, write *yes*, otherwise write *no*.

For applying the cutoff scaling function to an impulse response function with forward speed, use ``-coirf_speed`` instead of ``-coirf``.

It is also possible to apply automatically the cutoff scaling function to all impulse response functions for all bodies with the same cutoff time by using the following command::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -coirf_all 10.

And in case of impulse response functions with forward speed::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -coirf_all_speed 10.

Radiation mask
--------------

The radiation coefficients close to zero may be canceled, so that they won't be used in the time-domain computations of FRyDoM. To do so, a radiation mask is used. It is represented by a matrix of size :math:`(6 n_B) \times (6 n_B)` where :math:`n_B` is the number of bodies. By default, every radiation coefficient is fixed is used and its corresponding radiation mask coefficient is set to True. By using the command::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -rm

the quantity :math:`\mathbf{H}(j\omega) = |\mathbf{B}(\omega) + j\omega[\mathbf{A}(\omega) - \mathbf{A}^{\infty}]|` is evaluated and plotted for all bodies and all degrees of freedom. The notation is :math:`H_{\alpha_i \beta_j}` and represents the effect of the degree of freedom :math:`\beta` of body :math:`j` on the degree of freedom :math:`\alpha` of body :math:`i`. If you considered a quantity is negligible, you can click on the plot. The radiation mask for this coefficient will turn to False and, visually, the background color of the plot will become grey. By clicking another time on the same plot, the radiation mask for this coefficient will be equal to True again and the background color of the plot will turn white as initially. An example is given below:

.. figure:: /_static/Radiation_mask_before_clicking.png
   :align: center

   Plot of every coefficient of the matrix :math:`\mathbf{H}_{11}`

.. figure:: /_static/Radiation_mask_after_clicking_mistake.png
   :align: center

   The coefficients :math:`H_{y_1z_1}`, :math:`H_{z_1y_1}` and :math:`H_{z_1 \theta_1}` are considered as negligible

.. figure:: /_static/Radiation_mask_after_clicking_correction.png
   :align: center

   The coefficient :math:`H_{z_1 \theta_1}` is not considered as negligible anymore

Postprocessing of the hydrodynamic database
-------------------------------------------

The computation of the Froude-Krylov loads, the infinite added-mass matrices, the impulse response functions, the discretization and the interpolation of the hydrodynamic database are achieved by using the ``-init`` command::

    >$ hdb5tool -r path/to/hdb5/file -init

.. note::

    This command is only available when a *.hdb5* input file is read. When the output files of a frequency-domain potential flow based solver (such as **Nemoh**) are read, this command is automatically called.

Hydrostatic matrix
------------------

It is possible to define a hydrostatic stiffness matrix for each body, in order to be read by **FRyDoM-CE** or for computing the Response Amplitude Operators thereafter. This matrix is defined by:

.. math::

    K_{hs} = \begin{bmatrix}
                0 & 0 & 0 & 0 & 0 & 0 \\
                0 & 0 & 0 & 0 & 0 & 0 \\
                0 & 0 & k_{33} & k_{34} & k_{35} & 0 \\
                0 & 0 & k_{43} & k_{44} & k_{45} & 0 \\
                0 & 0 & k_{53} & k_{54} & k_{55} & 0 \\
                0 & 0 & 0 & 0 & 0 & 0 \\
             \end{bmatrix}

This matrix is symmetric so :math:`k_{43} = k_{34}`, :math:`k_{53} = k_{35}` and :math:`k_{54} = k_{45}` and only six coefficients needs to be provided, with the command::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -hs id k33 k44 k55 k34 k35 k45

where *id* is the index of the body.

Mass matrix
-----------

It is possible to define a mass matrix for each body. It is used for evaluating the Response Amplitude Operators. This matrix is expressed at the center of gravity of the body, where the hydrodynamic database was computed. It is defined by:

.. math::

    M_G = \begin{bmatrix} m & 0 & 0 & 0 & 0 & 0 \\
                          0 & m & 0 & 0 & 0 & 0 \\
			  0 & 0 & m & 0 & 0 & 0 \\
                          0 & 0 & 0 & I_{44} & I_{45} & I_{46} \\ 
                          0 & 0 & 0 & I_{54} & I_{55} & I_{56} \\ 
    			  0 & 0 & 0 & I_{64} & I_{65} & I_{66} \end{bmatrix}

This matrix is symmetric so :math:`I_{45} = I_{54}`, :math:`I_{46} = I_{64}` and :math:`I_{56} = I_{65}` and only six coefficients needs to be given, using the command::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -i id m I44 I55 I66 I45 I46 I56

where *id* is the index of the body.

The mass and the inertia matrix may be defined separately::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -m id m
    >$ hdb5tool -cal path/to/Nemoh/cal/folder -io id I44 I55 I66 I45 I46 I56

Symmetrization of the hydrodynamic database
-------------------------------------------

If the frequency-domain solver was run by defining the wave directions between :math:`0^{\circ}` and :math:`180^{\circ}`, it is necessary to symmetrize the diffraction loads and the Froude-Krylov loads from :math:`0^{\circ}` to :math:`360^{\circ}`. This is achieved by the command::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -sym

Writing a *.hdb5* file
-----------------------

To write a *.hdb5* file, the command is::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -w path/to/hdb5/file

.. note::

    The extension *.hdb5* is mandatory for the ouput file.

Plots
-----

Added mass and damping coefficients
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The command to plot the added mass and damping coefficients of the body *ibody_force* along the force *iforce* for a motion of *ibody_motion* along the degree of freedom *idof* is::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -pab ibody_force iforce ibody_motion idof

.. note::

    The red cross represents the infinite added-mass coeffcient.

Diffraction loads
~~~~~~~~~~~~~~~~~

The command to plot the amplitude and the phase of the diffraction loads of the body *ibody* along the force *iforce* for the wave direction *iwave* is::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -pdiff ibody iforce iwave

Froude-Krylov loads
~~~~~~~~~~~~~~~~~~~

The command to plot the amplitude and the phase of the Froude-Krylov loads of the body *ibody* along the force *iforce* for the wave direction *iwave* is::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -pfk ibody iforce iwave

Excitation loads
~~~~~~~~~~~~~~~~

The command to plot the amplitude and the phase of the excitation loads of the body *ibody* along the force *iforce* for the wave direction *iwave* is::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -pe ibody iforce iwave

Impulse response functions
~~~~~~~~~~~~~~~~~~~~~~~~~~

The command to plot the impulse response function of the body *ibody_force* along the force *iforce* for a motion of *ibody_motion* along the degree of freedom *idof* is::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -pirf ibody_force iforce ibody_motion idof

For the impulse response functions with forward speed, use ``-pirf_speed`` instead of ``-pirf``.

Mesh
~~~~

The command to plot the mesh of the body *ibody* is::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -pm ibody

Meshes
~~~~~~

The command to plot all the meshes is::

    >$ hdb5tool -cal path/to/Nemoh/cal/folder -pms

Example of use
--------------

Let us condsider a floating sphere of radius :math:`1` \\(m\\) with a draft of :math:`1` \\(m\\). The main properties of the sphere are presented in the next table:

========================= ==================================
Parameters                Values
========================= ==================================
Radius                    :math:`1` \\(m\\)
Initial sphere location   (:math:`0`, :math:`0`, :math:`0`)
Center of gravity         (:math:`0`, :math:`0`, :math:`0`)
Mass	                  :math:`2094.39` \\(kg\\)
Ixx                       :math:`837.76` \\(kg.m^2\\) 
Iyy                       :math:`837.76` \\(kg.m^2\\)
Izz                       :math:`837.76` \\(kg.m^2\\)
K33                       :math:`3.082\times10^4` \\(N/m\\)
K44                       :math:`1.699\times10^1` \\(N.m\\)
K55                       :math:`1.699\times10^1` \\(N.m\\)
========================= ==================================

We want to generate the corresponding *.hdb5* file, named *Sphere.hdb5*, after doing a new discretization of the hydrodynamic database for 41 wave directions, 150 wave frequencies, computing the impulse reponse functions with a final time of :math:`100` \\(s\\) and a time step of :math:`0.01` \\(s\\), providing the hydrostatic and inertia matrices and applying a cutoff scaling function with a cutoff time of :math:`10` \\(s\\) to :math:`K_{33}` and :math:`{Ku}_{33}`. We also want to plot the following quantities: :math:`A_{33}`, :math:`B_{44}`, :math:`F^{Diff}_{3}`, :math:`F^{Diff}_{4}`, :math:`F^{FK}_{4}`, :math:`F^{Exc}_{3}`, :math:`F^{Exc}_{4}`, :math:`K_{33}` and :math:`{Ku}_{44}`. The command is:: 

    hdb5tool -cal . -dw 41 -df 150 -ft 100 -dt 0.01 -hs 1 3.082e4 1.699e1 1.699e1 0 0 0 -i 1 2094.39 837.76 837.76 837.76 0 0 0 -sym -w Sphere.hdb5 -pab 1 3 1 3 -prad 1 4 1 4 -pd 1 3 1 -pd 1 4 1 -pfk 1 3 1 -pfk 1 4 1 -pe 1 3 1 -pe 1 4 1 -pirf 1 3 1 3 -pirf_speed 1 4 1 4 -coirf 10 1 3 1 3 -coirf_speed 10 1 3 1 3 -pm 1
