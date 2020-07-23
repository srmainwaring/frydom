.. _radiation:

Radiation force
***************

Linear theory
=============

.. _impulse_response_and_convolution:

Following Cummin's method, the added mass and radiation damping loads on a body or group of bodies are computed, in the time
domain, using the following convolution integral equation:

.. math::
    \mathbf{f}_{R}(t) = - \mathbf{A}_{\infty} \mathbf{\ddot{x}}(t) - \int_0^t \mathbf{K}(t-\tau) \mathbf{\dot{x}}(\tau) d\tau

The impulse response function :math:`\mathbf{K}(t)` account for the past motion of the body, while the infinite added mass
:math:`\mathbf{A}_{\infty}` gives the body's instantaneous response to acceleration. All these terms are to be computed using the
frequency-dependant added mass and radiation damping coefficients.

Impulse response function (IRF)
-------------------------------

The infinite added mass and damping coefficients are given by a linear potential flow based solver. The impulse response can be
computed from the frequency-domain damping coefficients.

.. math::
    \mathbf{K}(t) = c(t) \frac{2}{\pi} \int_0^{\infty} \mathbf{B}(\omega) \cos(\omega t) d\omega

where

- :math:`\mathbf{B}(\omega)` is the frequency-dependant damping matrix, at circular frequency :math:`\omega`,
- :math:`c(t)` is a cutoff scaling function

Recursive convolution
---------------------

An alternative to the classic convolution based on the IRF is the recursive convolution. It is based on the establishment of
macromodels in pole-residu form, using rational curve fitting algorithm, such as Vector Fitting [Grivet-Talocia]_.

For the hydrodynamic problem, it yields the following transfer function

.. math::
    K(s) = B(\omega) + s \left(A(\omega) - A_\infty \right) = \sum_{j=0}^{N_1} \dfrac{R_j^\mathbb{R}}{s-q_j^\mathbb{R}}
    + \sum_{j=0}^{N_2} \left( \dfrac{R_j^\mathbb{C}}{s-q_j^\mathbb{C}} + \dfrac{{R_j^\mathbb{C}}^*}{s-{q_j^\mathbb{C}}^*} \right)

The impulse response of this system is

.. math::
    k(t) &=& \mathscr{L}^{-1} \{ K(s) \} = \sum_{j=0}^{N_1} R_j^\mathbb{R} e^{q_j^\mathbb{R}t} \theta(t) + \sum_{j=0}^{N_2} \left( R_j^\mathbb{C} e^{q_j^\mathbb{C}t} + {R_j^\mathbb{C}}^* e^{{q_j^\mathbb{C}}^*t}  \right)\theta(t) \\
         &=& \sum_{j=0}^{N_1} R_j^\mathbb{R} e^{q_j^\mathbb{R}t} \theta(t) + 2 \sum_{j=0}^{N_2} \Re \left(R_j^\mathbb{C}e^{q_j^\mathbb{C}t}\theta(t) \right)

and the convolution can be written

.. math::
    v(t) = \int_0^t K(t-\tau) \dot{x}(\tau) d\tau = \sum_{j=0}^{N_1} R_j^\mathbb{R} u_j^\mathbb{R}(t) + 2 \sum_{j=0}^{N_2} \Re \left\{ R_j^\mathbb{C} u_j^\mathbb{C}(t)  \right)

where :math:`u_j(t)` is an auxiliary state variable

.. math::
    u_j(t) = \int_0^t e^{q_j(t-\tau)}\dot{x}(\tau) d\tau

for a time :math:`t = t_k`, this state variable can be derived as

.. math::
    u_j(t_k) &=& \int_0^{t_k} e^{q_j(t-\tau)}\dot{x}(\tau) d\tau \\
             &=& \int_0^{t_{k-1}} e^{q_j(t_k-\tau)}\dot{x}(\tau) d\tau + \int_{t_k-1}^{t_{k}} e^{q_j(t_k-\tau)}\dot{x}(\tau) d\tau\\
             &=& e^{q_j \Delta t} \int_0^{t_{k-1}} e^{q_j(t_{k-1}-\tau)}\dot{x}(\tau) d\tau + \int_{t_k-1}^{t_{k}} e^{q_j(t_k-\tau)}\dot{x}(\tau) d\tau\\
             &=& e^{q_j \Delta t} u_j(t_{k-1}) + \int_{t_k-1}^{t_{k}} e^{q_j(t_k-\tau)}\dot{x}(\tau) d\tau

The recursive form of this convolution is highlighted here : the state at time :math:`t_k` is expressed from its state at :math:`t_{k-1}`
plus a term that takes into account the variation of :math:`\dot{x}` on :math:`[t_{k-1}, t_k]` weighted by :math:`e^{q_j(t_k-\tau)}`,
which results in a lighter convolution implementation than the IRF one.

The discretization of the remaining convolution depends on the approximation : Trapezoidal rule, piecewise constant
approximation or piecewise linear approximation. The discrete-time estimates denoted with a hat :math:`\hat{}`, all
approximations can however be summed up as

.. math::
     \hat{u}_k = \alpha \hat{u}_{k-1} + \beta_0 \hat{\dot{x}}_{k-1} + \beta_1 \hat{\dot{x}}_k

where :math:`\alpha = e^{q \Delta t}`

Trapezoidal rule:
+++++++++++++++++
.. math::
   \begin{cases}
        \beta_0 = \dfrac{\Delta t}{2} e^{q \Delta t} \\
        \beta_1 = \dfrac{\Delta t}{2}
    \end{cases}

Be careful to fulfill the condition :math:`|p \Delta t|\ll 1`, to ensure that the approximation is sufficiently accurate.

Piecewise constant approximation:
+++++++++++++++++++++++++++++++++
.. math::
    \hat{u}_k = e^{q \Delta t} \hat{u}_{k-1} + \dfrac{e^{q \Delta t} - 1}{q} U_k

The constant :math:`U_k` can be defined as :math:`\hat{\dot{x}}_{k-1}`, :math:`\hat{\dot{x}}_k`, or a combination of these two values.

Piecewise linear approximation:
+++++++++++++++++++++++++++++++
.. math::
   \begin{cases}
        \beta_0 = \dfrac{1 + (q\Delta t-1)e^{q \Delta t}}{q^2\Delta t} \\
        \beta_1 = \dfrac{-1 -q\Delta t + e^{q \Delta t}}{q^2\Delta t}
    \end{cases}

Cutoff scaling function
-----------------------

The convolution involving the impulse response function requires to integrate from the start of the simulation, which leads
to an increasing cpu cost as the simulation progresses. However, since the impulse response function tends to decay to zero,
it is possible to cutoff the responses from the past motion of the body. Truncating the impulse response function may introduce
numerical errors and negative damping, with energy fed in the simulation. Scaling the impulse response function instead
ensure to limit the negative damping phenomena.

A cutoff scaling function can be applied when loading results from the linear potential flow based solver into the hydrodynamic
database, using the python scripts. The cutoff function is based on an exponential function:

.. math::
    c(t) = \exp\left(-\dfrac{3t}{T_c}\right)^2

where :math:`T_c` is the cutoff time.


Effect of constant forward speed on the radiation force
-------------------------------------------------------

When steady forward speed is considered, coupling effect between the yaw and pitch motion with the heave and sway force appears in the radiation model. In the case of slender body, the radiation convolution model presented in previous section can be modified to take into this effect [Rongere]_.

.. math::
    \mathbf{f}_R(t) = -\mathbf{A}(\infty, \mathbf{U}) \mathbf{\ddot{x}}(t) - \mathbf{B}(\infty, \mathbf{U})\mathbf{\dot{x}}(t) - \int_0^t \mathbf{K}(t-\tau, \mathbf{U}) \mathbf{\dot{x}}(\tau) d\tau

where :math:`\mathbf{B}(\infty, \mathbf{U})` is the linear damping term at infinity and the convolution term :math:`\mu(t)` :

.. math::
    \mu(t) = \int_0^t \mathbf{K}(t-\tau, \mathbf{U}) \mathbf{\dot{x}}(\tau) d\tau

The memory term can be decomposed into two functions :math:`\mu_0(t)` and :math:`\mu_U(t)` independent of :math:`\mathbf{U}` :

.. math::
    \mu(t) = \mu_0(t) + \mathbf{U} \mu_U(t)

with,

.. math::
    \mu_0(t) = \frac{2}{\pi} \int_0^{\infty} \mathbf{B}_0(\omega) \cos(\omega t) d\omega \\
    \mu_U(t) = \frac{2}{\pi} \int_0^{\infty} (\mathbf{A}_0(\infty) - \mathbf{A}_0(\omega)) \mathbf{L} \cos(\omega t) d\omega

where,

.. math::
    \mathbf{L} = \left( \begin{array}{cccc}
    0 & \ldots & 0 & 0 \\
    0 & \ldots & 0 & 1 \\
    0 & \ldots & -1 & 0 \\
    \vdots & \ddots & \vdots & \vdots \\
    0 & \ldots & 0 & 0 \\
    \end{array} \right)

The decomposition of the linear damping term takes the following forms:

.. math::
    \mathbf{B}_{\infty}(\mathbf{U}) &=& \lim\limits_{\omega \rightarrow +\infty} \mathbf{B}(\omega, \mathbf{U}) \\
        &=& \lim\limits_{\omega \rightarrow +\infty} \left( \mathbf{B}_0(\omega) + \mathbf{U} . \mathbf{B}_U(\omega) \right) \\
        &=& \lim\limits_{\omega \rightarrow +\infty} -\mathbf{U} \mathbf{A}_0(\omega) \mathbf{L} \\
        &=& -\mathbf{U} \mathbf{A}_0(\infty) \mathbf{L}


Reference:

.. [Rongere] F. Rongère, J.M. Kobus, A. Babarit, G. Delhommeau, "Comparative study of two methods to compute the radiation forces for a rowing application", 12eme Journées de l'Hydrodynamique, nantes, 17-19 novembre 2010
.. [Grivet-Talocia] S. Grivet-Talocia, B. Gustavsen, "Passive Macromodeling: Theory and Applications", John Wiley \& Sons, 2015, chapter 11


