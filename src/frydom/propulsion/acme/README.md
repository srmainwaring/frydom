ACME: ACtuators ModEls

This library aims at implementing marine actuators models with an agnostic approach and minimal dependencies.

Thrusters:
  - First quadrant Fixed Pitch Propeller model (FPP1Q)
  - Four quadrant Fixed Pitch Propeller model (FPP4Q)
  - Controllable Pitch Propeller model (CPP)

Rudder:
  - Simple rudder model
  - Flap rudder model

Sail:
  - Flettner rotor
  - ...

The philosophy is here to provide stateless classes for model implementation and lazy initialization.
Model data must be provided under the form of json string and not json files.


TODO:
On developpera un outil en ligne de commande permettant d'aider au calage des differents coefficients correcteurs
(wake fraction, thrust deduction factor, corrections sur les coefficients de thrust et torque etc...) suivant une
methodologie a developper.
On pourra voir si l'apport d'algo d'optimisation peut permettre d'automatiser certaines choses.

Quoiqu'il en soit, lors des etudes, il conviendra de demander des informations supplementaires sur les navires a etudier
telles que la vitesse de design, les rpm de design.
