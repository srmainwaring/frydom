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
