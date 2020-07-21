//
// Created by frongere on 29/10/19.
//


#include "FrTypeNames.h"


namespace frydom {

  class FrOffshoreSystem;

  TYPE_TO_STRING(FrOffshoreSystem, "OffshoreSystem")


  class FrBody;

  TYPE_TO_STRING(FrBody, "Body")

  class FrNode;

  TYPE_TO_STRING(FrNode, "Node")

  class FrEquilibriumFrame;

  TYPE_TO_STRING(FrEquilibriumFrame, "EquilibriumFrame")


  // CABLES

  class FrFEACable;

  TYPE_TO_STRING(FrFEACable, "DynamicCable")

  class FrCatenaryLine;

  TYPE_TO_STRING(FrCatenaryLine, "CatenaryLine")

  class FrCatenaryLineSeabed;

  TYPE_TO_STRING(FrCatenaryLineSeabed, "CatenaryLineSeabed")

  class FrClumpWeight;

  TYPE_TO_STRING(FrClumpWeight, "ClumpWeight");

  // FORCES

  class FrConstantForce;

  TYPE_TO_STRING(FrConstantForce, "ConstantForce")

  class FrCatenaryForce;

  TYPE_TO_STRING(FrCatenaryForce, "CatenaryCableForce")

  class FrLinearDamping;

  TYPE_TO_STRING(FrLinearDamping, "LinearDampingForce")

  class FrWaveDriftForce;

  TYPE_TO_STRING(FrWaveDriftForce, "SecondOrderWaveDriftForce")

  class FrLinearDiffractionForce;

  TYPE_TO_STRING(FrLinearDiffractionForce, "LinearDiffractionForce")

  class FrLinearFroudeKrylovForce;

  TYPE_TO_STRING(FrLinearFroudeKrylovForce, "LinearFroudeKrylovForce")

  class FrLinearExcitationForce;

  TYPE_TO_STRING(FrLinearExcitationForce, "LinearExcitationForce")

  class FrCurrentForce;

  TYPE_TO_STRING(FrCurrentForce, "CurrentForce")

  class FrWindStandardForce;

  TYPE_TO_STRING(FrWindStandardForce, "WindStandardForce")

  class FrCurrentStandardForce;

  TYPE_TO_STRING(FrCurrentStandardForce, "CurrentStandardForce")

  class FrNonLinearFroudeKrylovForce;

  TYPE_TO_STRING(FrNonLinearFroudeKrylovForce, "NonLinearFroudeKrylovForce")

  class FrITTCResistance;

  TYPE_TO_STRING(FrITTCResistance, "ITTCResistanceForce")

  class FrMorisonForce;

  TYPE_TO_STRING(FrMorisonForce, "MorisonForce")

  class FrManDampingTaylorExpansion;

  TYPE_TO_STRING(FrManDampingTaylorExpansion, "ManeuveringForce")

  class FrLinearHydrostaticForce;

  TYPE_TO_STRING(FrLinearHydrostaticForce, "LinearHydrostaticForce")

  class FrWindForce;

  TYPE_TO_STRING(FrWindForce, "WindForce")

  class FrQuadraticDamping;

  TYPE_TO_STRING(FrQuadraticDamping, "QuadraticDampingForce")

  class FrRadiationForce;

  TYPE_TO_STRING(FrRadiationForce, "LinearRadiationForce")

  class FrNonlinearHydrostaticForce;

  TYPE_TO_STRING(FrNonlinearHydrostaticForce, "NonLinearHydrostaticForce")


  // LINKS

  class FrPrismaticLink;

  TYPE_TO_STRING(FrPrismaticLink, "PrismaticLink")

  class FrCylindricalLink;

  TYPE_TO_STRING(FrCylindricalLink, "CylindricalLink")

  class FrDOFMaskLink;

  TYPE_TO_STRING(FrDOFMaskLink, "DOFMaskLink")

  class FrSphericalLink;

  TYPE_TO_STRING(FrSphericalLink, "SphericalLink")

  class FrFixedLink;

  TYPE_TO_STRING(FrFixedLink, "FixedLink")

  class FrRevoluteLink;

  TYPE_TO_STRING(FrRevoluteLink, "RevoluteLink")

  class FrPrismaticRevoluteLink;

  TYPE_TO_STRING(FrPrismaticRevoluteLink, "PrismaticRevoluteLink")


  // CONSTRAINTS

  class FrConstraintPointOnPlane;

  TYPE_TO_STRING(FrConstraintPointOnPlane, "ConstraintPointOnPlane")

  class FrConstraintPointOnLine;

  TYPE_TO_STRING(FrConstraintPointOnLine, "ConstraintPointOnLine")

  class FrConstraintDistanceBetweenPoints;

  TYPE_TO_STRING(FrConstraintDistanceBetweenPoints, "ConstraintDistanceBetweenPoints")

  class FrConstraintPerpendicular;

  TYPE_TO_STRING(FrConstraintPerpendicular, "ConstraintPerpendicular")

  class FrConstraintParallel;

  TYPE_TO_STRING(FrConstraintParallel, "ConstraintParallel")

  class FrConstraintPlaneOnPlane;

  TYPE_TO_STRING(FrConstraintPlaneOnPlane, "ConstraintPlaneOnPlane")

  class FrConstraintDistanceToAxis;

  TYPE_TO_STRING(FrConstraintDistanceToAxis, "ConstraintDistanceToAxis")


  // ACTUATORS

  class FrLinearActuator;

  TYPE_TO_STRING(FrLinearActuator, "LinearActuator")

  class FrAngularActuator;

  TYPE_TO_STRING(FrAngularActuator, "AngularActuator")

  // LUMPED MASS CABLE

  class FrLumpedMassCable;

  TYPE_TO_STRING(FrLumpedMassCable, "LumpedMassCable")

  // MORISON ELEMENTS
  class FrMorisonCompositeElement;
  TYPE_TO_STRING(FrMorisonCompositeElement, "MorisonElements");

}

