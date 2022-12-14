//
// Created by frongere on 03/12/2019.
//

#include "FrPathPolicies.h"


namespace frydom {

  class FrOffshoreSystem;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrOffshoreSystem, "FRYDOM_");

  class FrBody;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrBody, "BODIES/BODY_");

  class FrForce;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrForce, "FORCES/FORCE_");

  class FrNode;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrNode, "NODES/NODE_");

  class FrLink;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrLink, "LINKS/LINK_");

  class FrConstraint;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrConstraint, "CONSTRAINTS/CONSTRAINT_");

  class FrActuator;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrActuator, "ACTUATORS/ACTUATOR_");

  // NODES

  class FrRootNode;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrRootNode, "ROOT_");

  class FrFEACable;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrFEACable, "CABLES/DYN_CABLE_");

  class FrClumpWeight;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrClumpWeight, "CABLES/CLUMP_WEIGHT_");

  class FrCatenaryLineBase;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrCatenaryLineBase, "CABLES/CATENARY_CABLE_");

  class FrLumpedMassCable;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrLumpedMassCable, "CABLES/LUMPED_MASS_CABLE_");

  class FrEquilibriumFrame;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrEquilibriumFrame, "")

  class FrRadiationModel;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrRadiationModel, "")

  class FrHydroMesh;

  TYPE_TO_NORMALIZED_PATH_PREFIX(FrHydroMesh, "")

  class FrMorisonCompositeElement;
  TYPE_TO_NORMALIZED_PATH_PREFIX(FrMorisonCompositeElement, "");

  namespace internal {
    class FrFEALinkBase;
  }
  TYPE_TO_NORMALIZED_PATH_PREFIX(internal::FrFEALinkBase, "LINKS/FEA_LINKS_")

}  // end namespace frydom
