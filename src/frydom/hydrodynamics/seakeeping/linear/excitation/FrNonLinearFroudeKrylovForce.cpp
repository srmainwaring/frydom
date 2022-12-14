// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#include "FrNonLinearFroudeKrylovForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"

#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"
#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  /// This class defines the nonlinear hydrostatic force applied to a hydrodynamic body.
  /// The non linear hydrostatic force is computed by integrating the hydrostatic pressure (-rho g z) over the wetted
  /// body surface. This surface is defined using the body mesh, clipped by the free surface if the body is floating.
  /// 2 levels of approximation are defined for the free surface position
  /// - the mean water level : z = 0
  /// - the incident wave position : z = eta_i
  /// Be careful that in the second approximation, the hydrostatic force is no longer equals to rho g Vdisp vec(z)
  /// and contains some horizontal components. The hydrostatic force also does not apply on the buoyancy center, B,
  /// which means M_B (HS) != 0 (see technical report : non linear hydrostatics and Froude-Krylov)

  FrNonLinearFroudeKrylovForce::FrNonLinearFroudeKrylovForce(const std::string &name,
                                                             FrBody *body,
                                                             std::shared_ptr<FrHydroMesh> HydroMesh) :
      FrForce(name, TypeToString(this), body) {
    m_hydroMesh = HydroMesh;
  }

  void FrNonLinearFroudeKrylovForce::Compute(double time) {

    // This function computes the fully or weakly nonlinear Froude-Krylov forces from the pressure integration.

    Force FKforce = {};
    Torque FKtorque = {};

    auto body = GetBody();

    Position NormalPos;

    auto bodyPos = body->GetPosition(NWU);
    bodyPos.GetZ() = 0;

    auto clippedMesh = &(m_hydroMesh->GetClippedMesh());

    auto waveField = body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

    // Loop over the faces.
    for (auto &f_iter : clippedMesh->faces()) {

      // Normal
      NormalPos.GetX() = clippedMesh->normal(f_iter)[0];
      NormalPos.GetY() = clippedMesh->normal(f_iter)[1];
      NormalPos.GetZ() = clippedMesh->normal(f_iter)[2];

      // Centroid (where the pressure is evaluated).
      Position Centroid = mesh::OpenMeshPointToVector3d<Position>(clippedMesh->data(f_iter).Center());

      // Incident pressure.
      // The pressure is assumed constant over a panel.
      auto Pressure = waveField->GetPressure(Centroid + bodyPos, NWU);

      // Area.
      double Area = clippedMesh->GetArea(f_iter);

      // Pressure * Area.
      double PA = -Pressure * Area;

      // Froude-Krylov force.
      FKforce += PA * NormalPos;

      // Froude-Krylov torque.
      FKtorque += PA * Centroid.cross(NormalPos);

    }

    Position meshPos = body->GetPosition(NWU);
    meshPos.GetZ() = 0;

    SetForceTorqueInWorldAtPointInWorld(FKforce, FKtorque, meshPos, NWU);

  }

  std::shared_ptr<FrNonLinearFroudeKrylovForce>
  make_nonlinear_froude_krylov_force(const std::string &name,
                                     std::shared_ptr<FrBody> body,
                                     std::shared_ptr<FrHydroMesh> HydroMesh) {

    // This function creates a fully or weakly nonlinear Froude-Krylov force object.

    // Construction of the fully or weakly Froude-Krylov force object from the HDB.
    auto NonlinFKForce = std::make_shared<FrNonLinearFroudeKrylovForce>(name, body.get(), HydroMesh);

    // Add the Froude-Krylov force object as an external force to the body.
    body->AddExternalForce(NonlinFKForce); // Initialization of m_body.

    return NonlinFKForce;

  }

}  // end namespace frydom
