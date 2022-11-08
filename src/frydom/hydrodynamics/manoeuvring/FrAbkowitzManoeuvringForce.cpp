//
// Created by frongere on 12/10/2021.
//

#include "FrAbkowitzManoeuvringForce.h"
#include "frydom/logging/FrEventLogger.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/utils/FrFileSystem.h"
#include "frydom/hydrodynamics/manoeuvring/FrHullResistance.h"

namespace frydom {


  FrAbkowitzManoeuvringForce::FrAbkowitzManoeuvringForce(const std::string& name, FrBody* body,
                                                         const FrHydroDerivatives& hydroDerive,
                                                         const std::shared_ptr<FrHullResistanceForce>& hullResistanceForce,
                                                         const Position& mid_ship, double draft_m, double lpp_m) :
       FrForce(name, "FrManoeuvringForce", body), m_mid_ship(mid_ship),
       m_draft(draft_m), m_Lpp(lpp_m), m_hullResistanceForce(hullResistanceForce)
       {
    // Set hydrodynamic derivatives
    m_Xvv   = hydroDerive.m_Xvv;
    m_Xvvvv = hydroDerive.m_Xvvvv;
    m_Xvr   = hydroDerive.m_Xvr;
    m_Xrr   = hydroDerive.m_Xrr;
    m_Yv    = hydroDerive.m_Yv;
    m_Yr    = hydroDerive.m_Yr;
    m_Yvvv  = hydroDerive.m_Yvvv;
    m_Yvvr  = hydroDerive.m_Yvvr;
    m_Yvrr  = hydroDerive.m_Yvrr;
    m_Yrrr  = hydroDerive.m_Yrrr;
    m_Nv    = hydroDerive.m_Nv;
    m_Nr    = hydroDerive.m_Nr;
    m_Nvvv  = hydroDerive.m_Nvvv;
    m_Nvvr  = hydroDerive.m_Nvvr;
    m_Nvrr  = hydroDerive.m_Nvrr;
    m_Nrrr  = hydroDerive.m_Nrrr;
    // Deactivate hull force to not accounted twice
    m_hullResistanceForce->SetActive(false);
  }

  void FrAbkowitzManoeuvringForce::Initialize() {
    FrForce::Initialize();
  }

  double FrAbkowitzManoeuvringForce::GetUMin() const {
    return m_hullResistanceForce->GetUMin();
  }

  double FrAbkowitzManoeuvringForce::GetUMax() const {
    return m_hullResistanceForce->GetUMax();
  }

  void FrAbkowitzManoeuvringForce::Compute(double time) {

    auto body = GetBody();
    auto rho = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

    // Get the velocity at mid-ship
    Velocity vessel_vel_world = body->GetVelocityInWorldAtPointInBody(m_mid_ship, NWU);
    auto vessel_angular_vel_world = body->GetAngularVelocityInWorld(NWU);

    // Projection to the planar frame
    Direction vessel_x_axis_world = body->GetFrame().GetXAxisInParent(NWU);
    vessel_x_axis_world[2] = 0;
    vessel_x_axis_world.normalize();

    Direction vessel_y_axis_world = body->GetFrame().GetYAxisInParent(NWU);
    vessel_y_axis_world[2] = 0;
    vessel_y_axis_world.normalize();

    double u = vessel_vel_world.dot(vessel_x_axis_world);
    double v = vessel_vel_world.dot(vessel_y_axis_world);
    double r = vessel_angular_vel_world.GetWz();

    // Adim
    double vnorm2 = u*u + v*v;
    if (vnorm2 == 0.)
      return;

    double vnorm = std::sqrt(vnorm2);
    auto q = 0.5 *rho * m_Lpp * m_draft * vnorm2;
    u = u/vnorm;
    v = v/vnorm;
    r = r*m_Lpp/vnorm;

    // Compute hydrodynamic derivatives
    double c_Xvv = m_Xvv * v*v;
    double c_Xvr = m_Xvr * v*r;
    double c_Xrr = m_Xrr * r*r;
    double c_Xvvvv = m_Xvvvv * pow(v, 4);

    double c_Yv = m_Yv * v;
    double c_Yr = m_Yr * r;
    double c_Yvvv = m_Yvvv * pow(v, 3);
    double c_Yvvr = m_Yvvr * v*v*r;
    double c_Yvrr = m_Yvrr * v*r*r;
    double c_Yrrr = m_Yrrr * r*r*r;

    double c_Nv = m_Nv * v;
    double c_Nr = m_Nr * r;
    double c_Nvvv = m_Nvvv * pow(v, 3);
    double c_Nvvr = m_Nvvr * v*v*r;
    double c_Nvrr = m_Nvrr * v*r*r;
    double c_Nrrr = m_Nrrr * pow(r, 3);

    // Compute force and torque
    auto X = q * (c_Xvv + c_Xvr + c_Xrr + c_Xvvvv) + m_hullResistanceForce->Rh(u * vnorm);
    auto Y = q * (c_Yv + c_Yr + c_Yvvv + c_Yvvr + c_Yvrr + c_Yrrr);
    auto N = q * m_Lpp * (c_Nv + c_Nr + c_Nvvv + c_Nvvr + c_Nvrr + c_Nrrr);

    Force forceInWorld = X * vessel_x_axis_world + Y * vessel_y_axis_world;
    Torque torqueInWorld = {0., 0., N};

    SetForceTorqueInWorldAtPointInBody(forceInWorld, torqueInWorld, m_mid_ship, NWU);

  }

  void FrAbkowitzManoeuvringForce::DefineLogMessages() {
    FrForce::DefineLogMessages();
  }

  // ---------------------------------------------------------------------
  // MAKER FORCE
  // ---------------------------------------------------------------------

  std::shared_ptr<FrAbkowitzManoeuvringForce>
      make_abkowitz_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody> body,
                                      const FrHydroDerivatives& hydroDerive,
                                      const std::shared_ptr<FrHullResistanceForce>& hullResistanceForce,
                                      const Position& mid_ship, double draft_m, double lpp_m) {
    auto force = std::make_shared<FrAbkowitzManoeuvringForce>(name, body.get(), hydroDerive, hullResistanceForce,
                                                              mid_ship, draft_m, lpp_m);
    body->AddExternalForce(force);
    return force;
  }

}  // end namespace frydom
