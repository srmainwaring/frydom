//
// Created by lletourn on 01/07/2021.
//

#include "FrHullResistance.h"
#include "MathUtils/Unit.h"

namespace frydom {

  // -------------------------------------------------------------------------
  // Hull resistance force
  // -------------------------------------------------------------------------

  FrHullResistanceForce::FrHullResistanceForce(const std::string& name, FrBody* body, const Position& mid_ship)
    : FrForce(name, "FrHullResistanceForce", body), m_mid_ship(mid_ship) {}

  void FrHullResistanceForce::Initialize() {
    FrForce::Initialize();
  }

  void FrHullResistanceForce::Compute(double time) {

    auto body = GetBody();

    // Get the velocity at mid-ship
    Velocity vessel_vel_world = body->GetVelocityInWorldAtPointInBody(m_mid_ship, NWU);
    auto vessel_angular_vel_world = body->GetAngularVelocityInWorld(NWU);

    // Projection to the planar frame
    Direction vessel_x_axis_world = body->GetFrame().GetXAxisInParent(NWU);
    vessel_x_axis_world[2] = 0;
    vessel_x_axis_world.normalize();

    double u = vessel_vel_world.dot(vessel_x_axis_world);

    Force forceInWorld = -Rh(u) * vessel_x_axis_world;

    SetForceTorqueInWorldAtPointInBody(forceInWorld, Torque(), m_mid_ship, NWU);
  }

  // -----------------------------------------------------------------------------
  // Interp hull resistance
  // -----------------------------------------------------------------------------

  FrInterpHullResistanceForce::FrInterpHullResistanceForce(const std::string& name, FrBody* body,
                                                           const Position& mid_ship,
                                                           const std::shared_ptr<std::vector<double>> &u,
                                                           const std::shared_ptr<std::vector<double>> &Rh)
    : FrHullResistanceForce(name, body, mid_ship)
  {
    m_data.Initialize(u, Rh);
  }

  double FrInterpHullResistanceForce::Rh(double u) const {
    return m_data.Eval(u);
  }

  double FrInterpHullResistanceForce::GetUMin() const {
    return m_data.GetXmin();
  }

  double FrInterpHullResistanceForce::GetUMax() const {
    return m_data.GetXmax();
  }

  std::shared_ptr<FrInterpHullResistanceForce> make_interp_hull_resistance(
      const std::string& name, FrBody* body, const Position& mid_ship,
      const std::shared_ptr<std::vector<double>>& u, const std::shared_ptr<std::vector<double>>& Rh)
  {
    auto force = std::make_shared<FrInterpHullResistanceForce>(name, body, mid_ship, u, Rh);
    body->AddExternalForce(force);
    return force;
  }


  // --------------------------------------------------------------------------
  // Quadratic hull resistance
  // --------------------------------------------------------------------------

  FrQuadHullResistanceForce::FrQuadHullResistanceForce(const std::string& name, FrBody* body, const Position& mid_ship,
                                                       double a_pos, double a_neg, double b_pos, double b_neg,
                                                       double lpp_m):
      FrHullResistanceForce(name, body, mid_ship),
      m_a_pos(a_pos), m_a_neg(a_neg), m_b_pos(b_pos), m_b_neg(b_neg),
      m_lpp(lpp_m) {}

  double FrQuadHullResistanceForce::Rh(double u) const {
    double rh;
    if (u>=0) {
      rh = m_a_pos * u * u + m_b_pos * u;
    } else {
      rh = m_a_neg * u * u + m_b_neg * u;
    }
    return rh;
  }

  double FrQuadHullResistanceForce::GetUMin() const {
    return - 80 * MU_KNOT;
  }

  double FrQuadHullResistanceForce::GetUMax() const {
    return 80 * MU_KNOT;
  }

  std::shared_ptr<FrQuadHullResistanceForce> make_quadratic_hull_resistance(
      const std::string& name, FrBody* body, const Position& mid_ship,
      double a_pos, double a_neg, double b_pos, double b_neg, double lpp_m) {
    auto force = std::make_shared<FrQuadHullResistanceForce>(name, body, mid_ship,
                                                             a_pos, a_neg, b_pos, b_neg,
                                                             lpp_m);
    body->AddExternalForce(force);
    return force;
  }

} // end namespace frydom