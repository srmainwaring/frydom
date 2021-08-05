//
// Created by frongere on 06/08/2021.
//

#include "acme/acme.h"

using namespace acme;

int main() {

  ThrusterBaseParams params;
  params.m_diameter_m;
  params.m_screw_direction;

  params.m_hull_wake_fraction_0;
  params.m_thrust_deduction_factor_0;

  // TODO: Donnees a ajouter dans le json
  params.m_use_advance_velocity_correction_factor = false;
  params.m_propeller_design_rpm;
  params.m_vessel_design_speed_ms;

  params.m_thrust_corr = 0.;
  params.m_torque_corr = 0.;



  auto prop = FPP1Q(params, "");


  return 0;
}