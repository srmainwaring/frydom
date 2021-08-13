//
// Created by frongere on 09/08/2021.
//


#include "PropellerRudder.h"

namespace acme {

  template<class Propeller, class Rudder>
  PropellerRudder<Propeller, Rudder>::PropellerRudder(const PropellerParams &thruster_params,
                                                      const std::string &thruster_perf_data_json_string,
                                                      const RudderParams &rudder_params,
                                                      const std::string &rudder_perf_data_json_string) :
      m_propeller(std::make_unique<Propeller>(thruster_params, thruster_perf_data_json_string)),
      m_rudder(std::make_unique<Rudder>(rudder_params, rudder_perf_data_json_string)) {}

  template<class Propeller, class Rudder>
  void PropellerRudder<Propeller, Rudder>::Initialize() {
    m_propeller->Initialize();
    m_rudder->Initialize();
  }

  template<class Propeller, class Rudder>
  void PropellerRudder<Propeller, Rudder>::Compute(const double &water_density,
                                                   const double &u_NWU_propeller, // u_P0
                                                   const double &v_NWU_propeller, // v_P0
                                                   const double &r, // Vessel
                                                   const double &xr,
                                                   const double &rpm,
                                                   const double &pitch_ratio,
                                                   const double &rudder_angle_deg) const {

    /**
     * Solving for propeller action directly using propeller classes implementation
     */
    m_propeller->Compute(water_density,
                         u_NWU_propeller,
                         v_NWU_propeller,
                         rpm,
                         pitch_ratio);

    PropellerParams propeller_params = m_propeller->GetParameters();
    RudderParams rudder_params = m_rudder->GetParameters();

    /*
     * Computing velocities seen by the rudder outside the slipstream but taking into account the wake fraction
     */
    double wr0 = rudder_params.m_hull_wake_fraction_0; // rudder wake fraction in straight line

    double u_R0 = u_NWU_propeller;
    double v_R0 = v_NWU_propeller - r * xr;  // Transport of the propeller velocity to the rudder position

    double rudder_sidewash_angle_0 = std::atan2(v_R0, u_R0);

    // Estimated wake fraction for the rudder
    double wR = wr0 * std::exp(-4. * rudder_sidewash_angle_0 * rudder_sidewash_angle_0);

    double uRA = u_R0 * (1 - wR);

    // TODO: ici, pour calculer vRA, on peut appliquer la correction kappa
    double vRA = v_R0;
    if (rudder_params.m_use_transverse_velocity_correction) {

    }

    double rudder_angle_rad = rudder_angle_deg * MU_PI_180;

    // Mean axial speed of inflow to the propeller (with wake fraction correction included)
    double uPA = m_propeller->GetAdvanceVelocity();
    double vPA = v_NWU_propeller;

    // Propeller data
    double r0 = propeller_params.m_diameter_m;  // Propeller radius
    double Ap = MU_PI * r0 * r0;  // Propeller disk area

    // Rudder data
    double A_R = rudder_params.m_lateral_area_m2;// Rudder total area
    double c = rudder_params.m_chord_m;// Rudder chord length at its half height
    double hR = rudder_params.m_height_m;// Rudder height

    /**
     * Dealing with rudder forces INSIDE the slipstream of the propeller (RP)
     */

    // Stagnation pressure at propeller position
    double q_PA = 0.5 * water_density * (uPA * uPA + vPA * vPA);

    // Thrust loading coefficient
    double Cth = std::abs(m_propeller->GetThrust() / (q_PA * Ap));

    // Mean axial speed of the slipstream far behind the propeller
    double u_inf = uPA * std::sqrt(1. + Cth);

    // Slipstream radius far behind the propeller (potential)
    double r_inf = r0 * std::sqrt(0.5 * (1. + uPA / u_inf));

    // Slipstream radius at rudder position (potential)
    double rinf_r0 = r_inf / r0;
    double rinf_r0_3 = std::pow(rinf_r0, 3);
    double x_r0_1_5 = std::pow(xr / r0, 1.5);

    double rx = r0 * (0.14 * rinf_r0_3 + rinf_r0 * x_r0_1_5) / (0.14 * rinf_r0_3 + x_r0_1_5);

    // Axial velocity at rudder position (potential)
    double rinf_rx = r_inf / rx;
    double ux = u_inf * rinf_rx * rinf_rx;

    // Turbulent mixing correction on radius
    double drx = 0.15 * xr * (ux - uPA) / (ux + uPA);

    // Corrected radius and axial velocities
    double rRP = rx + drx; // corrected radius
    double r_rdr = rx / (rRP);
    double uRP = (ux - uPA) * r_rdr * r_rdr + uPA; // corrected axial velocity

    // Rudder area seen by the slipstream
    double A_RP = (2. * rRP / hR) * A_R;

    // TODO: ici, on calcule les efforts de portance et de trainee

    double vRP = vRA; // Radial velocity at the rudder position


    /// Debut du code replique
    // Drift angle in the slipstream
    double beta_RP = std::atan2(vRP, uRP);

    // Attack angle in the slipstream
    double alpha_RP_rad = rudder_angle_rad - beta_RP;

    // Get Coefficients
    double cl_RP, cd_RP, cn_RP;
    m_rudder->GetClCdCn(alpha_RP_rad, rudder_angle_rad, cl_RP, cd_RP, cn_RP);

    // Correction for the influence of lateral variation of flow speed
    double d = 0.886 * rRP;
    double f = 2. * std::pow(2. / (2. + d / c), 8);
    double lambda = std::pow(uPA / uRP, f);
    cl_RP *= lambda;

    // Stagnation pressure ar rudder level
    double q_RP = 0.5 * water_density * (uRP * uRP + vRP * vRP);

    // Computing loads at rudder in the slipstream
    double lift_RP = q_RP * cl_RP * A_RP; // FIXME: prise en compte du signe de alpha_RP_rad ??
    double drag_RP = q_RP * cd_RP * A_RP;
    double torque_RP = q_RP * cn_RP * A_RP * c;

    // Projection
    double Cbeta_RP = std::cos(beta_RP);
    double Sbeta_RP = std::sin(beta_RP);

    double fx_RP = Cbeta_RP * drag_RP - Sbeta_RP * lift_RP;
    double fy_RP = Sbeta_RP * drag_RP + Cbeta_RP * lift_RP;

    /// Fin du code replique


    /**
     * Dealing with rudder forces OUTSIDE the slipstream of the propeller (RA) (no influence of the propeller)
     */

    /// Debut du code replique
    // Drift angle in the slipstream
    double beta_RA = std::atan2(vRA, uRA);

    // Attack angle in the slipstream
    double alpha_RA_rad = rudder_angle_rad - beta_RA;

    // Get Coefficients
    double cl_RA, cd_RA, cn_RA;
    m_rudder->GetClCdCn(alpha_RA_rad, rudder_angle_rad, cl_RA, cd_RA, cn_RA);

    // Stagnation pressure ar rudder level
    double q_RA = 0.5 * water_density * (uRP * uRP + vRP * vRP);

    // Rudder area outside of the slipstream
    double A_RA = A_R - A_RP;

    // Computing loads at rudder in the slipstream
    double lift_RA = q_RA * cl_RA * A_RA; // FIXME: prise en compte du signe de alpha_RP_rad ??
    double drag_RA = q_RA * cd_RA * A_RA;
    double torque_RA = q_RA * cn_RA * A_RA * c;

    // Projection
    double Cbeta_RA = std::cos(beta_RA);
    double Sbeta_RA = std::sin(beta_RA);

    double fx_RA = Cbeta_RA * drag_RA - Sbeta_RA * lift_RA;
    double fy_RA = Sbeta_RA * drag_RA + Cbeta_RA * lift_RA;
    /// Fin du code replique


    /**
     * Summing up rudder forces from outside and inside the propeller slipstream
     */

    double fx_R = fx_RA + fx_RP;
    double fy_R = fy_RA + fy_RP;
    double torque_R = (torque_RA + torque_RP) - xr * fy_R;  // Transport of the rudder torque to the propeller location

  }


}  // end namespace acme