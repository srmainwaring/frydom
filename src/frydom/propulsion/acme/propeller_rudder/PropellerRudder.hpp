//
// Created by frongere on 09/08/2021.
//


#include "PropellerRudder.h"

namespace acme {

  template<class Propeller, class Rudder>
  PropellerRudder<Propeller, Rudder>::PropellerRudder(const ThrusterParams &thruster_params,
                                                      const std::string &thruster_perf_data_json_string,
                                                      const RudderParams &rudder_params,
                                                      const std::string &rudder_perf_data_json_string) :
      m_propeller(std::make_unique<Propeller>(thruster_params, thruster_perf_data_json_string)),
      m_rudder(std::make_unique<Rudder>(rudder_params, rudder_perf_data_json_string)) {}

  template<class Propeller, class Rudder>
  void PropellerRudder<Propeller, Rudder>::Compute(const double &water_density,
                                                   const double &u_NWU_propeller,
                                                   const double &v_NWU_propeller,
                                                   const double &rpm,
                                                   const double &pitch_ratio,
                                                   const double &u_NWU_rudder,
                                                   const double &v_NWU_rudder,
                                                   const double &rudder_angle_deg) const {

    // FIXME: aucune interaction introduite ici !!

    m_propeller->Compute(water_density,
                         u_NWU_propeller,
                         v_NWU_propeller,
                         rpm,
                         pitch_ratio);

    m_rudder->Compute(water_density,
                      u_NWU_rudder,
                      v_NWU_rudder,
                      rudder_angle_deg);

  }


}  // end namespace acme