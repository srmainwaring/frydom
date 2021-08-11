//
// Created by frongere on 09/08/2021.
//

#ifndef ACME_PROPELLERRUDDER_H
#define ACME_PROPELLERRUDDER_H

#include <memory>

#include "acme/thruster/ThrusterBaseModel.h"
#include "acme/rudder/SimpleRudderModel.h"

namespace acme {

  class PropellerRudderBase {

  };

  template<class Propeller, class Rudder>
  class PropellerRudder : public PropellerRudderBase {

   public:
    PropellerRudder(const ThrusterParams &thruster_params,
                    const std::string &thruster_perf_data_json_string,
                    const RudderParams &rudder_params,
                    const std::string &rudder_perf_data_json_string);

    void Compute(const double &water_density,
                 const double &u_NWU_propeller,
                 const double &v_NWU_propeller,
                 const double &rpm,
                 const double &pitch_ratio,
                 const double &u_NWU_rudder,
                 const double &v_NWU_rudder,
                 const double &rudder_angle_deg) const;


   private:
    std::unique_ptr<Propeller> m_propeller;
    std::unique_ptr<Rudder> m_rudder;

  };


  std::shared_ptr<PropellerRudderBase>
  build_pr() {

    ThrusterModelType prop_type(E_FPP1Q);
    RudderModelType rudder_type(E_SIMPLE_RUDDER);

    ThrusterParams thruster_params;
    std::string thruster_perf_data_json_string;

    RudderParams rudder_params;
    std::string rudder_perf_data_json_string;


    std::shared_ptr<PropellerRudderBase> pr;

    switch (prop_type) {
      case E_FPP1Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<PropellerRudder<FPP1Q, SimpleRudderModel>>(thruster_params,
                                                                             thruster_perf_data_json_string,
                                                                             rudder_params,
                                                                             rudder_perf_data_json_string);

            break;
          case E_FLAP_RUDDER:
            break;
        }
        break;
      case E_FPP4Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            break;
          case E_FLAP_RUDDER:
            break;
        }
        break;
      case E_CPP:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            break;
          case E_FLAP_RUDDER:
            break;
        }
        break;
    }

    return pr;
  }


}  // end namespace acme

#include "PropellerRudder.hpp"

#endif //ACME_PROPELLERRUDDER_H
