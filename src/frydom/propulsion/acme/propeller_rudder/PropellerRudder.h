//
// Created by frongere on 09/08/2021.
//

#ifndef ACME_PROPELLERRUDDER_H
#define ACME_PROPELLERRUDDER_H

#include <memory>

#include "acme/propeller/PropellerBaseModel.h"
#include "acme/rudder/SimpleRudderModel.h"

namespace acme {

  class PropellerRudderBase {

  };

  template<class Propeller, class Rudder>
  class PropellerRudder : public PropellerRudderBase {

   public:
    PropellerRudder(const PropellerParams &thruster_params,
                    const std::string &thruster_perf_data_json_string,
                    const RudderParams &rudder_params,
                    const std::string &rudder_perf_data_json_string);

    /// Perform the model calculations
    /// \param water_density in kg/m**3
    /// \param u_NWU_propeller axial velocity with respect to water at the propeller location as obtained from rigid
    ///        body motion of the vessel eventually (including current and waves orbital velocities) in m/s
    /// \param v_NWU_propeller radial velocity with respect to water at the propeller location as obtained from rigid
    ///        body motion of the vessel eventually (including current and waves orbital velocities) in m/s
    /// \param r vessel rotation velocity around its local z-axis (rad/s)
    /// \param xr distance between the propeller and the rudder in m. Accounted positive when the rudder is behind the
    ///           propeller
    /// \param rpm Propeller screw rate of rotation (round per minute)
    /// \param pitch_ratio Propeller pitch ratio. Only used if the chosen propeller model is a CPP.
    /// \param rudder_angle_deg rudder angle (in deg) between the vessel x axis and the rudder chord.
    ///
    /// \note In this implementation, it is considered that the rudder stock is collinear to the vessel z axis and
    ///       directed upwards. As such, u_NWU_propeller and v_NWU_propeller are lying in the Oxy plane of the vessel.
    ///       Propeller rotation axis is also collinear to the vessel x axis.
    void Compute(const double &water_density,
                 const double &u_NWU_propeller,
                 const double &v_NWU_propeller,
                 const double &r,
                 const double &xr,
                 const double &rpm,
                 const double &pitch_ratio,
                 const double &rudder_angle_deg) const;


   private:
    std::unique_ptr<Propeller> m_propeller;
    std::unique_ptr<Rudder> m_rudder;

  };


  std::shared_ptr<PropellerRudderBase> build_pr(PropellerModelType prop_type,
                                                PropellerParams prop_params,
                                                const std::string &prop_perf_data_string,
                                                RudderModelType rudder_type,
                                                RudderParams rudder_params,
                                                const std::string &rudder_perf_data_string) {

    std::shared_ptr<PropellerRudderBase> pr;

    switch (prop_type) {
      case E_FPP1Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<PropellerRudder<FPP1Q, SimpleRudderModel>>(prop_params,
                                                                             prop_perf_data_string,
                                                                             rudder_params,
                                                                             rudder_perf_data_string);

            break;
          case E_FLAP_RUDDER:
            pr = std::make_shared<PropellerRudder<FPP1Q, FlapRudderModel>>(prop_params,
                                                                           prop_perf_data_string,
                                                                           rudder_params,
                                                                           rudder_perf_data_string);
            break;
        }
        break;
      case E_FPP4Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<PropellerRudder<FPP4Q, SimpleRudderModel>>(prop_params,
                                                                             prop_perf_data_string,
                                                                             rudder_params,
                                                                             rudder_perf_data_string);
            break;
          case E_FLAP_RUDDER:
            pr = std::make_shared<PropellerRudder<FPP4Q, FlapRudderModel>>(prop_params,
                                                                           prop_perf_data_string,
                                                                           rudder_params,
                                                                           rudder_perf_data_string);
            break;
        }
        break;
      case E_CPP:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<PropellerRudder<CPP, SimpleRudderModel>>(prop_params,
                                                                           prop_perf_data_string,
                                                                           rudder_params,
                                                                           rudder_perf_data_string);
            break;
          case E_FLAP_RUDDER:
            pr = std::make_shared<PropellerRudder<CPP, FlapRudderModel>>(prop_params,
                                                                         prop_perf_data_string,
                                                                         rudder_params,
                                                                         rudder_perf_data_string);
            break;
        }
        break;
    }

    return pr;
  }


}  // end namespace acme

#include "PropellerRudder.hpp"

#endif //ACME_PROPELLERRUDDER_H
