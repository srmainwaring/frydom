//
// Created by frongere on 09/08/2021.
//

#ifndef ACME_RUDDERBASEMODEL_H
#define ACME_RUDDERBASEMODEL_H

#include <string>

#include "MathUtils/LookupTable1D.h"

#include "RudderModelType.h"

namespace acme {

  struct RudderParams {
    double m_lateral_area_m2; // Rudder lateral projected area (m**2)
    double m_chord_m; // Rudder chord length at its half height
    double m_height_m;

    double m_hull_wake_fraction_0 = 0.;

    // Optional
//    bool m_has_hull_influence_transverse_velocity = false;
    double m_flap_slope = 0.; // only used for a flap rudder type

    bool m_use_transverse_velocity_correction = false;
  };


  class SimpleRudderModel {

   public:
    SimpleRudderModel(const RudderParams params, const std::string &perf_data_json_string);

    virtual void Initialize();

    virtual void Compute(const double &water_density,
                         const double &u_NWU,
                         const double &v_NWU,
                         const double &rudder_angle_deg) const;

    RudderModelType GetRudderModelType() const;

    const RudderParams &GetParameters() const;

    double GetFx() const { return c_fx_N; }

    double GetFy() const { return c_fy_N; }

    double GetMz() const { return c_torque_Nm; }

    virtual void GetClCdCn(const double &attack_angle_rad,
                           const double &rudder_angle_rad,
                           double &cl,
                           double &cd,
                           double &cn) const;

   private:

    virtual void ParseRudderPerformanceCurveJsonString();


   protected:
    bool m_is_initialized;
    std::string m_temp_perf_data_json_string;

    RudderParams m_params;

    RudderModelType m_type;

   private:
    mathutils::LookupTable1D<double, double> m_cl_cd_cn_coeffs;

    mutable double c_alpha_R_rad;
    mutable double c_lift_N;
    mutable double c_drag_N;
    mutable double c_torque_Nm;
    mutable double c_fx_N;
    mutable double c_fy_N;

  };

  void ParseRudderJsonString(const std::string &json_string,
                             std::vector<double> &attack_angle_rad,
                             std::vector<double> &cd,
                             std::vector<double> &cl,
                             std::vector<double> &cn);


}  // end namespace acme

#endif //ACME_RUDDERBASEMODEL_H
