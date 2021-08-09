//
// Created by frongere on 09/08/2021.
//

#ifndef ACME_RUDDERBASEMODEL_H
#define ACME_RUDDERBASEMODEL_H

#include <string>

#include "MathUtils/LookupTable1D.h"


namespace acme {

  struct RudderBaseParams {
    double m_lateral_area_m2;
    double m_chord_m;

    double m_hull_wake_fraction_0;

  };


  class SimpleRudderModel {

   public:
    SimpleRudderModel(const RudderBaseParams params, const std::string &perf_data_json_string);

    virtual void Initialize();

    virtual void Compute(const double &water_density,
                         const double &u_NWU,
                         const double &v_NWU,
                         const double &rudder_angle_deg) const;

   protected:
//    void GetRudderRelativeVelocity(const double &u_NWU, const double &v_NWU,
//                                   double &uRA, double &vRA) const;

    // TODO: ajouter les getters pour les infos permettant de recuperer les composantes d'effort dans un repere local

   private:

    inline double cl(const double &attack_angle_rad) const;
    inline double cd(const double &attack_angle_rad) const;
    inline double cn(const double &attack_angle_rad) const;

    virtual void ParseRudderPerformanceCurveJsonString();


   protected:
    bool m_is_initialized;
    std::string m_temp_perf_data_json_string;

    RudderBaseParams m_params;

   private:
    mathutils::LookupTable1D<double, double> m_cl_cd_cn_coeffs;

    mutable double c_alpha_R;
    mutable double c_lift;
    mutable double c_drag;
    mutable double c_torque;
    mutable double c_fx;
    mutable double c_fy;



  };





}  // end namespace acme

#endif //ACME_RUDDERBASEMODEL_H
