//
// Created by lletourn on 04/06/2021.
//

#include "FrSutuloManoeuvringForce.h"
#include "frydom/logging/FrEventLogger.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/hydrodynamics/manoeuvring/FrHullResistance.h"

namespace frydom {

  FrSutuloManoeuvringForce::FrSutuloManoeuvringForce(const std::string& name, FrBody *body,
                                                     const FrSutuloManCoefficients& coeffs,
                                                     const std::shared_ptr<FrHullResistanceForce>& hullResistanceForce,
                                                     double draft_m, double lpp_m)
      : FrForce(name, "FrSutuloManoeuvringForce", body), m_draft(draft_m), m_lpp(lpp_m),
      m_hullResistanceForce(hullResistanceForce)
      {
    // Set coefficients
    m_Cm = coeffs.m_cm;
    m_mu22 = coeffs.m_mu22;
    m_cn0 = coeffs.m_cn0;
    m_cn1 = coeffs.m_cn1;
    m_cn2 = coeffs.m_cn2;
    m_cn3 = coeffs.m_cn3;
    m_cn4 = coeffs.m_cn4;
    m_cn5 = coeffs.m_cn5;
    m_cn6 = coeffs.m_cn6;
    m_cn7 = coeffs.m_cn7;
    m_cn8 = coeffs.m_cn8;
    m_cn9 = coeffs.m_cn9;
    m_cy0 = coeffs.m_cy0;
    m_cy1 = coeffs.m_cy1;
    m_cy2 = coeffs.m_cy2;
    m_cy3 = coeffs.m_cy3;
    m_cy4 = coeffs.m_cy4;
    m_cy5 = coeffs.m_cy5;
    m_cy6 = coeffs.m_cy6;
    m_cy7 = coeffs.m_cy7;
    m_cy8 = coeffs.m_cy8;
  }

  void FrSutuloManoeuvringForce::Initialize() {
    FrForce::Initialize();
  }

  void FrSutuloManoeuvringForce::Compute(double time) {

    auto rho = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    auto body = GetBody();
    auto shipVelocityInHeadingFrame = body->GetVelocityInHeadingFrame(NWU);
    double u = shipVelocityInHeadingFrame.GetVx();
    double v = shipVelocityInHeadingFrame.GetVy();
    double r = body->GetAngularVelocityInWorld(NWU).GetWz();
    double V2 = u * u + v * v;

    c_beta = ComputeShipDriftAngle(u, v);
    c_rpp = ComputeYawAdimVelocity(u, v, r, m_lpp);

    double cbeta = cos(c_beta);
    double sbeta = sin(c_beta);
    int sign_rpp = mathutils::sgn(c_rpp);

    c_Xpp0 = 0.;
    if (u > DBL_EPSILON)
      c_Xpp0 = -2 * Rh(u) / (rho * m_lpp * m_draft * V2) * cbeta * std::abs(cbeta) * (1. - c_rpp * c_rpp);
    c_Xpp1 = -2 * m_Cm * m_mu22 / (rho * m_draft * m_lpp * m_lpp) * sbeta * c_rpp * sqrt(1 - c_rpp * c_rpp);

    c_Xpp = c_Xpp0 + c_Xpp1;

    c_Ypp0 = m_cy0 * c_rpp;
    c_Ypp1 = m_cy1 * sbeta * sin(MU_PI * c_rpp) * sign_rpp;
    c_Ypp2 = m_cy2 * sbeta * cos(MU_PI_2 * c_rpp);
    c_Ypp3 = m_cy3 * sin(2. * c_beta) * cos(MU_PI_2 * c_rpp);
    c_Ypp4 = m_cy4 * cbeta * sin(MU_PI * c_rpp);
    c_Ypp5 = m_cy5 * cos(2. * c_beta) * sin(MU_PI * c_rpp);
    c_Ypp6 = m_cy6 * cbeta * (cos(MU_PI_2 * c_rpp) - cos(3. * MU_PI_2 * c_rpp)) * sign_rpp;
    c_Ypp7 = m_cy7 * (cos(2. * c_beta) - cos(4. * c_beta)) * cos(MU_PI_2 * c_rpp) * mathutils::sgn(c_beta);
    c_Ypp8 = m_cy8 * sin(3. * c_beta) * cos(MU_PI_2 * c_rpp);

    c_Ypp = c_Ypp0 + c_Ypp1 + c_Ypp2 + c_Ypp3 + c_Ypp4 + c_Ypp5 + c_Ypp6 + c_Ypp7 + c_Ypp8;

    c_Npp0 = m_cn0 * c_rpp;
    c_Npp1 = m_cn1 * sin(2. * c_beta) * cos(MU_PI_2 * c_rpp);
    c_Npp2 = m_cn2 * sbeta * cos(MU_PI_2 * c_rpp);
    c_Npp3 = m_cn3 * cos(2. * c_beta) * sin(MU_PI * c_rpp);
    c_Npp4 = m_cn4 * cbeta * sin(MU_PI * c_rpp);
    c_Npp5 = m_cn5 * (cos(2. * c_beta) - cos(4. * c_beta)) * sin(MU_PI * c_rpp);
    c_Npp6 = m_cn6 * cbeta * (cbeta - cos(3. * c_beta)) * sign_rpp;
    c_Npp7 = m_cn7 * sin(2. * c_beta) * (cos(MU_PI_2 * c_rpp) - cos(3. * MU_PI_2 * c_rpp));
    c_Npp8 = m_cn8 * sbeta * (cos(MU_PI_2 * c_rpp) - cos(3. * MU_PI_2 * c_rpp));
    c_Npp9 = m_cn9 * sin(2. * c_beta) * (cos(MU_PI_2 * c_rpp) - cos(3. * MU_PI_2 * c_rpp)) * sign_rpp;

    c_Npp = c_Npp0 + c_Npp1 + c_Npp2 + c_Npp3 + c_Npp4 + c_Npp5 + c_Npp6 + c_Npp7 + c_Npp8 + c_Npp9;

    auto mul = 0.5 * rho * (V2 + m_lpp * m_lpp * r * r) * m_lpp * m_draft;

    auto headingFrame = GetBody()->GetHeadingFrame();

    Force force_in_heading_frame(c_Xpp * mul, c_Ypp * mul, 0.);
    auto bareHullForceInWorld = headingFrame.ProjectVectorFrameInParent(force_in_heading_frame, NWU);

    Torque moment_in_heading_frame(0., 0., c_Npp * mul * m_lpp);
    auto moment_in_world = headingFrame.ProjectVectorFrameInParent(moment_in_heading_frame, NWU);

    SetForceTorqueInWorldAtCOG(bareHullForceInWorld, moment_in_world, NWU);

  }

  double FrSutuloManoeuvringForce::Rh(double u) const {
    return m_hullResistanceForce->Rh(u);
  }

  double FrSutuloManoeuvringForce::ComputeShipDriftAngle(const double &u, const double &v) {

    double vel = std::sqrt(u * u + v * v);

    double vp;
    if (std::abs(vel) < DBL_EPSILON) {
      vp = 0.;
    } else {
      vp = v / vel;
    }

    double beta;
    if (u >= 0.) {
      beta = -std::asin(vp);
    } else {
      beta = -MU_PI * mathutils::sgn(v) + std::asin(vp);
    }
    return beta;
  }

  double FrSutuloManoeuvringForce::ComputeYawAdimVelocity(const double &u, const double &v, const double &r,
                                                          const double &L) {
    double rpp = 0.;
    if (std::fabs(r) > 0.) {
      double v2 = u * u + v * v;
      rpp = r * L / std::sqrt(v2 + r * r * L * L);
    }

    return rpp;
  }

  double FrSutuloManoeuvringForce::GetUMin() const {
    return m_hullResistanceForce->GetUMin();
  }

  double FrSutuloManoeuvringForce::GetUMax() const {
    return m_hullResistanceForce->GetUMax();
  }

  void FrSutuloManoeuvringForce::DefineLogMessages() {

    auto msg = NewMessage("FrSutuloManoeuvringForce", "Sutulo manoeuvring force message");

    msg->AddField<double>("Time", "s", "Current time of the simulation",
                          [this]() { return m_chronoForce->GetChTime(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInBody", "N", fmt::format("force in body reference frame in {}", GetLogFC()),
         [this]() { return GetForceInBody(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInBodyAtCOG", "Nm", fmt::format("torque at COG in body reference frame in {}", GetLogFC()),
         [this]() { return GetTorqueInBodyAtCOG(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInWorld", "N", fmt::format("force in world reference frame in {}", GetLogFC()),
         [this]() { return GetForceInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInWorldAtCOG", "Nm", fmt::format("torque at COG in world reference frame in {}", GetLogFC()),
         [this]() { return GetTorqueInWorldAtCOG(GetLogFC()); });

    msg->AddField<double>("beta", "deg", "Ship drift angle",
                          [this]() { return RAD2DEG * this->c_beta; });

    msg->AddField<double>("YawAdimVelocity", "", "Yaw non dimensional velocity", &c_rpp);

    msg->AddField<double>("Xpp", "", "", &c_Xpp);
    msg->AddField<double>("Xpp0", "", "", &c_Xpp0);
    msg->AddField<double>("Xpp1", "", "", &c_Xpp1);

    msg->AddField<double>("Ypp", "", "", &c_Ypp);
    msg->AddField<double>("Ypp0", "", "", &c_Ypp0);
    msg->AddField<double>("Ypp1", "", "", &c_Ypp1);
    msg->AddField<double>("Ypp2", "", "", &c_Ypp2);
    msg->AddField<double>("Ypp3", "", "", &c_Ypp3);
    msg->AddField<double>("Ypp4", "", "", &c_Ypp4);
    msg->AddField<double>("Ypp5", "", "", &c_Ypp5);
    msg->AddField<double>("Ypp6", "", "", &c_Ypp6);
    msg->AddField<double>("Ypp7", "", "", &c_Ypp7);
    msg->AddField<double>("Ypp8", "", "", &c_Ypp8);

    msg->AddField<double>("Npp", "", "", &c_Npp);
    msg->AddField<double>("Npp0", "", "", &c_Npp0);
    msg->AddField<double>("Npp1", "", "", &c_Npp1);
    msg->AddField<double>("Npp2", "", "", &c_Npp2);
    msg->AddField<double>("Npp3", "", "", &c_Npp3);
    msg->AddField<double>("Npp4", "", "", &c_Npp4);
    msg->AddField<double>("Npp5", "", "", &c_Npp5);
    msg->AddField<double>("Npp6", "", "", &c_Npp6);
    msg->AddField<double>("Npp7", "", "", &c_Npp7);
    msg->AddField<double>("Npp8", "", "", &c_Npp8);
    msg->AddField<double>("Npp9", "", "", &c_Npp9);

  }

  // -----------------------------------------------------------------------
  // MAKERS
  // -----------------------------------------------------------------------

  std::shared_ptr<FrSutuloManoeuvringForce>
      make_sutulo_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody>& body, const FrSutuloManCoefficients& coeffs,
                                    const std::shared_ptr<FrHullResistanceForce>& hullResistanceForce,
                                    double draft_m, double lpp_m) {

    auto force = std::make_shared<FrSutuloManoeuvringForce>(name, body.get(), coeffs, hullResistanceForce, draft_m, lpp_m);
    body->AddExternalForce(force);
    return force;
  }

} // end namespace frydom
