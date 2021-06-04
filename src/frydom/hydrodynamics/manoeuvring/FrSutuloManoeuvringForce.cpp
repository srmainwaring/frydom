//
// Created by lletourn on 04/06/2021.
//

#include "FrSutuloManoeuvringForce.h"
#include "frydom/logging/FrEventLogger.h"
#include "frydom/environment/FrEnvironment.h"

namespace frydom {


  FrSutuloManoeuvringForce::FrSutuloManoeuvringForce(const std::string &name, frydom::FrBody *body,
                                                     const std::string &file)
      : FrForce(name, "FrManoeuvringForce", body), c_filepath(file), m_Cm(0.625), m_A22(0.), m_shipDraft(0.),
        m_shipLength(0.) {

  }

  void FrSutuloManoeuvringForce::Initialize() {
    FrForce::Initialize();
    LoadManoeuvringData(c_filepath);
  }

  void FrSutuloManoeuvringForce::SetShipCharacteristics(double length, double draft) {
    m_shipDraft = draft;
    m_shipLength = length;
  }

  double FrSutuloManoeuvringForce::GetShipLength() const {
    return m_shipLength;
  }

  double FrSutuloManoeuvringForce::GetShipDraft() const {
    return m_shipDraft;
  }

  void FrSutuloManoeuvringForce::SetCorrectionFactor(double Cm) {
    m_Cm = Cm;
  }

  double FrSutuloManoeuvringForce::GetCorrectionFactor() const {
    return m_Cm;
  }

  void FrSutuloManoeuvringForce::SetManoeuvringCoefficients(std::vector<double> cy, std::vector<double> cn) {
    m_cy = std::move(cy);
    m_cn = std::move(cn);
  }

  void FrSutuloManoeuvringForce::Compute(double time) {

    auto rho = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    auto shipVelocityInHeadingFrame = GetBody()->GetVelocityInHeadingFrame(NWU);
    auto u = shipVelocityInHeadingFrame.GetVx();
    auto v = shipVelocityInHeadingFrame.GetVy();
    auto V2 = u * u + v * v;
    auto beta = ComputeShipDriftAngle();
    auto r = ComputeYawAdimVelocity();

    auto cosBeta = cos(beta);
    auto sinBeta = sin(beta);
    auto signR = mathutils::sgn(r);

    auto Xsecond = -2 * Rh(u) / (rho * m_shipLength * m_shipDraft * V2) * cosBeta * std::abs(cosBeta) * (1. - r * r);
    Xsecond -= 2 * m_Cm * m_A22 / (rho * m_shipDraft * m_shipLength * m_shipLength) * sinBeta * r * sqrt(1 - r * r);

    auto Ysecond = m_cy[0] * r;
    Ysecond += m_cy[1] * sinBeta * sin(MU_PI * r) * signR;
    Ysecond += m_cy[2] * sinBeta * cos(MU_PI_2 * r);
    Ysecond += m_cy[3] * sin(2. * beta) * cos(MU_PI_2 * r);
    Ysecond += m_cy[4] * cosBeta * sin(MU_PI * r);
    Ysecond += m_cy[5] * cos(2. * beta) * sin(MU_PI * r);
    Ysecond += m_cy[6] * cosBeta * (cos(MU_PI_2 * r) - cos(3. * MU_PI_2 * r)) * signR;
    Ysecond += m_cy[7] * (cos(2. * beta) - cos(4. * beta)) * cos(MU_PI_2 * r) * mathutils::sgn(beta);
    Ysecond += m_cy[8] * sin(3. * beta) * cos(MU_PI_2 * r);

    auto Nsecond = m_cn[0] * r;
    Nsecond += m_cn[1] * sin(2. * beta) * cos(MU_PI_2 * r);
    Nsecond += m_cn[2] * sinBeta * cos(MU_PI_2 * r);
    Nsecond += m_cn[3] * cos(2. * beta) * sin(MU_PI * r);
    Nsecond += m_cn[4] * cosBeta * sin(MU_PI * r);
    Nsecond += m_cn[5] * (cos(2. * beta) - cos(4. * beta)) * sin(MU_PI * r);
    Nsecond += m_cn[6] * cosBeta * (cosBeta - cos(3. * beta)) * signR;
    Nsecond += m_cn[7] * sin(2. * beta) * (cos(MU_PI_2 * r) - cos(3. * MU_PI_2 * r));
    Nsecond += m_cn[8] * sinBeta * (cos(MU_PI_2 * r) - cos(3. * MU_PI_2 * r));
    Nsecond += m_cn[9] * sin(2. * beta) * (cos(MU_PI_2 * r) - cos(3. * MU_PI_2 * r)) * signR;

    auto mul = 0.5 * rho * (V2 + m_shipLength * m_shipLength * r * r) * m_shipLength * m_shipDraft;

    auto headingFrame = GetBody()->GetHeadingFrame();

    Force bareHullForceInHeadingFrame(Xsecond, Ysecond, 0.);
    auto bareHullForceInWorld = headingFrame.ProjectVectorFrameInParent(bareHullForceInHeadingFrame * mul, NWU);

    auto bareHullTorqueInWorld = headingFrame.ProjectVectorFrameInParent(Torque(0., 0., Nsecond * mul), NWU);

    SetForceTorqueInWorldAtCOG(bareHullForceInWorld, bareHullTorqueInWorld, NWU);

  }

  double FrSutuloManoeuvringForce::Rh(double u) const {
    return m_hullResistance.Eval(u);
  }

  double FrSutuloManoeuvringForce::ComputeShipDriftAngle() {
    auto shipVelocityInHeadingFrame = GetBody()->GetVelocityInHeadingFrame(NWU);
    double u = shipVelocityInHeadingFrame.GetVx();
    double v = shipVelocityInHeadingFrame.GetVy();
    auto beta = std::asin(v / sqrt(u * u + v * v));
    if (u >= 0.)
      beta *= -1;
    else if (u < 0)
      beta -= -MU_PI * mathutils::sgn(v);
    return beta;
  }

  double FrSutuloManoeuvringForce::ComputeYawAdimVelocity() {
    auto shipVelocityInHeadingFrame = GetBody()->GetVelocityInHeadingFrame(NWU);
    auto u = shipVelocityInHeadingFrame.GetVx();
    auto v = shipVelocityInHeadingFrame.GetVy();
    auto r = GetBody()->GetAngularVelocityInWorld(NWU).GetWz();

    return r * m_shipLength / sqrt(u * u + v * v + r * r * m_shipLength * m_shipLength);
  }

  void FrSutuloManoeuvringForce::LoadResistanceCurve(const std::string &filepath) {

    std::shared_ptr<std::vector<double>> u, Rh;

    // Loader.
    std::ifstream ifs(filepath);
    json j = json::parse(ifs);

    auto node = j["hull_resistance"];

    try {
      u = std::make_shared<std::vector<double>>(
          convert_velocity_unit(node["vessel_speed_kt"].get<std::vector<double>>(), KNOT, MS));
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no vessel_speed_kt in json file");
      exit(EXIT_FAILURE);
    }

    try {
      Rh = std::make_shared<std::vector<double>>(node["Rh_N"].get<std::vector<double>>());
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no Rh_N in json file");
      exit(EXIT_FAILURE);
    }

    m_hullResistance.Initialize(u, Rh);

  }

  void FrSutuloManoeuvringForce::LoadManoeuvringData(const std::string &filepath) {

    // Loader.
    std::ifstream ifs(filepath);
    json j = json::parse(ifs);

    auto node = j["Sutulo_manoeuvring_model"];

    try {
      m_shipLength = node["hull_characteristics"]["length_m"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no length_m in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_shipDraft = node["hull_characteristics"]["draft_m"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no draft_m in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Cm = node["Cm"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no Cm in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_A22 = node["A22"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no A22 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cy = node["coefficients"]["cy"].get<std::vector<double>>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn = node["coefficients"]["cn"].get<std::vector<double>>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy in json file");
      exit(EXIT_FAILURE);
    }

    try {
      auto hull_resistance_filepath = node["hull_resistance_filepath"].get<json::string_t>();
      LoadResistanceCurve(hull_resistance_filepath);
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no A22 in json file");
      exit(EXIT_FAILURE);
    }

  }

} // end namespace frydom