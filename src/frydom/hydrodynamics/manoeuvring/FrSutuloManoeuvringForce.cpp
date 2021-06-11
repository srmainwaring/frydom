//
// Created by lletourn on 04/06/2021.
//

#include "FrSutuloManoeuvringForce.h"
#include "frydom/logging/FrEventLogger.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/utils/FrFileSystem.h"

namespace frydom {


  FrSutuloManoeuvringForce::FrSutuloManoeuvringForce(const std::string &name, frydom::FrBody *body,
                                                     const std::string &file)
      : FrForce(name, "FrManoeuvringForce", body),
        c_filepath(file), m_Cm(0.625), m_A22(0.), m_shipDraft(0.), m_shipLength(0.),
        m_cy0(0.), m_cy1(0.), m_cy2(0.), m_cy3(0.), m_cy4(0.), m_cy5(0.), m_cy6(0.), m_cy7(0.), m_cy8(0.),
        m_cn0(0.), m_cn1(0.), m_cn2(0.), m_cn3(0.), m_cn4(0.), m_cn5(0.), m_cn6(0.), m_cn7(0.), m_cn8(0.), m_cn9(0.) {

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

  void FrSutuloManoeuvringForce::Compute(double time) {

    auto rho = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    auto shipVelocityInHeadingFrame = GetBody()->GetVelocityInHeadingFrame(NWU);
    auto u = shipVelocityInHeadingFrame.GetVx();
    auto v = shipVelocityInHeadingFrame.GetVy();
    auto V2 = u * u + v * v;
    auto beta = ComputeShipDriftAngle();
    auto r = ComputeYawAdimVelocity();

    auto cbeta = cos(beta);
    auto sbeta = sin(beta);
    auto signR = mathutils::sgn(r);

    double Xsecond = 0.;
    if (u>DBL_EPSILON)
      Xsecond = -2 * Rh(u) / (rho * m_shipLength * m_shipDraft * V2) * cbeta * std::abs(cbeta) * (1. - r * r);
    Xsecond -= 2 * m_Cm * m_A22 / (rho * m_shipDraft * m_shipLength * m_shipLength) * sbeta * r * sqrt(1 - r * r);

    auto Ysecond = m_cy0 * r;
    Ysecond += m_cy1 * sbeta * sin(MU_PI * r) * signR;
    Ysecond += m_cy2 * sbeta * cos(MU_PI_2 * r);
    Ysecond += m_cy3 * sin(2. * beta) * cos(MU_PI_2 * r);
    Ysecond += m_cy4 * cbeta * sin(MU_PI * r);
    Ysecond += m_cy5 * cos(2. * beta) * sin(MU_PI * r);
    Ysecond += m_cy6 * cbeta * (cos(MU_PI_2 * r) - cos(3. * MU_PI_2 * r)) * signR;
    Ysecond += m_cy7 * (cos(2. * beta) - cos(4. * beta)) * cos(MU_PI_2 * r) * mathutils::sgn(beta);
    Ysecond += m_cy8 * sin(3. * beta) * cos(MU_PI_2 * r);

    auto Nsecond = m_cn0 * r;
    Nsecond += m_cn1 * sin(2. * beta) * cos(MU_PI_2 * r);
    Nsecond += m_cn2 * sbeta * cos(MU_PI_2 * r);
    Nsecond += m_cn3 * cos(2. * beta) * sin(MU_PI * r);
    Nsecond += m_cn4 * cbeta * sin(MU_PI * r);
    Nsecond += m_cn5 * (cos(2. * beta) - cos(4. * beta)) * sin(MU_PI * r);
    Nsecond += m_cn6 * cbeta * (cbeta - cos(3. * beta)) * signR;
    Nsecond += m_cn7 * sin(2. * beta) * (cos(MU_PI_2 * r) - cos(3. * MU_PI_2 * r));
    Nsecond += m_cn8 * sbeta * (cos(MU_PI_2 * r) - cos(3. * MU_PI_2 * r));
    Nsecond += m_cn9 * sin(2. * beta) * (cos(MU_PI_2 * r) - cos(3. * MU_PI_2 * r)) * signR;
    auto mul = 0.5 * rho * (V2 + m_shipLength * m_shipLength * r * r) * m_shipLength * m_shipDraft;

    auto headingFrame = GetBody()->GetHeadingFrame();

    Force bareHullForceInHeadingFrame(Xsecond * mul, Ysecond * mul, 0.);
    auto bareHullForceInWorld = headingFrame.ProjectVectorFrameInParent(bareHullForceInHeadingFrame, NWU);

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
    auto beta = 0.;
    if (std::abs(v)>DBL_EPSILON)
       beta = std::asin(v / sqrt(u * u + v * v));
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
    double r_second = 0.;
    if (std::abs(r)>DBL_EPSILON)
      r_second = r * m_shipLength / sqrt(u * u + v * v + r * r * m_shipLength * m_shipLength);
    return r_second;
  }

  void FrSutuloManoeuvringForce::LoadResistanceCurve(const std::string &filepath) {

    std::shared_ptr<std::vector<double>> u, Rh;

    // Loader.
    try {
      std::ifstream ifs(filepath);
      json j = json::parse(ifs);

      auto node = j["hull_resistance"];

      try {
        u = std::make_shared<std::vector<double>>(node["vessel_speed_kt"].get<std::vector<double>>());
        for (auto &vel: *u) vel = convert_velocity_unit(vel, KNOT, MS);

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

    } catch (json::parse_error &err) {
      std::cout<<err.what()<<std::endl;
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "resistance curve file can't be parsed");
      exit(EXIT_FAILURE);
    }

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
      m_cy0 = node["coefficients"]["cy0"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cy1 = node["coefficients"]["cy1"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy1 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cy2 = node["coefficients"]["cy2"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy2 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cy3 = node["coefficients"]["cy3"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy3 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cy4 = node["coefficients"]["cy4"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy4 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cy5 = node["coefficients"]["cy5"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy5 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cy6 = node["coefficients"]["cy6"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy6 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cy7 = node["coefficients"]["cy7"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy7 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cy8 = node["coefficients"]["cy8"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cy8 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn0 = node["coefficients"]["cn0"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn0 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn1 = node["coefficients"]["cn1"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn1 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn2 = node["coefficients"]["cn2"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn2 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn3 = node["coefficients"]["cn3"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn3 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn4 = node["coefficients"]["cn4"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn4 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn5 = node["coefficients"]["cn5"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn5 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn6 = node["coefficients"]["cn6"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn6 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn7 = node["coefficients"]["cn7"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn7 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn8 = node["coefficients"]["cn8"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn8 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_cn9 = node["coefficients"]["cn9"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no cn9 in json file");
      exit(EXIT_FAILURE);
    }

    try {
      auto hull_resistance_filepath = node["hull_resistance_filepath"].get<json::string_t>();
      auto test_filepath = FrFileSystem::join({filepath, "..", hull_resistance_filepath});
      LoadResistanceCurve(test_filepath);
    } catch (json::parse_error &err) {
      event_logger::error("FrSutuloManoeuvringForce", GetName(), "no hull_resistance_filepath in json file");
      exit(EXIT_FAILURE);
    }

  }

  void FrSutuloManoeuvringForce::DefineLogMessages() {

    auto msg = NewMessage("FrSutuloManoeuvringForce", "Sutulo manoeuvring force message");

    msg->AddField<double>("Time", "s", "Current time of the simulation",
                          [this]() { return m_chronoForce->GetChTime(); });

    msg->AddField<double>("ShipDriftAngle", "rad", "Ship drift angle",
                          [this]() { return this->ComputeShipDriftAngle(); });

    msg->AddField<double>("YawAdimVelocity", "", "Yaw non dimensional velocity",
                          [this]() { return this->ComputeYawAdimVelocity(); });

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

  }

  std::shared_ptr<FrSutuloManoeuvringForce>
  make_Sutulo_manoeuvring_model(const std::string &name, FrBody *body, const std::string &file) {
    auto force = std::make_shared<FrSutuloManoeuvringForce>(name, body, file);
    body->AddExternalForce(force);
    return force;
  }
} // end namespace frydom