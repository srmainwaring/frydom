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
        c_filepath(file), m_Cm(0.625), m_mu22(0.), m_shipDraft(0.), m_shipLength(0.),
        m_cy0(0.), m_cy1(0.), m_cy2(0.), m_cy3(0.), m_cy4(0.), m_cy5(0.), m_cy6(0.), m_cy7(0.), m_cy8(0.),
        m_cn0(0.), m_cn1(0.), m_cn2(0.), m_cn3(0.), m_cn4(0.), m_cn5(0.), m_cn6(0.), m_cn7(0.), m_cn8(0.), m_cn9(0.),
        c_beta(0.), c_rpp(0.),
        c_Xpp(0.), c_Xpp0(0.), c_Xpp1(0.),
        c_Ypp(0.),
        c_Ypp0(0.), c_Ypp1(0.), c_Ypp2(0.), c_Ypp3(0.), c_Ypp4(0.), c_Ypp5(0.), c_Ypp6(0.), c_Ypp7(0.), c_Ypp8(0.),
        c_Npp(0.),
        c_Npp0(0.), c_Npp1(0.), c_Npp2(0.), c_Npp3(0.), c_Npp4(0.), c_Npp5(0.), c_Npp6(0.), c_Npp7(0.), c_Npp8(0.),
        c_Npp9(0.) {}

  void FrSutuloManoeuvringForce::Initialize() {
    FrForce::Initialize();
    LoadManoeuvringData(c_filepath);
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
    c_rpp = ComputeYawAdimVelocity(u, v, r, m_shipLength);

    double cbeta = cos(c_beta);
    double sbeta = sin(c_beta);
    int sign_rpp = mathutils::sgn(c_rpp);

    c_Xpp0 = 0.;
    if (u > DBL_EPSILON)
      c_Xpp0 = -2 * Rh(u) / (rho * m_shipLength * m_shipDraft * V2) * cbeta * std::abs(cbeta) * (1. - c_rpp * c_rpp);
    c_Xpp1 = -2 * m_Cm * m_mu22 / (rho * m_shipDraft * m_shipLength * m_shipLength) * sbeta * c_rpp * sqrt(1 - c_rpp * c_rpp);

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

    auto mul = 0.5 * rho * (V2 + m_shipLength * m_shipLength * r * r) * m_shipLength * m_shipDraft;

    auto headingFrame = GetBody()->GetHeadingFrame();

    Force force_in_heading_frame(c_Xpp * mul, c_Ypp * mul, 0.);
    auto bareHullForceInWorld = headingFrame.ProjectVectorFrameInParent(force_in_heading_frame, NWU);

    Torque moment_in_heading_frame(0., 0., c_Npp * mul * m_shipLength);
    auto moment_in_world = headingFrame.ProjectVectorFrameInParent(moment_in_heading_frame, NWU);

    SetForceTorqueInWorldAtCOG(bareHullForceInWorld, moment_in_world, NWU);

  }

  double FrSutuloManoeuvringForce::Rh(double u) const {
    return m_hullResistance.Eval(u);
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
      std::cout << err.what() << std::endl;
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
      m_mu22 = node["A22"].get<double>();
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

  std::shared_ptr<FrSutuloManoeuvringForce>
  make_Sutulo_manoeuvring_model(const std::string &name, FrBody *body, const std::string &file) {
    auto force = std::make_shared<FrSutuloManoeuvringForce>(name, body, file);
    body->AddExternalForce(force);
    return force;
  }
} // end namespace frydom