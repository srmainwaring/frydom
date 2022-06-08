//
// Created by frongere on 12/10/2021.
//

#include "FrAbkowitzManoeuvringForce.h"
#include "frydom/logging/FrEventLogger.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/io/JSONNode.h"
#include "frydom/utils/FrFileSystem.h"
#include "frydom/hydrodynamics/manoeuvring/FrHullResistance.h"

namespace frydom {

  FrAbkowitzManoeuvringForce::FrAbkowitzManoeuvringForce(const std::string &name,
                                                         FrBody *body,
                                                         const std::string &file,
                                                         const Position& mid_ship) :
      FrForce(name, "FrManoeuvringForce", body),
      c_filepath(file),
      m_Xvv(0.), m_Xvvvv(0.), m_Xrr(0.), m_Xvr(0.),
      m_Yv(0.), m_Yvvv(0.), m_Yvrr(0.), m_Yr(0.), m_Yrrr(0.), m_Yvvr(0.),
      m_Nv(0.), m_Nvvv(0.), m_Nvrr(0.), m_Nr(0.), m_Nrrr(0.), m_Nvvr(0.),
      m_mid_ship(mid_ship) {}

  void FrAbkowitzManoeuvringForce::Initialize() {
    FrForce::Initialize();
    LoadManoeuvringData(c_filepath);
  }

  double FrAbkowitzManoeuvringForce::GetUMin() const {
    double Umin = 0.;
    if (dynamic_cast<FrInterpHullResistance*>(m_hullResistance.get())) {
      Umin = m_hullResistance->GetUMin();
    } else if (dynamic_cast<FrQuadHullResistance*>(m_hullResistance.get())) {
      Umin =  - 0.3 * std::sqrt(GetSystem()->GetGravityAcceleration() * m_Lpp);
    } else {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "wrong hull resistance model");
      Umin = 0;
    }
    return Umin;
  }

  double FrAbkowitzManoeuvringForce::GetUMax() const {
    double Umax;
    if (dynamic_cast<FrInterpHullResistance*>(m_hullResistance.get())) {
      Umax = m_hullResistance->GetUMax();
    } else if (dynamic_cast<FrQuadHullResistance*>(m_hullResistance.get())) {
      Umax = 0.3 * std::sqrt(GetSystem()->GetGravityAcceleration() * m_Lpp);
    } else {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "wrong hull resistance model");
      Umax = 0;
    }
    return Umax;
  }

  void FrAbkowitzManoeuvringForce::Compute(double time) {

    auto body = GetBody();
    auto rho = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

    // Get the velocity at mid-ship
    Velocity vessel_vel_world = body->GetVelocityInWorldAtPointInBody(m_mid_ship, NWU);
    auto vessel_angular_vel_world = body->GetAngularVelocityInWorld(NWU);

    // Projection to the planar frame
    Direction vessel_x_axis_world = body->GetFrame().GetXAxisInParent(NWU);
    vessel_x_axis_world[2] = 0;
    vessel_x_axis_world.Normalize();

    Direction vessel_y_axis_world = body->GetFrame().GetYAxisInParent(NWU);
    vessel_y_axis_world[2] = 0;
    vessel_y_axis_world.Normalize();

    double u = vessel_vel_world.dot(vessel_x_axis_world);
    double v = vessel_vel_world.dot(vessel_y_axis_world);
    double r = vessel_angular_vel_world.GetWz();

    // Adim
    double vnorm2 = u*u + v*v;
    if (vnorm2 == 0.)
      return;

    double vnorm = std::sqrt(vnorm2);
    auto q = 0.5 *rho * m_Lpp * m_draft * vnorm2;
    u = u/vnorm;
    v = v/vnorm;
    r = r*m_Lpp/vnorm;

    // Compute hydrodynamic derivatives
    double c_Xvv = m_Xvv * v*v;
    double c_Xvr = m_Xvr * v*r;
    double c_Xrr = m_Xrr * r*r;
    double c_Xvvvv = m_Xvvvv * pow(v, 4);

    double c_Yv = m_Yv * v;
    double c_Yr = m_Yr * r;
    double c_Yvvv = m_Yvvv * pow(v, 3);
    double c_Yvvr = m_Yvvr * v*v*r;
    double c_Yvrr = m_Yvrr * v*r*r;
    double c_Yrrr = m_Yrrr * r*r*r;

    double c_Nv = m_Nv * v;
    double c_Nr = m_Nr * r;
    double c_Nvvv = m_Nvvv * pow(v, 3);
    double c_Nvvr = m_Nvvr * v*v*r;
    double c_Nvrr = m_Nvrr * v*r*r;
    double c_Nrrr = m_Nrrr * pow(r, 3);

    // Compute force and torque
    auto X = q * (c_Xvv + c_Xvr + c_Xrr + c_Xvvvv) - m_hullResistance->Rh(u * vnorm);
    auto Y = q * (c_Yv + c_Yr + c_Yvvv + c_Yvvr + c_Yvrr + c_Yrrr);
    auto N = q * m_Lpp * (c_Nv + c_Nr + c_Nvvv + c_Nvvr + c_Nvrr + c_Nrrr);

    Force forceInWorld = X * vessel_x_axis_world + Y * vessel_y_axis_world;
    Torque torqueInWorld = {0., 0., N};

    SetForceTorqueInWorldAtPointInBody(forceInWorld, torqueInWorld, m_mid_ship, NWU);

  }

  void FrAbkowitzManoeuvringForce::DefineLogMessages() {
    FrForce::DefineLogMessages();
  }

  void FrAbkowitzManoeuvringForce::LoadResistanceCurve(const std::string& filepath) {

    bool interp_hull_resistance = false;

    // Loader

    try {
      std::ifstream ifs(filepath);
      json j = json::parse(ifs);

      auto node = j["hull_resistance"];

      try {
        auto u = std::make_shared<std::vector<double>>(node["vessel_speed_kt"].get<std::vector<double>>());
        for (auto &vel : *u) vel = convert_velocity_unit(vel, KNOT, MS);
        auto Rh = std::make_shared<std::vector<double>>(node["Rh_N"].get<std::vector<double>>());
        m_hullResistance = std::make_shared<FrInterpHullResistance>(u, Rh);
        interp_hull_resistance = true;
      } catch (json::parse_error& err) {
        // TODO
      } catch (nlohmann::detail::type_error &error) {
        // TODO
      }

      if (!interp_hull_resistance) {
        try {
          auto a_pos = node["a+"].get<double>();
          auto a_neg = node["a-"].get<double>();
          auto b_pos = node["b+"].get<double>();
          auto b_neg = node["b-"].get<double>();
          m_hullResistance = std::make_shared<FrQuadHullResistance>(a_pos, a_neg, b_pos, b_neg);
        } catch (json::parse_error& err) {
          event_logger::error("frAbkowitzManoeuvringForce", GetName(), "wrong data format in hull resistance json file");
          exit(EXIT_FAILURE);
        }
      }

    } catch (json::parse_error& err) {
      std::cout << err.what() << std::endl;
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "hull resistance curve file can't be parsed");
      exit(EXIT_FAILURE);
    }

  }

  void FrAbkowitzManoeuvringForce::LoadManoeuvringData(const std::string &filepath) {

    // Loader
    std::ifstream ifs(filepath);
    json j = json::parse(ifs);

    auto node = j["Abkowitz_manoeuvring_model"];

    //if (node.get_val<std::string>("type") != "abkowitz") {
    //  std::cerr << "Manoeuvring model enclosed in json file "
    //            << filepath
    //            << " was intended to be of type abkowitz but type was "
    //            << node.get_val<std::string>("type")
    //            << std::endl
    //            << "Aborting..."
    //            << std::endl;
    //  exit(EXIT_FAILURE);
    //}

    //auto file_format_version = node.get_val<std::string>("file_format_version");

    try {
      m_Lpp = node["hull_characteristics"]["length_m"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no length_m in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_draft = node["hull_characteristics"]["draft_m"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no draft_m in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Xvv = node["coefficients"]["Xvv"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Xvv in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Xvvvv = node["coefficients"]["Xvvvv"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Xvvvv in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Xvr = node["coefficients"]["Xvr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Xvr in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Xrr = node["coefficients"]["Xrr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Xrr in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Yv = node["coefficients"]["Yv"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Yv in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Yvvv = node["coefficients"]["Yvvv"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Yvvv in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Yvrr = node["coefficients"]["Yvrr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Yvrr in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Yr = node["coefficients"]["Yr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Yr in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Yrrr = node["coefficients"]["Yrrr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Yrrr in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Yvvr = node["coefficients"]["Yvvr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Yvvr in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Nv = node["coefficients"]["Nv"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Nv in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Nvvv= node["coefficients"]["Nvvv"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Nvvv in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Nvrr= node["coefficients"]["Nvrr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Nvrr in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Nr= node["coefficients"]["Nr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Nr in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Nrrr= node["coefficients"]["Nrrr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Nrrr in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_Nvvr= node["coefficients"]["Nvvr"].get<double>();
    } catch (json::parse_error& err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no Nvvr in json file");
      exit(EXIT_FAILURE);
    }

    //try {
    //  auto wave_resistance_file = node["wave_resistance_file"].get<json::string_t>();
    //  // TODO
    //} catch (json::parse_error& err) {
    //  event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no wave_resistance_file in json file");
    //  exit(EXIT_FAILURE);
    //}

    try {
      auto hull_resistance_filepath = node["hull_resistance_filepath"].get<json::string_t>();
      auto test_filepath = FrFileSystem::join({c_filepath, "..", hull_resistance_filepath});
      LoadResistanceCurve(test_filepath);
    } catch (json::parse_error &err) {
      event_logger::error("FrAbkowitzManoeuvringForce", GetName(), "no hull_resistance_filepath in json file");
      exit(EXIT_FAILURE);
    }

  }

  std::shared_ptr<FrAbkowitzManoeuvringForce>
      make_abkowitz_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody> body,
                                      const std::string& file, const Position& mid_ship) {
    auto force = std::make_shared<FrAbkowitzManoeuvringForce>(name, body.get(), file, mid_ship);
    body->AddExternalForce(force);
    return force;
  }

}  // end namespace frydom
