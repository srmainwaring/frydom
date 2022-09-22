//
// Created by frongere on 12/10/2021.
//

#include "FrAbkowitzManoeuvringForce.h"
#include "frydom/logging/FrEventLogger.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/utils/FrFileSystem.h"
#include "frydom/hydrodynamics/manoeuvring/FrHullResistance.h"

namespace frydom {

  FrAbkowitzManoeuvringForce::FrAbkowitzManoeuvringForce(const std::string &name,
                                                         FrBody *body,
                                                         const std::string &file,
                                                         const Position& mid_ship) :
      FrForce(name, "FrManoeuvringForce", body), m_mid_ship(mid_ship) {
    LoadAbkowitzManoeuvringFile(file);
  }

  FrAbkowitzManoeuvringForce::FrAbkowitzManoeuvringForce(const std::string &name,
                                                         FrBody* body,
                                                         const JSONNode& node_man,
                                                         const JSONNode& node_res,
                                                         const Position& mid_ship,
                                                         double draft_m, double lpp_m) :
       FrForce(name, "FrManoeuvringForce", body), m_mid_ship(mid_ship),
       m_draft(draft_m), m_Lpp(lpp_m)
       {
    // Load hydrodynamic derivatives
    m_Xvv = node_man.get<double>("Xvv");
    m_Xvvvv = node_man.get<double>("Xvvvv");
    m_Xrr = node_man.get<double>("Xrr");
    m_Xvr = node_man.get<double>("Xvr");
    m_Yv = node_man.get<double>("Yv");
    m_Yr = node_man.get<double>("Yr");
    m_Yvvv = node_man.get<double>("Yvvv");
    m_Yvvr = node_man.get<double>("Yvvr");
    m_Yvrr = node_man.get<double>("Yvrr");
    m_Yrrr = node_man.get<double>("Yrrr");
    m_Nv = node_man.get<double>("Nv");
    m_Nr = node_man.get<double>("Nr");
    m_Nvvv = node_man.get<double>("Nvvv");
    m_Nvvr = node_man.get<double>("Nvvr");
    m_Nvrr = node_man.get<double>("Nvrr");
    m_Nrrr = node_man.get<double>("Nrrr");
    // Calm water resistance
    m_hullResistance = make_hull_resistance(node_res);
  }

  void FrAbkowitzManoeuvringForce::Initialize() {
    FrForce::Initialize();
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
    vessel_x_axis_world.normalize();

    Direction vessel_y_axis_world = body->GetFrame().GetYAxisInParent(NWU);
    vessel_y_axis_world[2] = 0;
    vessel_y_axis_world.normalize();

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

  void FrAbkowitzManoeuvringForce::LoadAbkowitzManoeuvringFile(const std::string &filepath) {

    // Loader
    auto j = JSONNode(filepath);
    auto node = j["Abkowitz_manoeuvring_model"];

    // Read main particulars
    auto node_hull = node["hull_characteristics"];
    m_Lpp = node_hull.get<double>("length_m");
    m_draft = node_hull.get<double>("draft_m");

    // Coefficients
    auto node_coeff = node["coefficients"];
    m_Xvv = node_coeff.get<double>("Xvv");
    m_Xvvvv = node_coeff.get<double>("Xvvvv");
    m_Xvr = node_coeff.get<double>("Xvr");
    m_Xrr = node_coeff.get<double>("Xrr");
    m_Yv = node_coeff.get<double>("Yv");
    m_Yr = node_coeff.get<double>("Yr");
    m_Yvvv = node_coeff.get<double>("Yvvv");
    m_Yvvr = node_coeff.get<double>("Yvvr");
    m_Yvrr = node_coeff.get<double>("Yvrr");
    m_Yrrr = node_coeff.get<double>("Yrrr");
    m_Nv = node_coeff.get<double>("Nv");
    m_Nr = node_coeff.get<double>("Nr");
    m_Nvvv = node_coeff.get<double>("Nvvv");
    m_Nvvr = node_coeff.get<double>("Nvvr");
    m_Nvrr = node_coeff.get<double>("Nvrr");
    m_Nrrr = node_coeff.get<double>("Nrrr");

    // Hull resistance loader
    auto hull_resistance_filepath = node.get<std::string>("hull_resistance_filepath");
    auto filename = FrFileSystem::join({filepath, "..", hull_resistance_filepath});
    LoadResistanceCurve(filename);
  }

  // ---------------------------------------------------------------------
  // MAKER FORCE
  // ---------------------------------------------------------------------

  std::shared_ptr<FrAbkowitzManoeuvringForce>
      make_abkowitz_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody> body,
                                      const std::string& file, const Position& mid_ship) {
    auto force = std::make_shared<FrAbkowitzManoeuvringForce>(name, body.get(), file, mid_ship);
    body->AddExternalForce(force);
    return force;
  }

  std::shared_ptr<FrAbkowitzManoeuvringForce>
      make_abkowitz_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody> body,
                                      const JSONNode& node_man, const JSONNode& node_res,
                                      const Position& mid_ship, double draft_m, double lpp_m) {
    auto force = std::make_shared<FrAbkowitzManoeuvringForce>(name, body.get(), node_man, node_res,
                                                              mid_ship, draft_m, lpp_m);
    body->AddExternalForce(force);
    return force;
  }

}  // end namespace frydom
