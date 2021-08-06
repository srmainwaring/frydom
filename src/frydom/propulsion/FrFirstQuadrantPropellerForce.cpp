//
// Created by lletourn on 03/06/2021.
//

#include "FrFirstQuadrantPropellerForce.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/logging/FrEventLogger.h"

namespace frydom {


  FrFirstQuadrantPropellerForce::FrFirstQuadrantPropellerForce(const std::string &name, FrBody *body,
                                                               Position propellerPositionInBody,
                                                               const std::string &fileCoefficients, FRAME_CONVENTION fc)
      : FrPropellerForce(name, body, propellerPositionInBody, fc), c_fileCoefficients(fileCoefficients) {

  }

  double FrFirstQuadrantPropellerForce::kt(double J) const {
    return m_coefficients.Eval("kt", J);
  }

  double FrFirstQuadrantPropellerForce::kq(double J) const {
    return m_coefficients.Eval("kq", J);
  }

  GeneralizedForce FrFirstQuadrantPropellerForce::ComputeGeneralizedForceInBody() {

    auto density = GetBody()->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

    auto J = ComputeAdvanceRatio();
    auto n2 = GetRotationalVelocity(RPM) / 60.;
    n2 *= n2;
    c_thrust = density * std::pow(GetDiameter(), 4) * kt(J) * n2 * (1-m_thrust_deduction_factor);
    c_torque = density * std::pow(GetDiameter(), 5) * kq(J) * n2 * GetScrewDirectionSign();

    return {Force(c_thrust, 0., 0.), Torque(c_torque, 0., 0.)};
  }

  double FrFirstQuadrantPropellerForce::ComputeAdvanceRatio() {
    return 60. * ComputeLongitudinalVelocity() / (GetRotationalVelocity(RPM) * GetDiameter());
  }

  void FrFirstQuadrantPropellerForce::ReadCoefficientsFile() {
// This function reads the rudder polar coefficients from a Json input file.

    std::vector<double> advanceRatio, kt, kq;
    int screwDirectionSign;

    // Loader.
    std::ifstream ifs(c_fileCoefficients);
    json j = json::parse(ifs);

    auto node = j["propeller"];

//    try {
//      m_name = node["name"].get<json::string_t>();
//    } catch (json::parse_error &err) {
//      event_logger::error("FrFirstQuadrantPropellerForce", GetName(), "no name in json file");
//      exit(EXIT_FAILURE);
//    }

    try {
      m_reference = node["reference"].get<json::string_t>();
    } catch (json::parse_error &err) {
      event_logger::error("FrFirstQuadrantPropellerForce", GetName(), "no reference in json file");
      exit(EXIT_FAILURE);
    }

    try {
      auto type = node["type"].get<json::string_t>();
      if (type != "FPP_FIRST_QUADRANT") {
        event_logger::error("FrFirstQuadrantPropellerForce", GetName(),
                            "file given is not a FPP First Quadrant propeller file coefficients");
        exit(EXIT_FAILURE);
      }
    } catch (json::parse_error &err) {
      event_logger::error("FrFirstQuadrantPropellerForce", GetName(), "no reference in json file");
      exit(EXIT_FAILURE);
    }

//    try {
//      m_diameter = node["data"]["diameter_m"].get<double>();
//    } catch (json::parse_error &err) {
//      event_logger::error("FrFirstQuadrantPropellerForce", GetName(), "no diameter in json file");
//      exit(EXIT_FAILURE);
//    }

    try {
      auto screwDirection = node["open_water_table"]["screw_direction"].get<std::string>();
      if (screwDirection == "RIGHT_HANDED")
        screwDirectionSign = 1;
      else if (screwDirection == "LEFT_HANDED")
        screwDirectionSign = -1;
      else {
        event_logger::error("FrFirstQuadrantPropellerForce", "",
                            "wrong screw_direction value in open_water_table : RIGHT_HANDED or LEFT_HANDED");
        exit(EXIT_FAILURE);
      }
    } catch (json::parse_error &err) {
      event_logger::error("FrFirstQuadrantPropellerForce", "", "no screw_direction in open_water_table");
      exit(EXIT_FAILURE);
    }

    try {
      advanceRatio = node["open_water_table"]["J"].get<std::vector<double>>();
      m_coefficients.SetX(advanceRatio);
    } catch (json::parse_error &err) {
      event_logger::error("FrFirstQuadrantPropellerForce", "", "no J in open_water_table");
      exit(EXIT_FAILURE);
    }

    try {
      kt = node["open_water_table"]["kt"].get<std::vector<double>>();
      m_coefficients.AddY("kt", kt);
    } catch (json::parse_error &err) {
      event_logger::error("FrFirstQuadrantPropellerForce", "", "no kt in open_water_table");
      exit(EXIT_FAILURE);
    }

    try {
      kq = node["open_water_table"]["kq"].get<std::vector<double>>();
      for (auto &coeff:kq) coeff *= screwDirectionSign;
      m_coefficients.AddY("kq", kq);
    } catch (json::parse_error &err) {
      event_logger::error("FrFirstQuadrantPropellerForce", "", "no kq in open_water_table");
      exit(EXIT_FAILURE);
    }

  }

  std::shared_ptr<FrFirstQuadrantPropellerForce>
  make_first_quadrant_propeller_force(const std::string &name,
                                      const std::shared_ptr<FrBody> &body,
                                      Position propellerPositionInBody,
                                      const std::string &fileCoefficients,
                                      FRAME_CONVENTION fc) {
    auto force = std::make_shared<FrFirstQuadrantPropellerForce>(name, body.get(), propellerPositionInBody,
                                                                 fileCoefficients, fc);
    body->AddExternalForce(force);
    return force;
  }
}// end namespace frydom