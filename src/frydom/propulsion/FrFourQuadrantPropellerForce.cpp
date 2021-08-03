//
// Created by lletourn on 08/06/2021.
//

#include "FrFourQuadrantPropellerForce.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/logging/FrEventLogger.h"

frydom::FrFourQuadrantPropellerForce::FrFourQuadrantPropellerForce(const std::string &name, FrBody *body,
                                                                   Position propellerPositionInBody,
                                                                   const std::string &fileCoefficients,
                                                                   FRAME_CONVENTION fc) :
    FrPropellerForce(name, body, propellerPositionInBody, fc), c_fileCoefficients(fileCoefficients) {

}

double frydom::FrFourQuadrantPropellerForce::Ct(double gamma) const {
  return m_coefficients.Eval("ct", gamma);
}

double frydom::FrFourQuadrantPropellerForce::Cq(double gamma) const {
  return m_coefficients.Eval("cq", gamma);
}

double frydom::FrFourQuadrantPropellerForce::ComputeAdvanceAngle() {
  auto uPA = ComputeLongitudinalVelocity();
  auto vp = 0.35 * GetDiameter() * GetRotationalVelocity(RADS);

  return atan2(uPA, GetScrewDirectionSign() * vp);
}

frydom::GeneralizedForce frydom::FrFourQuadrantPropellerForce::ComputeGeneralizedForceInBody() {
  auto density = GetBody()->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

  auto uPA = ComputeLongitudinalVelocity();
  auto vp = 0.35 * GetDiameter() * GetRotationalVelocity(RADS);
  auto advanceAngle = atan2(uPA, GetScrewDirectionSign() * vp);

  auto squaredVelocity = uPA * uPA + vp * vp;

  c_thrust = 0.25 * density * Ct(advanceAngle) * squaredVelocity * MU_PI * GetDiameter();
  c_torque = 0.25 * density * Cq(advanceAngle) * squaredVelocity * MU_PI * GetDiameter() * GetDiameter() *
           GetScrewDirectionSign();

  return {Force(c_thrust, 0., 0.), Torque(c_torque, 0., 0.)};
}

void frydom::FrFourQuadrantPropellerForce::ReadCoefficientsFile() {
  // This function reads the rudder polar coefficients from a Json input file.

  // Loader.
  std::ifstream ifs(c_fileCoefficients);
  json j = json::parse(ifs);

  auto node = j["propeller"];

//  try {
//    m_name = node["name"].get<json::string_t>();
//  } catch (json::parse_error &err) {
//    event_logger::error("FrFourQuadrantPropellerForce", GetName(), "no name in json file");
//    exit(EXIT_FAILURE);
//  }

  try {
    m_reference = node["reference"].get<json::string_t>();
  } catch (json::parse_error &err) {
    event_logger::error("FrFourQuadrantPropellerForce", GetName(), "no reference in json file");
    exit(EXIT_FAILURE);
  }

  ReadPropellerTable(node);

}

void frydom::FrFourQuadrantPropellerForce::ReadPropellerTable(const json &node) {

  try {
    auto type = node["type"].get<json::string_t>();
    //FIXME : verification du type necessite une convention de nommage des types
    if (type != "FPP_FOUR_QUADRANTS") {
      event_logger::error("FrFourQuadrantPropellerForce", GetName(),
                          "file given is not a FPP Four Quadrants propeller file coefficients");
      exit(EXIT_FAILURE);
    }
  } catch (json::parse_error &err) {
    event_logger::error("FrFourQuadrantPropellerForce", GetName(), "no reference in json file");
    exit(EXIT_FAILURE);
  }

  std::vector<double> advanceAngle, PD, Ct, Cq;
  int screwDirectionSign;

  try {
    auto screwDirection = node["open_water_table"]["screw_direction"].get<std::string>();
    if (screwDirection == "RIGHT_HANDED")
      screwDirectionSign = 1;
    else if (screwDirection == "LEFT_HANDED")
      screwDirectionSign = -1;
    else {
      event_logger::error("FrCPPForce", "",
                          "wrong screw_direction value in open_water_table : RIGHT_HANDED or LEFT_HANDED");
      exit(EXIT_FAILURE);
    }
  } catch (json::parse_error &err) {
    event_logger::error("FrCPPForce", "", "no screw_direction in open_water_table");
    exit(EXIT_FAILURE);
  }

  try {
    advanceAngle = node["open_water_table"]["beta_deg"].get<std::vector<double>>();
    for (auto &angle:advanceAngle) angle *= DEG2RAD;
    m_coefficients.SetX(advanceAngle);
  } catch (json::parse_error &err) {
    event_logger::error("FrCPPForce", "", "no beta_deg in open_water_table");
    exit(EXIT_FAILURE);
  }

  try {
    Ct = node["open_water_table"]["ct"].get<std::vector<double>>();
    m_coefficients.AddY("Ct", Ct);
  } catch (json::parse_error &err) {
    event_logger::error("FrCPPForce", "", "no ct in open_water_table");
    exit(EXIT_FAILURE);
  }

  try {
    Cq = node["open_water_table"]["cq"].get<std::vector<double>>();
    for (auto &coeff:Cq) coeff *= screwDirectionSign;
    m_coefficients.AddY("Cq", Cq);
  } catch (json::parse_error &err) {
    event_logger::error("FrCPPForce", "", "no cq in open_water_table");
    exit(EXIT_FAILURE);
  }
}

std::shared_ptr<frydom::FrFourQuadrantPropellerForce>
frydom::make_four_quadrant_propeller_force(const std::string &name,
                                           const std::shared_ptr<FrBody> &body,
                                           Position propellerPositionInBody,
                                           const std::string &fileCoefficients,
                                           FRAME_CONVENTION fc) {
  auto force = std::make_shared<FrFourQuadrantPropellerForce>(name, body.get(), propellerPositionInBody,
                                                              fileCoefficients, fc);
  body->AddExternalForce(force);
  return force;
}
