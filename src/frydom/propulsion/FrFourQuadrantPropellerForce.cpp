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
    FrPropellerForce(name, body, propellerPositionInBody, fc) {

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

frydom::GeneralizedForce frydom::FrFourQuadrantPropellerForce::ComputeGeneralizedForceInWorld() {
  auto density = GetBody()->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

  auto uPA = ComputeLongitudinalVelocity();
  auto vp = 0.35 * GetDiameter() * GetRotationalVelocity(RADS);
  auto advanceAngle = atan2(uPA, GetScrewDirectionSign() * vp);

  auto squaredVelocity = uPA*uPA + vp*vp;

  auto T = 0.25 * density * Ct(advanceAngle) * squaredVelocity * MU_PI * GetDiameter();
  auto Q = 0.25 * density * Cq(advanceAngle) * squaredVelocity * MU_PI * GetDiameter() * GetDiameter() * GetScrewDirectionSign();

  auto force = GetBody()->ProjectVectorInWorld(Force(T, 0., 0.), NWU);
  auto torque = GetBody()->ProjectVectorInWorld(Force(Q, 0., 0.), NWU);

  return {force, torque};
}

void frydom::FrFourQuadrantPropellerForce::ReadCoefficientsFile() {
  // This function reads the rudder polar coefficients from a Json input file.

  // Loader.
  std::ifstream ifs(c_fileCoefficients);
  json j = json::parse(ifs);

  auto node = j["propeller"];

  try {
    m_name = node["name"].get<json::string_t>();
  } catch (json::parse_error &err) {
    event_logger::error("FrFourQuadrantPropellerForce", GetName(), "no name in json file");
    exit(EXIT_FAILURE);
  }

  try {
    m_reference = node["reference"].get<json::string_t>();
  } catch (json::parse_error &err) {
    event_logger::error("FrFourQuadrantPropellerForce", GetName(), "no reference in json file");
    exit(EXIT_FAILURE);
  }

  try {
    auto type = node["type"].get<json::string_t>();
    //FIXME : verification du type necessite une convention de nommage des types
//    if (type != "CPP") {
//      event_logger::error("FrCPPForce", GetName(), "file given is not a CPP propeller file coefficients");
//      exit(EXIT_FAILURE);
//    }
  } catch (json::parse_error &err) {
    event_logger::error("FrFourQuadrantPropellerForce", GetName(), "no reference in json file");
    exit(EXIT_FAILURE);
  }

  try {
    m_diameter = node["data"]["diameter_m"].get<double>();
  } catch (json::parse_error &err) {
    event_logger::error("FrFourQuadrantPropellerForce", GetName(), "no diameter in json file");
    exit(EXIT_FAILURE);
  }

}

void frydom::FrFourQuadrantPropellerForce::ReadPropellerTable(const json &node) {

  std::vector<double> advanceAngle, PD, Ct, Cq;
  int screwDirectionSign;

  try {
    auto screwDirection = node["open_water_table"]["screw_direction"].get<std::string>();
    if (screwDirection == "RIGHT_HANDED")
      screwDirectionSign = 1;
    else if (screwDirection == "LEFT_HANDED")
      screwDirectionSign = -1;
    else {
      event_logger::error("FrCPPForce", "", "wrong screw_direction value in open_water_table : RIGHT_HANDED or LEFT_HANDED");
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
    event_logger::error("FrCPPForce", "", "no Ct in open_water_table");
    exit(EXIT_FAILURE);
  }

  try {
    Cq = node["open_water_table"]["cq"].get<std::vector<double>>();
    m_coefficients.AddY("Cq", Cq);
  } catch (json::parse_error &err) {
    event_logger::error("FrCPPForce", "", "no Cq in open_water_table");
    exit(EXIT_FAILURE);
  }
}

std::shared_ptr<frydom::FrFourQuadrantPropellerForce>
frydom::make_four_quadrant_propeller_force(const std::string &name, FrBody *body, Position propellerPositionInBody,
                                           const std::string &fileCoefficients, FRAME_CONVENTION fc) {
  auto force = std::make_shared<FrFourQuadrantPropellerForce>(name, body, propellerPositionInBody, fileCoefficients, fc);
  body->AddExternalForce(force);
  return force;
}
