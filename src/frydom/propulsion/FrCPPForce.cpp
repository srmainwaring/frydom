//
// Created by lletourn on 02/06/2021.
//

#include "FrCPPForce.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/logging/FrEventLogger.h"

namespace frydom {

  FrCPPForce::FrCPPForce(const std::string &name, frydom::FrBody *body, frydom::Position propellerPositionInBody,
                         const std::string& fileCoefficients) : FrPropellerForce(name, body, propellerPositionInBody),
                                                                c_fileCoefficients(fileCoefficients),
                                                                m_screwDirection(RIGHT_HANDED){

  }

  void FrCPPForce::ReadCoefficientsFile() {
    // This function reads the rudder polar coefficients from a Json input file.

    std::vector<double> advanceAngle, PD;
    std::vector<std::vector<double>> Ct, Cq;
    int screwDirectionSign;


    // Loader.
    std::ifstream ifs(c_fileCoefficients);
    json j = json::parse(ifs);

    auto node = j["propeller"];

    try {
      m_name = node["name"].get<json::string_t>();
    } catch (json::parse_error &err) {
      event_logger::error("FrCPPForce", GetName(), "no name in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_reference = node["reference"].get<json::string_t>();
    } catch (json::parse_error &err) {
      event_logger::error("FrCPPForce", GetName(), "no reference in json file");
      exit(EXIT_FAILURE);
    }

    try {
      auto type = node["type"].get<json::string_t>();
      if (type != "CPP") {
        event_logger::error("FrCPPForce", GetName(), "file given is not a CPP propeller file coefficients");
        exit(EXIT_FAILURE);
      }
    } catch (json::parse_error &err) {
      event_logger::error("FrCPPForce", GetName(), "no reference in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_diameter = node["data"]["diameter_m"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrCPPForce", GetName(), "no diameter in json file");
      exit(EXIT_FAILURE);
    }

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
      PD = node["open_water_table"]["p_d"].get<std::vector<double>>();
      m_coefficients.SetY(PD);
    } catch (json::parse_error &err) {
      event_logger::error("FrCPPForce", "", "no p_d in open_water_table");
      exit(EXIT_FAILURE);
    }

    try {
      Ct = node["open_water_table"]["ct"].get<std::vector<std::vector<double>>>();
      Eigen::MatrixXd mat(PD.size(), advanceAngle.size());
      for (int i = 0; i < PD.size(); i++) {
        mat.row(i) = Eigen::VectorXd::Map(&Ct[i][0], Ct[i].size());
      }
//    mat.transposeInPlace();
      std::vector<double> coeff = {mat.data(), mat.data() + mat.rows() * mat.cols()};

      m_coefficients.AddData("Ct", coeff);
    } catch (json::parse_error &err) {
      event_logger::error("FrCPPForce", "", "no Ct in open_water_table");
      exit(EXIT_FAILURE);
    }

    try {
      Cq = node["open_water_table"]["cq"].get<std::vector<std::vector<double>>>();
      Eigen::MatrixXd mat(PD.size(), advanceAngle.size());
      for (int i = 0; i < PD.size(); i++)
        mat.row(i) = screwDirectionSign * Eigen::VectorXd::Map(&Cq[i][0], Cq[i].size());
//    mat.transposeInPlace();
      std::vector<double> coeff = {mat.data(), mat.data() + mat.rows() * mat.cols()};

      m_coefficients.AddData("Cq", coeff);
    } catch (json::parse_error &err) {
      event_logger::error("FrCPPForce", "", "no Cq in open_water_table");
      exit(EXIT_FAILURE);
    }
  }

  GeneralizedForce FrCPPForce::ComputeGeneralizedForceInWorld() {

    auto density = GetBody()->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

    auto uPA = ComputeLongitudinalVelocity();
    auto vp = 0.35 * GetDiameter() * GetRotationalVelocity(RADS);
    auto advanceAngle = atan2(uPA, GetScrewDirectionSign() * vp);

    auto squaredVelocity = uPA*uPA + vp*vp;

    auto T = 0.25 * density * Ct(advanceAngle, GetPitchRatio()) * squaredVelocity * MU_PI * GetDiameter();
    auto Q = 0.25 * density * Cq(advanceAngle, GetPitchRatio()) * squaredVelocity * MU_PI * GetDiameter() * GetDiameter();

    auto force = GetBody()->ProjectVectorInWorld(Force(T, 0., 0.), NWU);
    auto torque = GetBody()->ProjectVectorInWorld(Force(Q, 0., 0.), NWU);

    return {force, torque};

  }

  double FrCPPForce::Ct(double beta, double PD) const {
    return m_coefficients.Eval("Ct", beta, PD);
  }

  double FrCPPForce::Cq(double beta, double PD) const {
    return m_coefficients.Eval("Cq", beta, PD);
  }

  void FrCPPForce::SetPitchRatio(double PD) {
    m_pitchRatio = PD;
  }

  double FrCPPForce::GetPitchRatio() const {
    return m_pitchRatio;
  }

  double FrCPPForce::ComputeAdvanceAngle() {
    auto uPA = GetLongitudinalVelocity();
    auto vp = 0.35 * GetDiameter() * GetRotationalVelocity(RADS);

    return atan2(uPA, GetScrewDirectionSign() * vp);
  }

  void FrCPPForce::SetScrewDirection(FrCPPForce::SCREW_DIRECTION dir) {
    m_screwDirection = dir;
  }

  signed int FrCPPForce::GetScrewDirectionSign() const {
    signed int result = 0;
    switch (m_screwDirection) {
      case LEFT_HANDED: {
        result = -1;
        break;
      }
      case RIGHT_HANDED: {
        result = 1;
        break;
      }
    }
    return result;
  }

}// end namespace frydom