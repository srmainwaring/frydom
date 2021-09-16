//
// Created by lletourn on 02/06/2021.
//

#include "FrCPPForce.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/logging/FrEventLogger.h"

namespace frydom {

  FrCPPForce::FrCPPForce(const std::string &name,
                         FrBody *body,
                         Position propellerPositionInBody,
                         const std::string &fileCoefficients,
                         FRAME_CONVENTION fc) :
      FrFourQuadrantPropellerForce(name, body, propellerPositionInBody, fileCoefficients, fc) {

  }

  double FrCPPForce::Ct(double beta) const {
    return m_coefficients.Eval("Ct", beta, GetPitchRatio()) + m_delta_ct * std::pow(GetPitchRatio() / 1.5, 8);
  }

  double FrCPPForce::Cq(double beta) const {
    return m_coefficients.Eval("Cq", beta, GetPitchRatio()) + m_delta_cq * std::pow(GetPitchRatio() / 1.5, 1);
  }

  void FrCPPForce::SetPitchRatio(double P_D) {
    m_pitchRatio = P_D;
  }

  double FrCPPForce::GetPitchRatio() const {
    return m_pitchRatio;
  }

  void frydom::FrCPPForce::ReadPropellerTable(const json &node) {

    try {
      auto type = node["type"].get<json::string_t>();
      //FIXME : verification du type necessite une convention de nommage des types
      if (type != "CPP") {
        event_logger::error("FrCPPForce", GetName(),
                            "file given is not a CPP propeller file coefficients");
        exit(EXIT_FAILURE);
      }
    } catch (json::parse_error &err) {
      event_logger::error("FrCPPForce", GetName(), "no reference in json file");
      exit(EXIT_FAILURE);
    }

    std::vector<double> advanceAngle, PD;
    std::vector<std::vector<double>> Ct, Cq;
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

  std::shared_ptr<FrCPPForce>
  make_controllable_pitch_propeller(const std::string &name,
                                    const std::shared_ptr<FrBody> &body,
                                    Position propellerPositionInBody,
                                    const std::string &fileCoefficients,
                                    FRAME_CONVENTION fc) {
    auto force = std::make_shared<FrCPPForce>(name, body.get(), propellerPositionInBody, fileCoefficients, fc);
    body->AddExternalForce(force);
    return force;
  }
}// end namespace frydom