//
// Created by lletourn on 01/06/2021.
//

#include "FrFlapRudderForce.h"
#include "frydom/logging/FrEventLogger.h"


namespace frydom {

  FrFlapRudderForce::FrFlapRudderForce(const std::string &name, frydom::FrBody *body,
                                               const std::string &fileCoefficients, const std::shared_ptr<FrNode> &node)
      : FrRudderForce(name, body, fileCoefficients, node), m_flapLaw(1.) {}

  double FrFlapRudderForce::GetFlapAngle() const {
    return m_flapLaw * m_rudderAngle;
  }

  mathutils::Vector3d<double> FrFlapRudderForce::GetCoefficients(double attackAngle) const {
    return {m_coefficients.Eval("drag", attackAngle, GetFlapAngle()),
            m_coefficients.Eval("lift", attackAngle, GetFlapAngle()),
            m_coefficients.Eval("torque", attackAngle, GetFlapAngle())};
  }

  void FrFlapRudderForce::ReadCoefficientsFile() {
    // This function reads the rudder polar coefficients from a Json input file.

    std::vector<double> flap_angle, attack_angle;
    std::vector<std::vector<double>> Cd, Cl, Cm;


    // Loader.
    std::ifstream ifs(c_fileCoefficients);
    json j = json::parse(ifs);

    auto node = j["rudder"];

    try {
      m_name = node["name"].get<json::string_t>();
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", GetName(), "no name in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_reference = node["reference"].get<json::string_t>();
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", GetName(), "no reference in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_flapLaw = node["flap_law"]["coeff"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", GetName(), "no flap law coefficient in json file");
      exit(EXIT_FAILURE);
    }

    try {
      m_projectedLateralArea = node["data"]["area_m2"].get<double>();
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", GetName(), "no area in json file");
      exit(EXIT_FAILURE);
    }

    try {
      flap_angle = node["load_coefficients"]["flap_angle_deg"].get<std::vector<double>>();
      for (auto &angle:flap_angle) angle *= DEG2RAD;
      m_coefficients.SetY(flap_angle);
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", "", "no flap_angle_deg in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      attack_angle = node["load_coefficients"]["flow_incidence_on_main_rudder_deg"].get<std::vector<double>>();
      for (auto &angle:attack_angle) angle *= DEG2RAD;
      m_coefficients.SetX(attack_angle);
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", "", "no flow_incidence_on_main_rudder_deg in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      Cd = node["load_coefficients"]["Cd"].get<std::vector<std::vector<double>>>();
      Eigen::MatrixXd mat(flap_angle.size(), attack_angle.size());
      for (int i = 0; i < flap_angle.size(); i++)
        mat.row(i) = Eigen::VectorXd::Map(&Cd[i][0], Cd[i].size());
//    mat.transposeInPlace();
      std::vector<double> coeff2 = {mat.data(), mat.data() + mat.rows() * mat.cols()};

      m_coefficients.AddData("drag", coeff2);
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", "", "no Cd in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      Cl = node["load_coefficients"]["Cl"].get<std::vector<std::vector<double>>>();
      Eigen::MatrixXd mat(flap_angle.size(), attack_angle.size());
      for (int i = 0; i < flap_angle.size(); i++)
        mat.row(i) = Eigen::VectorXd::Map(&Cl[i][0], Cl[i].size());
//    mat.transposeInPlace();
      std::vector<double> coeff = {mat.data(), mat.data() + mat.rows() * mat.cols()};
      m_coefficients.AddData("lift", coeff);
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", "", "no Cl in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      Cm = node["load_coefficients"]["Cm"].get<std::vector<std::vector<double>>>();
      Eigen::MatrixXd mat(flap_angle.size(), attack_angle.size());
      for (int i = 0; i < flap_angle.size(); i++)
        mat.row(i) = Eigen::VectorXd::Map(&Cm[i][0], Cm[i].size());
//    mat.transposeInPlace();
      std::vector<double> coeff = {mat.data(), mat.data() + mat.rows() * mat.cols()};
      m_coefficients.AddData("torque", coeff);
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", "", "no Cm in load_coefficients");
      exit(EXIT_FAILURE);
    }

  }

} // end namespace frydom