//
// Created by lletourn on 01/06/2021.
//

#include "FrFlapRudderForce.h"
#include "frydom/logging/FrEventLogger.h"


namespace frydom {

  FrFlapRudderForce::FrFlapRudderForce(const std::string &name, frydom::FrBody *body,
                                       const std::shared_ptr<FrNode> &node,
                                       const std::string &fileCoefficients)
      : FrRudderForce(name, body, node, fileCoefficients), m_flapLaw(1.) {}

  double FrFlapRudderForce::GetFlapAngle() const {
    return m_flapLaw * GetRudderAngle(RAD);
  }

  double FrFlapRudderForce::GetLiftCoefficient(double attackAngle) const {
    return m_coefficients.Eval("lift", attackAngle, GetFlapAngle());
  }

  double FrFlapRudderForce::GetDragCoefficient(double attackAngle) const {
    return m_coefficients.Eval("drag", attackAngle, GetFlapAngle());
  }

  double FrFlapRudderForce::GetTorqueCoefficient(double attackAngle) const {
    return m_coefficients.Eval("torque", attackAngle, GetFlapAngle());
  }

  void FrFlapRudderForce::ReadCoefficientsFile() {
    // This function reads the rudder polar coefficients from a Json input file.

    std::vector<double> flap_angle, attack_angle;
    std::vector<std::vector<double>> Cd, Cl, Cm;
    FRAME_CONVENTION fc;
    DIRECTION_CONVENTION dc;


    // Loader.
    std::ifstream ifs(c_fileCoefficients);
    json j = json::parse(ifs);

    auto node = j["rudder"];

    try {
      m_reference = node["reference"].get<json::string_t>();
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", GetName(), "no reference in json file");
      exit(EXIT_FAILURE);
    }

    try {
      fc = STRING2FRAME(node["frame_convention"].get<json::string_t>());
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", "", "no frame_convention in load_coefficients");
      exit(EXIT_FAILURE);
    }

    try {
      dc = STRING2DIRECTION(node["direction_convention"].get<json::string_t>());
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", "", "no direction_convention in load_coefficients");
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
      Cm = node["load_coefficients"]["Cn"].get<std::vector<std::vector<double>>>();
      Eigen::MatrixXd mat(flap_angle.size(), attack_angle.size());
      for (int i = 0; i < flap_angle.size(); i++)
        mat.row(i) = Eigen::VectorXd::Map(&Cm[i][0], Cm[i].size());
//    mat.transposeInPlace();
      std::vector<double> coeff = {mat.data(), mat.data() + mat.rows() * mat.cols()};
      m_coefficients.AddData("torque", coeff);
    } catch (json::parse_error &err) {
      event_logger::error("FrFlapRudderForce", "", "no Cn in load_coefficients");
      exit(EXIT_FAILURE);
    }

    //TODO : gerer les changements de conventions NWU/NED et GOT/COMEFROM,
    //             les conventions d'angles ([pi,pi] ou [0, 2pi]
    //             les symetries, etc.

  }

  void FrFlapRudderForce::SetFlapLaw(double coefficient) {
    m_flapLaw = coefficient;
  }

  double FrFlapRudderForce::GetFlapLawCoefficient() const {
    return m_flapLaw;
  }

  std::shared_ptr<FrFlapRudderForce>
  make_flap_rudder_force(const std::string &name,
                         const std::shared_ptr<FrBody> &body,
                         const std::shared_ptr<FrNode> &node,
                         const std::string &fileCoefficients) {
    auto force = std::make_shared<FrFlapRudderForce>(name, body.get(), node, fileCoefficients);
    body->AddExternalForce(force);
    return force;
  }
} // end namespace frydom