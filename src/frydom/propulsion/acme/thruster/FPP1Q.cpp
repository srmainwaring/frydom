//
// Created by frongere on 04/08/2021.
//

#include "FPP1Q.h"

#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace acme {

  FPP1Q::FPP1Q(const ThrusterParams &params, const std::string &kt_kq_json_string) :
      ThrusterBaseModel(params, kt_kq_json_string, ThrusterModelType::E_FPP1Q) {
  }

//  void FPP1Q::Initialize() {
//    ThrusterBaseModel::Initialize();
//  }

  void FPP1Q::Compute(const double &water_density,
                      const double &u_NWU,
                      const double &v_NWU,
                      const double &rpm,
                      const double &pitch_ratio) const {

    if (!m_is_initialized) {
      std::cerr << "Propulsion model MUST be initialized before being used." << std::endl;
      exit(EXIT_FAILURE);
    }

    if (u_NWU < 0.) {
      std::cerr << "This first quadrant model is only applicable for positive vessel forward speed" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Propeller advance velocity
    double uPA = GetPropellerAdvanceVelocity(u_NWU, v_NWU);

    // propeller rotation frequency in Hz
    double n = rpm / 60.;

    // Advance ratio
    double J = uPA / (n * m_params.m_diameter_m);

    // Get Coefficients
    double kt, kq;
    GetKtKq(J, kt, kq);

    // Propeller Thrust
    double n2 = n * n;
    double D4 = std::pow(m_params.m_diameter_m, 4);
    double _kt = kt + m_params.m_thrust_coefficient_correction;
    double propeller_thrust = water_density * n2 * D4 * _kt;

    // Effective propeller thrust
    c_thrust_N = propeller_thrust * (1 - m_params.m_thrust_deduction_factor_0);

    // Torque
    double _kq = kq + m_params.m_torque_coefficient_correction;
//    c_torque_Nm = water_density * n2 * std::pow(m_params.m_diameter_m, 5) * _kq * GetScrewDirectionSign();
    c_torque_Nm = water_density * n2 * D4 * m_params.m_diameter_m * _kq;

    // Efficiency
    c_efficiency = J * _kt / (MU_2PI * _kq);

    // Power
    c_power_W =  MU_2PI * n * c_torque_Nm;

  }

  void FPP1Q::ParsePropellerPerformanceCurveJsonString() {

    /**
     * ATTENTION. Note aux developpeurs
     *
     * Si un json ne passe pas, ne changez pas ce code pour le faire fonctionner mais plutot en parler car c'est ce
     * code de parsing json qui fait foi en terme de convention sur le formattage des json. On peut faire evoluer le
     * format mais il est necessaire d'avoir une discussion prealable.
     *
     * Merci :)
     *
     * On remarquera en outre que le choix a ete fait de faire une lazy initialisation en stockant temporairement non
     * pas le path vers le fichier json mais la chaine de caractere json a la place. C'est un choix delibere qui pourra
     * etre etendu en tant que pattern a toutes nos classes necessitant la lecture de parametres sous format json.
     * La raison est que cela permet de preparer l'instantiation a la volee d'objets en utilisant json comme format
     * de serialisation des parametres. Cela rend possible la creation a distance de ces objets sans avoir a rendre
     * disponible un fichier sur le disque (sans IO disque donc). Cela sera utilise plus tard.
     *
     */

    std::cout << m_temp_perf_data_json_string << std::endl;

    auto jnode = json::parse(m_temp_perf_data_json_string);
    m_temp_perf_data_json_string.clear();

    auto j = jnode["j"].get<std::vector<double>>();
    auto kt = jnode["kt"].get<std::vector<double>>();
    auto kq = jnode["kq"].get<std::vector<double>>();

//    // Only one
//    if (screw_direction == "LEFT_HANDED") {
//      for (auto &c : kq) {
//        c = -c;
//      }
//    } else if (screw_direction == "RIGHT_HANDED") {
//      // Nothing
//    } else {
//      std::cerr << "Unknown screw direction " << screw_direction << std::endl;
//      exit(EXIT_FAILURE);
//    }

    m_kt_kq_coeffs.SetX(j);
    m_kt_kq_coeffs.AddY("kt", kt);
    m_kt_kq_coeffs.AddY("kq", kq);
  }

  inline void FPP1Q::GetKtKq(const double &J, double &kt, double &kq) const {
    kt = m_kt_kq_coeffs.Eval("kt", J);
    kq = m_kt_kq_coeffs.Eval("kq", J);
  }

}  // end namespace acme
