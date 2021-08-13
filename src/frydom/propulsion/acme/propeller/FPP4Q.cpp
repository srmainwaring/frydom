//
// Created by frongere on 04/08/2021.
//

#include "FPP4Q.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace acme {

  FPP4Q::FPP4Q(const acme::PropellerParams &params, const std::string &ct_cq_json_string) :
      PropellerBaseModel(params, ct_cq_json_string, PropellerModelType::E_FPP4Q) {
  }

  void FPP4Q::Compute(const double &water_density,
                      const double &u_NWU,
                      const double &v_NWU,
                      const double &rpm,
                      const double &pitch_ratio) const {

    if (!m_is_initialized) {
      std::cerr << "Propulsion model MUST be initialized before being used." << std::endl;
      exit(EXIT_FAILURE);
    }

    // Propeller advance velocity
    double uPA = GetPropellerAdvanceVelocity(u_NWU, v_NWU);

    // propeller rotation frequency in rps
    double n = rpm / 60.;

    // tangential blade velocity at 0.7R
    double vp = 0.7 * MU_PI * n * m_params.m_diameter_m;

    // Effective blade advance angle
    double gamma = std::atan2(uPA, vp);

    // Effective total velocity squared
    double vB2 = uPA * uPA + vp * vp;

    // Propeller disk area
    double Ad = MU_PI * m_params.m_diameter_m * m_params.m_diameter_m / 4.;

    // Get Coefficients
    double ct, cq;
    GetCtCq(gamma, pitch_ratio, ct, cq);

    // Propeller Thrust
    double _ct = ct + m_params.m_thrust_coefficient_correction;
    double propeller_thrust = 0.5 * water_density * vB2 * Ad * _ct;

    // Effective propeller thrust
    c_thrust_N = propeller_thrust * (1 - m_params.m_thrust_deduction_factor_0);

    // Torque
    double _cq = cq + m_params.m_torque_coefficient_correction;
    c_torque_Nm = 0.5 * water_density * vB2 * Ad * m_params.m_diameter_m * _cq;

    // Efficiency
    if (n != 0.) {
      double J = uPA / (n * m_params.m_diameter_m);
      c_efficiency = J * _ct / (MU_2PI * _cq);
    } else {
      c_efficiency = 0.; // TODO: voir si on met 0 ou 1...
    }

    // Power
    c_power_W = MU_2PI * n * c_torque_Nm;

  }

  void FPP4Q::GetCtCq(const double &gamma,
                      const double &pitch_ratio,
                      double &ct,
                      double &cq) const {

    ct = m_ct_ct_coeffs.Eval("ct", gamma);
    cq = m_ct_ct_coeffs.Eval("cq", gamma);
  }

  void FPP4Q::ParsePropellerPerformanceCurveJsonString() {

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

    auto beta = jnode["beta_deg"].get<std::vector<double>>();
    auto ct = jnode["ct"].get<std::vector<double>>();
    auto cq = jnode["cq"].get<std::vector<double>>();

    for (auto& b : beta) b *= DEG2RAD;

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

    m_ct_ct_coeffs.SetX(beta);
    m_ct_ct_coeffs.AddY("ct", ct);
    m_ct_ct_coeffs.AddY("cq", cq);
  }

}  // end namespace acme