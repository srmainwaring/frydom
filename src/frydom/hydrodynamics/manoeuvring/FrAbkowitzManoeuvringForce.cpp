//
// Created by frongere on 12/10/2021.
//

#include "FrAbkowitzManoeuvringForce.h"
#include "frydom/io/JSONNode.h"

namespace frydom {

  FrAbkowitzManoeuvringForce::FrAbkowitzManoeuvringForce(const std::string &name,
                                                         FrBody *body,
                                                         const std::string &file) :
      FrForce(name, "FrManoeuvringForce", body),
      c_filepath(file),
      m_Xvv(0.), m_Xvvvv(0.), m_Xvvu(0.), m_Xrr(0.), m_Xrru(0.), m_Xvr(0.), m_Xvru(0.),
      m_Yv(0.), m_Yvvv(0.), m_Yvrr(0.), m_Yvu(0.), m_Yvuu(0.), m_Yr(0.), m_Yrrr(0.), m_Yvvr(0.), m_Yru(0.), m_Yruu(0.),
      m_Nv(0.), m_Nvvv(0.), m_Nvrr(0.), m_Nvu(0.), m_Nvuu(0.), m_Nr(0.), m_Nrrr(0.), m_Nvvr(0.), m_Nru(0.), m_Nruu(0.) {

  }

  void FrAbkowitzManoeuvringForce::Initialize() {
    FrForce::Initialize();
    LoadManoeuvringData();
  }

  void FrAbkowitzManoeuvringForce::Compute(double time) {

  }

  void FrAbkowitzManoeuvringForce::DefineLogMessages() {
    FrForce::DefineLogMessages();
  }

  void FrAbkowitzManoeuvringForce::LoadManoeuvringData() {
    JSONNode node(c_filepath);

    JSONNode man_node = node({"manoeuvring_model"});

    if (man_node.get_val<std::string>("type") != "abkowitz") {
      std::cerr << "Manoeuvring model enclosed in json file "
                << c_filepath
                << " was intended to be of type abkowitz but type was "
                << man_node.get_val<std::string>("type")
                << std::endl
                << "Aborting..."
                << std::endl;
      exit(EXIT_FAILURE);
    }

    auto file_format_version = man_node.get_val<std::string>("file_format_version");

    m_Lpp = man_node({"hull_characteristics"}).get_val<double>("length_m");
    m_draft = man_node({"hull_characteristics"}).get_val<double>("draft_m");

    auto wave_resistance_file = man_node.get_filepath("wave_resistance_file");

    JSONNode coeffs_node = man_node({"coefficients"});

    m_Xvv = coeffs_node.get_val<double>("Xvv", 0.0);
    m_Xvvvv = coeffs_node.get_val<double>("Xvvvv", 0.0);
    m_Xvvu = coeffs_node.get_val<double>("Xvvu", 0.0);
    m_Xrr = coeffs_node.get_val<double>("Xrr", 0.0);
    m_Xrru = coeffs_node.get_val<double>("Xrru", 0.0);
    m_Xvr = coeffs_node.get_val<double>("Xvr", 0.0);
    m_Xvru = coeffs_node.get_val<double>("Xvru", 0.0);
    m_Yv = coeffs_node.get_val<double>("Yv", 0.0);
    m_Yvvv = coeffs_node.get_val<double>("Yvvv", 0.0);
    m_Yvrr = coeffs_node.get_val<double>("Yvrr", 0.0);
    m_Yvu = coeffs_node.get_val<double>("Yvu", 0.0);
    m_Yvuu = coeffs_node.get_val<double>("Yvuu", 0.0);
    m_Yr = coeffs_node.get_val<double>("Yr", 0.0);
    m_Yrrr = coeffs_node.get_val<double>("Yrrr", 0.0);
    m_Yvvr = coeffs_node.get_val<double>("Yvvr", 0.0);
    m_Yru = coeffs_node.get_val<double>("Yru", 0.0);
    m_Yruu = coeffs_node.get_val<double>("Yruu", 0.0);
    m_Nv = coeffs_node.get_val<double>("Nv", 0.0);
    m_Nvvv = coeffs_node.get_val<double>("Nvvv", 0.0);
    m_Nvrr = coeffs_node.get_val<double>("Nvrr", 0.0);
    m_Nvu = coeffs_node.get_val<double>("Nvu", 0.0);
    m_Nvuu = coeffs_node.get_val<double>("Nvuu", 0.0);
    m_Nr = coeffs_node.get_val<double>("Nr", 0.0);
    m_Nrrr = coeffs_node.get_val<double>("Nrrr", 0.0);
    m_Nvvr = coeffs_node.get_val<double>("Nvvr", 0.0);
    m_Nru = coeffs_node.get_val<double>("Nru", 0.0);
    m_Nruu = coeffs_node.get_val<double>("Nruu", 0.0);
  }

}  // end namespace frydom
