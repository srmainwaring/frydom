//
// Created by frongere on 26/02/2021.
//

#ifndef FRYDOM_JSONNODE_H
#define FRYDOM_JSONNODE_H

#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>
#include <filesystem>

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/propulsion/FrPropellerType.h"


namespace fs = std::filesystem;
using json = nlohmann::json;

namespace frydom {

  class JSONNode {

   public:

    explicit JSONNode(const std::string &json_filename, const char &comment_char = '_') :
        m_json_file_path(json_filename),
        m_comment_char(comment_char),
        m_root("") {

      if (!fs::exists(json_filename)) {
        std::cerr << "JSON file " << json_filename << " not found" << std::endl;
        exit(EXIT_FAILURE);
      }

      std::ifstream ifs(json_filename);
      m_json_node = json::parse(ifs);
    }

    JSONNode(JSONNode *jnode, json node) {

    }

    bool exists(const std::string &key) const {
      return m_json_node.find(key) != m_json_node.end();
    }

    // TODO: avoir une version qui permet de specifier des valeurs par defaut lors de la lecture
    JSONNode get_node(const std::vector<std::string> &key_list) const {
      JSONNode node = *this;
      for (const auto &key: key_list) {
        node.next_node(key);
      }
      return node;
    }

    template<typename T>
    T get_val(const std::string &key) const {
      return get_node({key}).m_json_node.get<T>();
    }

    std::string get_json_file() const {
      return m_json_file_path;
    }

    std::string get_json_file_root_directory() const {
      return fs::path(m_json_file_path).remove_filename();
    }

    using const_iterator = json::const_iterator;

    const_iterator begin() const {
      // FIXME: l'implementation actuelle permet de sauter les commentaires commencant par '_' car la lib json place
      // tous ces champs en premier lors des iterations. La methode n'est donc pas robuste car si le comportement change
      // plus rien ne fonctionne...
      auto iter = m_json_node.cbegin();

      std::string key = iter.key();
      // Skipping keys starting by m_comment_char that are considered as comments
      while (!key.rfind(m_comment_char, 0)) {
        iter++;
        key = iter.key();
      }

      return iter;
    }

    const_iterator end() const {
      return m_json_node.cend();
    }

    /// Moves the current node state to the next node
    void next_node(const std::string &key) {
      if (exists(key)) {
        m_json_node = m_json_node[key];
        m_root += "/" + key;
      } else {
        std::cerr << "In JSON file " << m_json_file_path << std::endl;
        std::cerr << "JSON Node " << m_root << "/" << key << " is missing" << std::endl;
        exit(EXIT_FAILURE);
      }
    }

   private:
    std::string m_json_file_path;
    std::string m_root;
    json m_json_node;
    char m_comment_char;

  };


/**
 * In the following, special template specification for FRyDoM are defined
 */

  template<>
  inline frydom::Position JSONNode::get_val<frydom::Position>(const std::string &key) const {
    auto pos = get_val<std::vector<double>>(key);
    return {pos[0], pos[1], pos[2]};
  }

  template<>
  inline frydom::FRAME_CONVENTION JSONNode::get_val<frydom::FRAME_CONVENTION>(const std::string &key) const {
    auto fc_str = get_val<std::string>(key);
    frydom::FRAME_CONVENTION fc;
    if (fc_str == "NED") {
      fc = frydom::FRAME_CONVENTION::NED;
    } else if (fc_str == "NWU") {
      fc = frydom::FRAME_CONVENTION::NWU;
    } else {
      fs::path dir_path = get_json_file_root_directory();
      std::cerr << "In JSON file " << dir_path / m_json_file_path << std::endl;
      std::cerr << "ERROR at " << m_root << "/" << key << std::endl;
      std::cerr << "Unknown frame convention keyword " << fc_str << ". Only NED and NWU are accepted." << std::endl;
      exit(EXIT_FAILURE);
    }
    return fc;
  }

  template<>
  inline frydom::DIRECTION_CONVENTION JSONNode::get_val<frydom::DIRECTION_CONVENTION>(const std::string &key) const {
    auto fc_str = get_val<std::string>(key);
    frydom::DIRECTION_CONVENTION fc;
    if (fc_str == "GOTO") {
      fc = frydom::DIRECTION_CONVENTION::GOTO;
    } else if (fc_str == "COMEFROM") {
      fc = frydom::DIRECTION_CONVENTION::COMEFROM;
    } else {
      fs::path dir_path = get_json_file_root_directory();
      std::cerr << "In JSON file " << dir_path / m_json_file_path << std::endl;
      std::cerr << "ERROR at " << m_root << "/" << key << std::endl;
      std::cerr << "Unknown direction convention keyword " << fc_str << ". Only GOTO and COMEFROM are accepted."
                << std::endl;
      exit(EXIT_FAILURE);
    }
    return fc;
  }

  template<>
  inline frydom::PropellerModelType JSONNode::get_val<frydom::PropellerModelType>(const std::string &key) const {
    auto pt_str = get_val<std::string>(key);
    frydom::PropellerModelType pt;
    if (pt_str == "FPP_1Q") {
      pt = frydom::PropellerModelType::E_FPP1Q;
    } else if (pt_str == "FPP_4Q") {
      pt = frydom::PropellerModelType::E_FPP4Q;
    } else if (pt_str == "CPP") {
      pt = frydom::PropellerModelType::E_CPP;
    } else {
      fs::path dir_path = get_json_file_root_directory();
      std::cerr << "In JSON file " << dir_path / m_json_file_path << std::endl;
      std::cerr << "ERROR at " << m_root << "/" << key << std::endl;
      std::cerr << "Unknown propeller type keyword " << pt_str <<
                ". Only FPP_1Q, FPP_4Q and CPP are accepted." << std::endl;
      exit(EXIT_FAILURE);
    }
    return pt;
  }

  template<>
  inline frydom::SCREW_DIRECTION JSONNode::get_val<frydom::SCREW_DIRECTION>(const std::string &key) const {
    auto sd_str = get_val<std::string>(key);
    frydom::SCREW_DIRECTION sd;
    if (sd_str == "LEFT_HANDED") {
      sd = frydom::SCREW_DIRECTION::LEFT_HANDED;
    } else if (sd_str == "RIGHT_HANDED") {
      sd = frydom::SCREW_DIRECTION::RIGHT_HANDED;
    } else {
      fs::path dir_path = get_json_file_root_directory();
      std::cerr << "In JSON file " << dir_path / m_json_file_path << std::endl;
      std::cerr << "ERROR at " << m_root << "/" << key << std::endl;
      std::cerr << "Unknown screw direction type keyword " << sd_str <<
                ". Only LEFT_HANDED and RIGHT_HANDED are accepted." << std::endl;
      exit(EXIT_FAILURE);
    }
    return sd;
  }

// TODO: voir avec Guillaume comment faire un operator overloading de ++ qui permette de skipper
// les commentaires (commencant par le caractere de commentaire, _ par defaut)


}  // end namespace frydom

#endif //FRYDOM_JSONNODE_H
