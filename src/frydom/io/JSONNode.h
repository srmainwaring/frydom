#ifndef FRYDOM_JSONNODE_H
#define FRYDOM_JSONNODE_H

#include <fstream>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include <filesystem>


namespace fs = std::filesystem;
using json = nlohmann::json;

namespace frydom {

  namespace internal {

    inline bool startswith(const std::string &str, const std::string &pattern) {
      return str.rfind(pattern, 0) == 0;
    }

    inline bool is_comment(const std::string &str) {
      return startswith(str, "_");
    }

    inline bool exists(const json &node, const std::string &key) {
      return node.find(key) != node.end();
    }

    void walk_over_node(const json &node, const fs::path &current_root_dir, json &working_node);

  }  // end namespace esperado::internal

  /// Iterator class to navigate among JSON Nodes in a JSON hierarchy
  template<class JSONNode, class InnerIterType>
  class JSONNodeIterator {
   public:
//    using iterator_category = std::forward_iterator_tag;
//    using difference_type = std::ptrdiff_t;
    using value_type = JSONNode;
    using pointer = value_type *;
//    using reference = value_type &;

   public:
    JSONNodeIterator(JSONNode *ptr, json::iterator json_iter) : m_ptr(ptr), m_json_iter(json_iter) {}

    JSONNodeIterator(JSONNode *ptr, json::const_iterator json_iter) : m_ptr(ptr), m_json_iter(json_iter) {}

    std::string key() {
      return m_json_iter.key();
    }

    value_type operator*() const {
      return JSONNode(*m_json_iter, m_ptr->m_node_name / m_json_iter.key());
    }

    JSONNodeIterator &operator++() {
      ++m_json_iter;

      return *this;
    }

    JSONNodeIterator operator++(int) {
      JSONNodeIterator tmp = *this;
      ++(*this);
      return tmp;
    }

    friend bool operator==(const JSONNodeIterator &a, const JSONNodeIterator &b) {
      return a.m_json_iter == b.m_json_iter;
    }

    friend bool operator!=(const JSONNodeIterator &a, const JSONNodeIterator &b) {
      return a.m_json_iter != b.m_json_iter;
    }

   private:
    pointer m_ptr;
    InnerIterType m_json_iter;
  };

  /// Class that represents a JSON Node in a JSON hierarchy
  ///
  /// It works like a pointer into the JSON hierarchy but has only methods to move towards JSON hierarchy leafs
  class JSONNode {
   public:

    /// CTOR from file path on disk
    explicit JSONNode(const std::string &json_filename) : m_node_name("/") {
      if (!fs::exists(json_filename)) {
        std::cerr << "JSON file " << json_filename << " not found" << std::endl;
        exit(EXIT_FAILURE);
      }

      fs::path root_dir = fs::path(json_filename).remove_filename();

      std::ifstream ifs(json_filename);
      json raw_node = json::parse(ifs);

      m_json_node = raw_node;

      internal::walk_over_node(raw_node, root_dir, m_json_node);

    }

    ///
    std::string dump(int indent = 2) const {
      return m_json_node.dump(indent);
    }

    bool exists(const std::string &key) const {
      return internal::exists(m_json_node, key);
    }

    JSONNode operator[](const std::string &key) const {

      auto pkey = m_node_name / key;

      if (!internal::exists(m_json_node, key)) {
        std::cerr << "Key " << pkey << " does not exists" << std::endl;
        exit(EXIT_FAILURE);
      }
      auto node = m_json_node[key];
      return JSONNode(node, pkey);
    }

    template<typename T>
    T get() const {
      if (!m_json_node.is_primitive()) {
        std::cerr << "JSON Node " << full_name() << " is not a primitive" << std::endl;
        exit(EXIT_FAILURE);
      }
      return m_json_node.get<T>();
    }

    template<typename T>
    T get(const std::string &key) const {
      if (!internal::exists(m_json_node, key)) {
        std::cerr << "Key " << m_node_name / key << " does not exist" << std::endl;
        exit(EXIT_FAILURE);
      }

      auto node = m_json_node[key];

      if (!node.is_primitive() && !node.is_array()) {
        std::cerr << "JSON Node " << full_name() << " is not a primitive nor an array" << std::endl;
        exit(EXIT_FAILURE);
      }

      return node.get<T>();
    }

    template<typename T>
    T get(const std::string &key, const T &default_value) const {
      return internal::exists(m_json_node, key) ? get<T>(key) : default_value;
    }

    std::string full_name() const {
      return m_node_name;
    }

    std::string name() const {
      return m_node_name.filename();
    }

    JSONNodeIterator<JSONNode, json::iterator> begin() {
      return {this, m_json_node.begin()};
    }

    JSONNodeIterator<JSONNode, json::iterator> end() {
      return {this, m_json_node.end()};
    }

    JSONNodeIterator<const JSONNode, json::const_iterator> begin() const {
      return {this, m_json_node.cbegin()};
    }

    JSONNodeIterator<const JSONNode, json::const_iterator> end() const {
      return {this, m_json_node.cend()};
    }

   private:

    JSONNode(const json &node, const fs::path &node_name) :
        m_json_node(node),
        m_node_name(node_name) {}

   private:
    json m_json_node;
    fs::path m_node_name;

    friend class JSONNodeIterator<JSONNode, json::iterator>;

    friend class JSONNodeIterator<const JSONNode, json::const_iterator>;

  };

}  // end namespace frydom

#endif //FRYDOM_JSONNODE_H
