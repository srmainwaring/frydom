// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#ifndef FRYDOM_FRPATHMANAGER_H
#define FRYDOM_FRPATHMANAGER_H

#include <unordered_set>
#include <typeinfo>
#include <iostream>

#include "FrPathPolicies.h"
#include "FrEventLogger.h"


namespace frydom {

  // TODO : renommer en TreeManager ??
  class FrPathManager {

   public:

    template<class NodeType>
    static std::string GetPath(NodeType *node) {
      if (!node) return "";

      return GetPath(node->GetParent()) + GetNormalizedPathName(node);
    }

    template<class NodeType>
    static std::string GetPath(const NodeType &node) {
      return GetPath(&node);
    }


    template<class NodeType>
    void RegisterTreeNode(NodeType *node) {
      auto path = GetPath(node);

      if (!RegisterPath(path)) {
        event_logger::error("PathManager", "",
            "Object with name {} already exists in this context and cannot be defined twice.", node->GetName());
        throw std::runtime_error(
            "Object with name " + node->GetName() + " already exists in this context. Defined twice.");
      }

      node->SetTreePath(path);
    }

    template <class NodeType>
    void UnregisterTreeNode(NodeType *node) {
      std::string node_path = node->GetTreePath();
      if (!HasPath(node_path)) {
        event_logger::error("PathManager", "",
            "Attempting to unregister a node named {} that does not exist", node->GetName());
      }

      m_used_paths.erase(node_path);

    }


   private:

    bool RegisterPath(const std::string &path);


    bool HasPath(const std::string &path);

    /// Gives the normalized path of the node given a hard coded policy concerning the naming scheme.
    template<class NodeType>
    static std::string GetNormalizedPathName(NodeType *node) {
      return TypeToNormalizedPathPrefix(node) + node->GetName() + "/";
    }

   private:
    std::unordered_set<std::string> m_used_paths;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

} // end namespace frydom

#endif //FRYDOM_FRPATHMANAGER_H
