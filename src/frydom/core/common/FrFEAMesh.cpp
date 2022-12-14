//
// Created by lletourn on 06/03/19.
//

#include "FrFEAMesh.h"

namespace frydom {

  namespace internal {

    FrFEAMeshBase::FrFEAMeshBase(frydom::FrFEAMesh *frydom_mesh) :
        chrono::fea::ChMesh(),
        m_frydom_mesh(frydom_mesh) {}

    std::shared_ptr<FrFEAMeshBase> GetChronoFEAMesh(std::shared_ptr<FrFEAMesh> mesh) {
      return mesh->m_chrono_mesh;
    }

  }  // end namespace frydom::internal


  FrFEAMesh::FrFEAMesh(const std::string &name,
                       const std::string &type_name,
                       FrOffshoreSystem *system,
                       std::shared_ptr<internal::FrFEAMeshBase> chrono_mesh) :
      m_chrono_mesh(chrono_mesh),
      FrLoggable<FrOffshoreSystem>(name, type_name, system) {}

  std::shared_ptr<internal::FrFEAMeshBase> FrFEAMesh::GetFEAMeshBase() {
    return m_chrono_mesh;
  }

}  // end namespace frydom
