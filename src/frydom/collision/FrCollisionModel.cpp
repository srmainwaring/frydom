//
// Created by lletourn on 21/05/19.
//

#include "FrCollisionModel.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {

  namespace internal {
    FrCollisionModelBase::FrCollisionModelBase(FrCollisionModel *collisionModel) :
        m_frydomCollisionModel(collisionModel) {

    }

  } // end namespace frydom::internal


  FrCollisionModel::FrCollisionModel() {
    m_chronoCollisionModel = std::make_shared<internal::FrCollisionModelBase>(this);
    m_chronoCollisionModel->ClearModel();
  }

  bool FrCollisionModel::AddSphere(FrContactParams* mat, double radius, const Position &pos) {

    auto chPos = internal::Vector3dToChVector(pos);
    auto chrono_material = internal::GetChronoMaterial(mat);
    return m_chronoCollisionModel->AddSphere(chrono_material, radius, chPos);

  }

  bool
  FrCollisionModel::AddEllipsoid(FrContactParams* mat, double rx, double ry, double rz, const Position &pos, const FrRotation &rot) {

    auto chPos = internal::Vector3dToChVector(pos);
    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddEllipsoid(chrono_material, rx, ry, rz, chPos, chRot);

  }

  bool FrCollisionModel::AddBox(FrContactParams* mat, double hx, double hy, double hz, const Position &pos, const FrRotation &rot) {

    auto chPos = internal::Vector3dToChVector(pos);
    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddBox(chrono_material, hx, hy, hz, chPos, chRot);

  }

  bool
  FrCollisionModel::AddCylinder(FrContactParams* mat, double rx, double rz, double hy, const Position &pos, const FrRotation &rot) {

    auto chPos = internal::Vector3dToChVector(pos);
    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddCylinder(chrono_material, rx, rz, hy, chPos, chRot);

  }

  bool FrCollisionModel::AddConvexHull(FrContactParams* mat, const std::vector<Position> &pointlist, const Position &pos,
                                       const FrRotation &rot) {

    std::vector<chrono::ChVector<double>> chVect;
    for (const auto &point : pointlist) {
      chVect.push_back(internal::Vector3dToChVector(point));
    }

    auto chPos = internal::Vector3dToChVector(pos);
    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddConvexHull(chrono_material, chVect, chPos, chRot);

  }

  bool FrCollisionModel::AddTriangleMesh(FrContactParams* mat, std::shared_ptr<FrTriangleMeshConnected> trimesh, bool is_static,
                                         bool is_convex, const Position &pos, const FrRotation &rot,
                                         double sphereswept_thickness) {

    auto chPos = internal::Vector3dToChVector(pos);
    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddTriangleMesh(chrono_material, trimesh, is_static, is_convex, chPos, chRot, sphereswept_thickness);

  }

  bool
  FrCollisionModel::AddTriangleMeshConcave(FrContactParams* mat, std::shared_ptr<FrTriangleMeshConnected> trimesh, const Position &pos,
                                           const FrRotation &rot) {

    auto chPos = internal::Vector3dToChVector(pos);
    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddTriangleMeshConcave(chrono_material, trimesh, chPos, chRot);

  }

  void FrCollisionModel::Initialize() {
    m_chronoCollisionModel->BuildModel();
  }

  void FrCollisionModel::ClearModel() {
    m_chronoCollisionModel->ClearModel();
  }

  void FrCollisionModel::BuildModel() {
    m_chronoCollisionModel->BuildModel();
  }

  bool FrCollisionModel::AddTriangleMesh(FrContactParams* mat, const std::string &obj_filename, const Position &pos, const FrRotation &rot,
                                         bool is_static, bool is_convex, double sphereswept_thickness) {

    auto mesh = std::make_shared<FrTriangleMeshConnected>();
    mesh->LoadWavefrontMesh(obj_filename);

    return AddTriangleMesh(mat, mesh, is_static, is_convex, pos, rot, sphereswept_thickness);

  }

  void FrCollisionModel::SetDefaultSuggestedEnvelope(double env) {
    m_chronoCollisionModel->SetDefaultSuggestedEnvelope(env);
    m_chronoCollisionModel->SetEnvelope(env);
  }

} // end namespace frydom