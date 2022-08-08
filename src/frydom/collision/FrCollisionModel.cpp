//
// Created by lletourn on 21/05/19.
//

#include "FrCollisionModel.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"
#include "frydom/core/contact/FrMaterialSurface.h"

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

  bool FrCollisionModel::AddSphere(FrMaterialSurface *mat, double radius, const Position &pos) {

    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddSphere(chrono_material, radius, pos);

  }

  bool
  FrCollisionModel::AddEllipsoid(FrMaterialSurface *mat,
                                 double rx,
                                 double ry,
                                 double rz,
                                 const Position &pos,
                                 const FrRotation &rot) {

    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddEllipsoid(chrono_material, rx, ry, rz, pos, chRot);

  }

  bool FrCollisionModel::AddBox(FrMaterialSurface *mat,
                                double hx,
                                double hy,
                                double hz,
                                const Position &pos,
                                const FrRotation &rot) {

    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddBox(chrono_material, hx, hy, hz, pos, chRot);

  }

  bool
  FrCollisionModel::AddCylinder(FrMaterialSurface *mat,
                                double rx,
                                double rz,
                                double hy,
                                const Position &pos,
                                const FrRotation &rot) {

    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddCylinder(chrono_material, rx, rz, hy, pos, chRot);

  }

  bool FrCollisionModel::AddConvexHull(FrMaterialSurface *mat,
                                       const std::vector<Position> &pointlist,
                                       const Position &pos,
                                       const FrRotation &rot) {

    std::vector<chrono::ChVector<double>> chVect;
    for (const auto &point: pointlist) {
      chVect.emplace_back(point);
    }

    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddConvexHull(chrono_material, chVect, pos, chRot);

  }

  bool FrCollisionModel::AddTriangleMesh(FrMaterialSurface *mat,
                                         std::shared_ptr<FrTriangleMeshConnected> trimesh,
                                         bool is_static,
                                         bool is_convex,
                                         const Position &pos,
                                         const FrRotation &rot,
                                         double sphereswept_thickness) {

    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddTriangleMesh(chrono_material, trimesh, is_static, is_convex, pos, chRot,
                                                   sphereswept_thickness);

  }

  bool
  FrCollisionModel::AddTriangleMeshConcave(FrMaterialSurface *mat,
                                           std::shared_ptr<FrTriangleMeshConnected> trimesh,
                                           const Position &pos,
                                           const FrRotation &rot) {

    auto chRot = internal::Fr2ChQuaternion(rot.GetQuaternion());
    auto chrono_material = internal::GetChronoMaterial(mat);

    return m_chronoCollisionModel->AddTriangleMeshConcave(chrono_material, trimesh, pos, chRot);

  }

  void FrCollisionModel::Initialize() {
    m_chronoCollisionModel->BuildModel();
  }

  void FrCollisionModel::ClearModel() {
    m_chronoCollisionModel->ClearModel();
  }

  bool FrCollisionModel::AddTriangleMesh(FrMaterialSurface *mat,
                                         const std::string &obj_filename,
                                         const Position &pos,
                                         const FrRotation &rot,
                                         bool is_static,
                                         bool is_convex,
                                         double sphereswept_thickness) {

    auto mesh = std::make_shared<FrTriangleMeshConnected>();
    mesh->LoadWavefrontMesh(obj_filename);

    return AddTriangleMesh(mat, mesh, is_static, is_convex, pos, rot, sphereswept_thickness);

  }

  void FrCollisionModel::SetDefaultSuggestedEnvelope(double env) {
    m_chronoCollisionModel->SetDefaultSuggestedEnvelope(env);
    m_chronoCollisionModel->SetEnvelope(env);
  }

} // end namespace frydom