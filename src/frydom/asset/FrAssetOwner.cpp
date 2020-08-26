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

#include <chrono/assets/ChColorAsset.h>
#include <chrono/physics/ChLinkMotorRotation.h>


#include "frydom/core/body/FrBody.h"
#include "frydom/mesh/FrHydroMesh.h"
#include "frydom/core/link/links_lib/actuators/FrAngularActuator.h"
#include "frydom/core/link/links_lib/actuators/FrLinearActuator.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"
#include "shape/FrBoxShape.h"
#include "shape/FrCylinderShape.h"
#include "shape/FrSphereShape.h"
#include "shape/FrTriangleMeshShape.h"
#include "frydom/cable/catenary/FrCatenaryLineBase.h"
#include "frydom/cable/catenary_ee444075/FrCatenaryLine_ee444.h"
#include "frydom/core/common/FrPhysicsItem.h"


namespace frydom {


  void FrAssetOwner::AddBoxShape(double xSize,
                                 double ySize,
                                 double zSize,
                                 const Position &relative_position,
                                 FRAME_CONVENTION fc) {
    auto shape = std::make_shared<FrBoxShape>(xSize, ySize, zSize, relative_position, fc);
    m_boxShapes.push_back(shape);
    internal::GetChronoPhysicsItemFromAsset(this)->AddAsset(internal::GetChronoAsset(shape));
  }

  void
  FrAssetOwner::AddCylinderShape(double radius,
                                 double height,
                                 const Position &relative_position,
                                 FRAME_CONVENTION fc) {
    auto shape = std::make_shared<FrCylinderShape>(radius, height, relative_position, fc);
    m_cylinderShapes.push_back(shape);
    internal::GetChronoPhysicsItemFromAsset(this)->AddAsset(internal::GetChronoAsset(shape));
  }

  void FrAssetOwner::AddSphereShape(double radius,
                                    const Position &relative_position,
                                    FRAME_CONVENTION fc) {
    auto shape = std::make_shared<FrSphereShape>(radius, relative_position, fc);
    m_sphereShapes.push_back(shape);
    internal::GetChronoPhysicsItemFromAsset(this)->AddAsset(internal::GetChronoAsset(shape));
  }

  void FrAssetOwner::AddMeshAsset(std::string obj_filename) {
    auto mesh = std::make_shared<FrTriangleMeshConnected>();
    mesh->LoadWavefrontMesh(obj_filename);
    AddMeshAsset(mesh);
  }

  void FrAssetOwner::AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh) {
    auto shape = std::make_shared<FrTriangleMeshShape>(mesh);
    m_meshShapes.push_back(shape);
    internal::GetChronoPhysicsItemFromAsset(this)->AddAsset(internal::GetChronoAsset(shape));
  }

  void FrAssetOwner::AddAsset(std::shared_ptr<FrAsset> asset) {
    m_assets.push_back(asset);
    internal::GetChronoPhysicsItemFromAsset(this)->AddAsset(internal::GetChronoAsset(asset));
  }

  void FrAssetOwner::RemoveAssets() {
    m_assets.clear();
    internal::GetChronoPhysicsItemFromAsset(this)->GetAssets().clear();
  }

//  void FrAssetOwner::RemoveAsset(std::shared_ptr<FrAsset> asset) {
//
//    assert(std::find<std::vector<std::shared_ptr<FrAsset>>::iterator>(m_assets.begin(), m_assets.end(), asset) !=
//           m_assets.end());
//
//    m_assets.erase(std::find<std::vector<std::shared_ptr<FrAsset>>::iterator>(m_assets.begin(), m_assets.end(), asset));
//
//    // Remove also the asset into Chrono
//    auto assets = internal::GetChronoPhysicsItemFromAsset(this)->GetAssets();
//
//    assert(std::find<std::vector<std::shared_ptr<chrono::ChAsset>>::iterator>(assets.begin(), assets.end(),
//                                                                              asset) != assets.end());
//
//    auto it0 = std::find(assets.begin(), assets.end(), asset);
//    assets.erase(it0);
//
//  }

  FrAssetOwner::BoxShapeConstContainer FrAssetOwner::GetBoxShapes() const {
    FrAssetOwner::BoxShapeConstContainer result;
    for (const auto &shape : m_boxShapes) {
      result.push_back(std::const_pointer_cast<const FrBoxShape>(shape));
    }
    return result;
  }

  FrAssetOwner::CylinderShapeConstContainer FrAssetOwner::GetCylinderShapes() const {
    FrAssetOwner::CylinderShapeConstContainer result;
    for (const auto &shape : m_cylinderShapes) {
      result.push_back(std::const_pointer_cast<const FrCylinderShape>(shape));
    }
    return result;
  }

  FrAssetOwner::SphereShapeConstContainer FrAssetOwner::GetSphereShapes() const {
    FrAssetOwner::SphereShapeConstContainer result;
    for (const auto &shape : m_sphereShapes) {
      result.push_back(std::const_pointer_cast<const FrSphereShape>(shape));
    }
    return result;
  }

  FrAssetOwner::TriangleMeshShapeConstContainer FrAssetOwner::GetMeshAssets() const {
    FrAssetOwner::TriangleMeshShapeConstContainer result;
    for (const auto &shape : m_meshShapes) {
      result.push_back(std::const_pointer_cast<const FrTriangleMeshShape>(shape));
    }
    return result;
  }

  void FrAssetOwner::SetColor(NAMED_COLOR colorName) {
    SetColor(FrColor(colorName));
  }

  void FrAssetOwner::SetColor(const FrColor &color) {
    auto colorAsset = std::make_shared<chrono::ChColorAsset>(
        chrono::ChColor(color.R, color.G, color.B));
    internal::GetChronoPhysicsItemFromAsset(this)->AddAsset(colorAsset);
  }

  void FrAssetOwner::UpdateAsset() {
    // StepFinalize of assets
    auto assetIter = asset_begin();
    for (; assetIter != asset_end(); assetIter++) {
      (*assetIter)->StepFinalize();
    }
  }

  // Asset linear iterators
  FrAssetOwner::AssetIter FrAssetOwner::asset_begin() {
    return m_assets.begin();
  }

  FrAssetOwner::ConstAssetIter FrAssetOwner::asset_begin() const {
    return m_assets.cbegin();
  }

  FrAssetOwner::AssetIter FrAssetOwner::asset_end() {
    return m_assets.end();
  }

  FrAssetOwner::ConstAssetIter FrAssetOwner::asset_end() const {
    return m_assets.cend();
  }


  namespace internal {

    /**
     * TODO: en realite, il y a un gros pb d'architecture !!
     * Il conviendrait que tous les asset owners soient des physics item
     * Se posera alors le pb de CatenaryAssetOwner...
     */

    std::shared_ptr<chrono::ChPhysicsItem> GetChronoPhysicsItemFromAsset(FrAssetOwner *assetOwner) {

      if (auto body = dynamic_cast<FrBody *>(assetOwner)) {
        return internal::GetChronoBody(body);
      }

      // Links

      if (auto angular_actuator = dynamic_cast<FrAngularActuator *>(assetOwner)) {
        return internal::GetChronoActuator(angular_actuator);
      }

      if (auto linear_actuator = dynamic_cast<FrLinearActuator *>(assetOwner)) {
        return internal::GetChronoActuator(linear_actuator);
      }

      if (auto catenary_line_base = dynamic_cast<FrCatenaryLineBase *>(assetOwner)) {
        // FIXME: voir si on a besoin de differencier catenary line & hydro_mesh
        return internal::GetChronoPhysicsItem(catenary_line_base);
      }

      if (auto catenary_line_base = dynamic_cast<FrCatenaryLine_ee444 *>(assetOwner)) {
        // FIXME: voir si on a besoin de differencier catenary line & hydro_mesh
        return internal::GetChronoPhysicsItem(catenary_line_base);
      }

//      if (auto catenary_line = dynamic_cast<FrCatenaryLineBase *>(assetOwner)) {
//        return internal::GetChronoPhysicsItem(catenary_line);
//      }
//
//      if (auto hydro_mesh = dynamic_cast<FrHydroMesh *>(assetOwner)) {
//        return internal::GetChronoPhysicsItem(hydro_mesh);
//      }


    }

  }  // end namespace frydom::internal

}// end namespace frydom
