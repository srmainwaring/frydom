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


#ifndef FRYDOM_FRTRIANGLEMESHSHAPE_H
#define FRYDOM_FRTRIANGLEMESHSHAPE_H

#include <memory>

#include "MathUtils/Vector3d.h"

#include "frydom/asset/FrAssetOwner.h"

namespace chrono {
  class ChAsset;

  class ChTriangleMeshShape;
}  // end namespace chrono


namespace frydom {

  // forward declaration
  class FrTriangleMeshShape;

  namespace internal {

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(std::shared_ptr<FrTriangleMeshShape> mesh);

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(FrTriangleMeshShape *mesh);

  } // end namespace frydom::internal


  class FrTriangleMeshConnected;

  class FrTriangleMeshShape {
   public:

    using Normal =          mathutils::Vector3d<double>;
    using Vertex =          mathutils::Vector3d<double>;
    using FaceVertexIndex = mathutils::Vector3d<int>;
    using FaceNormalIndex = mathutils::Vector3d<int>;

    explicit FrTriangleMeshShape(std::shared_ptr<FrTriangleMeshConnected> mesh);

    std::vector<Vertex> vertices() const;

    std::vector<Normal> normals() const;

    std::vector<FaceVertexIndex> faceVertexIndices() const;

    std::vector<FaceNormalIndex> faceNormalIndices() const;

   private:
    std::shared_ptr<chrono::ChTriangleMeshShape> m_mesh;

    friend std::shared_ptr<chrono::ChAsset> internal::GetChronoAsset(std::shared_ptr<FrTriangleMeshShape>);

    friend std::shared_ptr<chrono::ChAsset> internal::GetChronoAsset(FrTriangleMeshShape *);
  };

}  // end namespace frydom

#endif  // FRYDOM_FRTRIANGLEMESHSHAPE_H
