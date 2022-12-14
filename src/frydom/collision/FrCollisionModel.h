//
// Created by lletourn on 21/05/19.
//

#include "chrono/collision/ChCollisionModelBullet.h"

#include "frydom/core/body/FrBody.h"

#ifndef FRYDOM_FRCOLLISIONMODEL_H
#define FRYDOM_FRCOLLISIONMODEL_H

namespace frydom {

  // Forward declarations
  class Position;

  class FrRotation;

  class FrTriangleMeshConnected;

  class FrCollisionModel;

  class FrMaterialSurface;

  namespace internal {


    class FrCollisionModelBase : public chrono::collision::ChCollisionModelBullet {

     private:

      FrCollisionModel *m_frydomCollisionModel;

     public:

      explicit FrCollisionModelBase(FrCollisionModel *collisionModel);

    };


  } // end namespace frydom::internal



  class FrCollisionModel {

   protected:

    std::shared_ptr<internal::FrCollisionModelBase> m_chronoCollisionModel;

   public:

    explicit FrCollisionModel();

    void ClearModel();

    //
    // GEOMETRY DESCRIPTION
    //
    //  The following functions must be called inbetween
    //  the ClearModel() BuildModel() pair.

    /// Add a sphere shape to this model, for collision purposes
    bool AddSphere(FrMaterialSurface* mat, double radius, const Position &pos);

    /// Add an ellipsoid shape to this model, for collision purposes
    bool AddEllipsoid(FrMaterialSurface* mat, double rx, double ry, double rz, const Position &pos, const FrRotation &rot);

    /// Add a box shape to this model, for collision purposes
    bool AddBox(FrMaterialSurface* mat, double hx, double hy, double hz, const Position &pos, const FrRotation &rot);

    /// Add a cylinder to this model (default axis on Y direction), for collision purposes
    bool AddCylinder(FrMaterialSurface* mat, double rx, double rz, double hy, const Position &pos, const FrRotation &rot);

    /// Add a convex hull to this model. A convex hull is simply a point cloud that describe
    /// a convex polytope. Connectivity between the vertexes, as faces/edges in triangle meshes is not necessary.
    /// Points are passed as a list, that is instantly copied into the model.
    bool AddConvexHull(FrMaterialSurface* mat, const std::vector<Position> &pointlist, const Position &pos, const FrRotation &rot);

    /// Add a triangle mesh to this model, passing a triangle mesh.
    /// Note: if possible, for better performance, avoid triangle meshes and prefer simplified
    /// representations as compounds of primitive convex shapes (boxes, sphers, etc).
    bool AddTriangleMesh(                           //
        FrMaterialSurface* mat,
        const std::string &obj_filename,                    ///< the triangle mesh
        const Position &pos,                                ///< displacement respect to COG
        const FrRotation &rot,                              ///< the rotation of the mesh
        bool is_static = false,                             ///< true if model doesn't move. May improve performance.
        bool is_convex = true,                              ///< if true, a convex hull is used. May improve robustness.
        double sphereswept_thickness = 0.0                  ///< outward sphere-swept layer (when supported)
    );

    void SetDefaultSuggestedEnvelope(double env);

    void Initialize();

   protected:

    /// Add a triangle mesh to this model, passing a triangle mesh.
    /// Note: if possible, for better performance, avoid triangle meshes and prefer simplified
    /// representations as compounds of primitive convex shapes (boxes, sphers, etc).
    bool AddTriangleMesh(                           //
        FrMaterialSurface* mat,
        std::shared_ptr<FrTriangleMeshConnected> trimesh,  ///< the triangle mesh
        bool is_static,                                     ///< true if model doesn't move. May improve performance.
        bool is_convex,                                     ///< if true, a convex hull is used. May improve robustness.
        const Position &pos,                                ///< displacement respect to COG
        const FrRotation &rot,                              ///< the rotation of the mesh
        double sphereswept_thickness = 0.0                  ///< outward sphere-swept layer (when supported)
    );

    /// CUSTOM for this class only: add a concave triangle mesh that will be managed
    /// by GImpact mesh-mesh algorithm. Note that, despite this can work with
    /// arbitrary meshes, there could be issues of robustness and precision, so
    /// when possible, prefer simplified representations as compounds of convex
    /// shapes of boxes/spheres/etc.. type.
    bool AddTriangleMeshConcave(FrMaterialSurface* mat, std::shared_ptr<FrTriangleMeshConnected> trimesh,
                                const Position &pos,
                                const FrRotation &rot);

//            /// CUSTOM for this class only: add a concave triangle mesh that will be decomposed
//            /// into a compound of convex shapes. Decomposition could be more efficient than
//            /// AddTriangleMeshConcave(), but preprocessing decomposition might take a while, and
//            /// decomposition result is often approximate. Therefore, despite this can work with
//            /// arbitrary meshes, there could be issues of robustness and precision, so
//            /// when possible, prefer simplified representations as compounds of convex
//            /// shapes of boxes/spheres/etc.. type.
//            virtual bool AddTriangleMeshConcaveDecomposed(std::shared_ptr<ChConvexDecomposition> mydecomposition,
//                                                          const Position& pos,
//                                                          const FrRotation& rot);

    friend void FrBody::SetCollisionModel(std::shared_ptr<FrCollisionModel>);
  };

} // end namespace frydom

#endif //FRYDOM_FRCOLLISIONMODEL_H
