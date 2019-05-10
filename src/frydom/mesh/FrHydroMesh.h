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


#ifndef FRYDOM_FRHYDROMESH_H
#define FRYDOM_FRHYDROMESH_H

#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/mesh/FrMesh.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

#include "frydom/core/common/FrFrame.h"

namespace frydom {

    class FrOffshoreSystem;
    class FrBody;

    /**
     * \class FrHydroMesh
     * \brief Class for managing the meshes used for computing the nonlinear hydrostatic and Froude-Krylov loads.
     */
    class FrHydroMesh : public FrPrePhysicsItem {

    public:

        enum class ClippingSupport {    ///< Support for the clipping procedure : can be a horizontal plan
            PLANSURFACE,                ///< horizontal plan, defined by its elevation corresponding to the tidal height
            WAVESURFACE                 ///< wave surface, defined by the free surface position at a given (x,y) position
        };

        /// Constructor.
        FrHydroMesh(FrOffshoreSystem* system, const std::shared_ptr<FrBody>& body, FrHydroMesh::ClippingSupport support);

        FrHydroMesh(FrOffshoreSystem* system, const std::shared_ptr<FrBody>& body, const std::string& meshFile, FrFrame meshOffsset, FrHydroMesh::ClippingSupport support);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "HydroMesh"; }

        /// Initialize the nonlinear hydrostatic force model.
        void Initialize() override;

        /// Import a mesh and apply the mesh frame offset transformation, so that the mesh can be expressed in the body reference frame
        /// \param meshFile mesh file name
        /// \param meshOffset mesh frame offset, relatively to the body reference frame
        /// \return imported mesh, in body reference frame
        mesh::FrMesh& ImportMesh(const std::string& meshFile, FrFrame meshOffset);

        /// Get a reference to the clipped mesh (its frame is adjusted to the body's motions)
        /// \return reference to the clipped mesh
        mesh::FrMesh& GetClippedMesh();

        /// Get a reference to the initial mesh (as defined and read from the input file)
        /// \return reference to the initial mesh
        mesh::FrMesh& GetInitialMesh();


    private:

        /// Update nonlinear hydrostatic force.
        /// \param time Current time of the simulation from beginning.
        void Compute(double time) override;

    private:

        FrOffshoreSystem* m_system;                     ///< reference to the offshore system
        std::shared_ptr<FrBody> m_body;                 ///< reference to the body related to this FrHydroMesh

        std::unique_ptr<mesh::FrMeshClipper> m_clipper; ///< mesh clipper service

        mesh::FrMesh m_initMesh;                        ///< Input mesh file (as defined and read from the input file)
        mesh::FrMesh m_clippedMesh;                     ///< Clipped mesh (its frame follows the body frame in its motions)

        FrFrame m_meshOffset;                           ///< Offset frame between mesh and body frame (defined initially)

        ClippingSupport m_clippingSupport;              ///< Support for the clipping procedure

    };

    /// This function creates a hydrodynamic mesh for using in the computation of the nonlinear hydrostatic and/or Froude-Krylov loads.
    std::shared_ptr<FrHydroMesh> make_hydro_mesh(const std::shared_ptr<FrBody>& body, FrHydroMesh::ClippingSupport support);

    std::shared_ptr<FrHydroMesh> make_hydro_mesh(const std::shared_ptr<FrBody>& body, const std::string& meshFile,
                                                 FrFrame meshOffset, FrHydroMesh::ClippingSupport support);


//    /// This function creates a hydrodynamic mesh for using in the computation of the nonlinear hydrostatic and/or Froude-Krylov loads.
//    std::shared_ptr<FrHydroMesh> make_hydro_mesh_nonlinear(const std::shared_ptr<FrBody>& body, const std::string& meshfile);
//
//    /// This function creates a hydrodynamic mesh for using in the computation of the weakly nonlinear hydrostatic and/or Froude-Krylov loads.
//    std::shared_ptr<FrHydroMesh> make_hydro_mesh_weakly_nonlinear(const std::shared_ptr<FrBody>& body, const std::string& meshfile);

    }  // end namespace frydom

#endif //FRYDOM_FRHYDROMESH_H