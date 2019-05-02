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

    private:

        FrOffshoreSystem* m_system;                     ///< reference to the offshore system
        std::shared_ptr<FrBody> m_body;                 ///< reference to the body related to this FrHydroMesh

        std::unique_ptr<mesh::FrMeshClipper> m_clipper; ///< mesh clipper service

        std::string m_meshfilename;                     ///< Input mesh file.
        mesh::FrMesh m_initMesh;                        ///< Input mesh file (as defined and read from the input file)
        mesh::FrMesh m_clippedMesh;                     ///< Clipped mesh (its frame follows the body frame in its motions)

        FrFrame m_meshOffset;                           ///< Offset frame between mesh and body frame (defined initially)

        bool m_WNL_or_NL;                               ///< Boolean to know if the mesh is clipped by a wave (True) or a plane (False).

    public:

        /// Constructor.
        FrHydroMesh(FrOffshoreSystem* system, std::string meshfile, std::shared_ptr<FrBody> body, bool WNL_or_NL);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "HydroMesh"; }

        /// Initialize the nonlinear hydrostatic force model.
        void Initialize() override;

        /// Initialize the log
        void InitializeLog() override;

        /// This function sets the offset of the mesh frame in the body frame.
        void SetMeshOffset(FrFrame meshOffset);

        FrFrame GetMeshOffset() const;

        /// Get a reference to the clipped mesh (its frame is adjusted to the body's motions)
        /// \return reference to the clipped mesh
        mesh::FrMesh& GetClippedMesh();

        /// Get a reference to the initial mesh (as defined and read from the input file)
        /// \return reference to the initial mesh
        mesh::FrMesh& GetInitialMesh();


    private:

        /// Update the frame of the clipped mesh according to the body's motions
        void UpdateMeshFrame();

        /// Get the position in the body reference frame of a point which position is specified in the mesh reference frame
        /// \param point position of a point in the mesh reference frame
        /// \return point position in the body reference frame
        mesh::FrMesh::Point GetMeshPointPositionInBody(mesh::FrMesh::Point point) const;

        /// Update nonlinear hydrostatic force.
        /// \param time Current time of the simulation from beginning.
        void Compute(double time) override;

    };

    /// This function creates a hydrodynamic mesh for using in the computation of the nonlinear hydrostatic and/or Froude-Krylov loads.
    std::shared_ptr<FrHydroMesh> make_hydro_mesh_nonlinear(const std::shared_ptr<FrBody>& body, const std::string& meshfile);

    /// This function creates a hydrodynamic mesh for using in the computation of the weakly nonlinear hydrostatic and/or Froude-Krylov loads.
    std::shared_ptr<FrHydroMesh> make_hydro_mesh_weakly_nonlinear(const std::shared_ptr<FrBody>& body, const std::string& meshfile);

    }  // end namespace frydom

#endif //FRYDOM_FRHYDROMESH_H