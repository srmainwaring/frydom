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


#ifndef FRYDOM_FRFREESURFACEPHYSICITEM_H
#define FRYDOM_FRFREESURFACEPHYSICITEM_H

#include "frydom/asset/FrGridAsset.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"

namespace frydom {

    class FrFreeSurface_;
    class FrTriangleMeshConnected;

    /**
     * \class FrFreeSurfaceGridAsset
     * \brief Class for displaying the free surface grid.
     */
    class FrFreeSurfaceGridAsset : public FrGridAsset{

    private:
        FrFreeSurface_* m_freeSurface;    ///> Pointer to the free surface containing this asset

    public:
        /// Default constructor
        /// \param body body containing this asset (usually WorldBody)
        explicit FrFreeSurfaceGridAsset(FrFreeSurface_* freeSurface);

        /// FrFreeSurfaceGridAsset update method
        /// \param time time of the simulation
//        void Update(double time) override;

        void Update() override {};

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

    };
} // end namespace frydom

#endif //FRYDOM_FRFREESURFACEPHYSICITEM_H
