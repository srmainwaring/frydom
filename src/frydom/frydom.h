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

#ifndef FRYDOM_FRYDOM_H
#define FRYDOM_FRYDOM_H

#include <cstdlib>

#ifndef H5_NO_IRRLICHT
  #include <irrlicht.h>
#endif

// FRyDoM related headers
#include "core/FrCore.h"
#include "environment/FrEnvironmentInc.h"
#include "frydom/hydrodynamics/FrHydrodynamicsInc.h"
#include "cable/FrCableInc.h"
#include "frydom/io/FrIOInc.h"  // TODO: pour respecter le nommage, paser le repertoire io en io
#include "mesh/FrMeshInc.h"
#include "utils/FrUtilsInc.h"
#include "asset/FrAssetInc.h"
#include "collision/FrCollisionModel.h"
#include "propulsion/FrPropulsionInc.h"

//#include <H5Cpp.h>

#endif //FRYDOM_FRYDOM_H
