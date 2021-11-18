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


#ifndef FRYDOM_FRRADIATIONMODEL_H
#define FRYDOM_FRRADIATIONMODEL_H

#include <memory>
#include <unordered_map>

#include "frydom/utils/FrRecorder.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/core/common/FrTreeNode.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"

namespace frydom {

  // Forward declarations
  class FrHydroDB;

//  class FrBEMBody;

  class FrHydroMapper;

  class FrBody;

  class FrOffshoreSystem;

  namespace internal {
    class FrRadiationModelBase;
  }

  /**
   * \class FrRadiationModel
   * \brief Class for computing the radiation loads.
   */
  class FrRadiationModel : public FrTreeNode<FrOffshoreSystem>, public FrPhysicsItem {

   protected:

    std::shared_ptr<FrHydroDB> m_HDB;
    std::unordered_map<FrBEMBody *, GeneralizedForce> m_radiationForce;

   public:

    /// Constructor with specified hydrodynamic database
    /// \param HDB Hydrodynamic database
    explicit FrRadiationModel(const std::string &name,
                              FrOffshoreSystem *system,
                              std::shared_ptr<FrHydroDB> HDB);

    /// Return true if the radiation model is included in the static analysis
    bool IncludedInStaticAnalysis() const override { return false; }

    /// Return the hydrodynamic database linked with the radiation model
    /// \return Hydrodynamic database
    FrHydroDB *GetHydroDB() const { return m_HDB.get(); }

    /// Return the radiation force applied on a body
    /// \param BEMBody BEM body database
    /// \return Radiation force
    Force GetRadiationForce(FrBEMBody *BEMBody) const;

    /// Return the radiation force applied on a body
    /// \param body body (frydom object)
    /// \return Radiation force
    Force GetRadiationForce(FrBody *body) const;

    /// Return the radiation torque applied on a body
    /// \param BEMBody BEM body database
    /// \return Radiation torque
    Torque GetRadiationTorque(FrBEMBody *BEMBody) const;

    /// Return the radiation torque applied on a body
    /// \param body body (frydom object)
    /// \return Radiation torque
    Torque GetRadiationTorque(FrBody *body) const;

    /// Return the generalized force part relative to the added mass term
    /// \param body Body for which the motion is considered
    /// \return Part the the radiation force linked with the acceleration of the body
    GeneralizedForce GetRadiationInertiaPart(FrBody *body) const;

    /// Method to initialize the radiation model
    void Initialize() override;

    /// Return the mapper between body and BEM body database
    /// \return Mapper
    FrHydroMapper *GetMapper() const;

   private:

    /// Compute the internal states of the Radiation model
    /// \param time Current time of the simulation from beginning, in seconds
    void Compute(double time) override;

  };

}  // end namespace frydom


#endif //FRYDOM_FRRADIATIONMODEL_H
