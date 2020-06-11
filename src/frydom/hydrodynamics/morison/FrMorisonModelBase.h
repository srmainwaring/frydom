//
// Created by camille on 16/04/2020.
//

#ifndef FRYDOM_FRMORISONMODELBASE_H
#define FRYDOM_FRMORISONMODELBASE_H

#include "frydom/core/common/FrPhysicsItem.h"

namespace frydom {

  // forward declaration
  class FrMorisonCompositeElement;

  namespace internal {

    // forward declaration
    class FrVariablesAddedMass;

    class FrMorisonModelBase : public FrPhysicsItemBase {

    private:

     FrMorisonCompositeElement* m_frydomMorisonCompositeElement;
     std::shared_ptr<FrVariablesAddedMass> m_variables;

    public:

     explicit FrMorisonModelBase(FrMorisonCompositeElement* morisonCompositeElement);

     void SetupInitial() override;

     void Update(bool update_assets) override;

     void Update(double time, bool update_assets) override;

     // Descriptor

     void IntLoadResidual_Mv(const unsigned int off,
                             chrono::ChVectorDynamic<> &R,
                             const chrono::ChVectorDynamic<> &w,
                             const double c) override;

     void IntToDescriptor(const unsigned int off_v, const chrono::ChStateDelta &v,
                          const chrono::ChVectorDynamic<> &R, const unsigned int off_L,
                          const chrono::ChVectorDynamic<> &L, const chrono::ChVectorDynamic<> &Qc) override;

     void IntFromDescriptor(const unsigned int off_v, chrono::ChStateDelta &v,
                            const unsigned int off_L, chrono::ChVectorDynamic<> &L) override;

     unsigned int GetBodyOffset() const;

   };


  } // end namespace frydom::internal

} // end namespace frydom

#endif //FRYDOM_FRMORISONMODELBASE_H
