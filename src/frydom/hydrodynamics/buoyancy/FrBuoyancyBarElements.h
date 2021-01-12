//
// Created by camille on 27/03/2020.
//

#ifndef FRYDOM_FRBUOYANCYBARELEMENTS_H
#define FRYDOM_FRBUOYANCYBARELEMENTS_H

#include "frydom/core/force/FrForce.h"
#include "frydom/hydrodynamics/buoyancy/FrBarElement.h"

namespace frydom {

  class FrBuoyancyBarElements : public FrForce {

   public:

    FrBuoyancyBarElements(const std::string& name, FrBody* body, std::shared_ptr<FrBarElementBase> elements);

    void Initialize() override;

    bool IncludedInStaticAnalysis() const override { return true; }

   protected:

    void Compute(double time) override;

   private:

    std::shared_ptr<FrBarElementBase> m_elements;

  };

  std::shared_ptr<FrBuoyancyBarElements>
      make_buoyancy_bar_elements(const std::string& name, std::shared_ptr<FrBody> body,
                                 std::shared_ptr<FrBarElementBase> elements);

} // end namespace frydom

#endif //FRYDOM_FRBUOYANCYBARELEMENTS_H
