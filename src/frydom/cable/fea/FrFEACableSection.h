//
// Created by frongere on 05/05/2020.
//

#ifndef FRYDOM_FRFEACABLESECTION_H
#define FRYDOM_FRFEACABLESECTION_H

#include <chrono/fea/ChBeamSectionCosserat.h>


namespace frydom {

  namespace internal {
    class FrFEACableSection : public chrono::fea::ChBeamSectionCosserat {

     public:
      explicit FrFEACableSection(
          std::shared_ptr<chrono::fea::ChElasticityCosserat> melasticity  /// the elasticity model for this section, ex.ChElasticityCosseratSimple
      );

      FrFEACableSection(
          std::shared_ptr<chrono::fea::ChElasticityCosserat> melasticity,  /// the elasticity model for this section, ex.ChElasticityCosseratSimple
          std::shared_ptr<chrono::fea::ChPlasticityCosserat> mplasticity /// the plasticity model for this section, if any
      );

      FrFEACableSection(
          std::shared_ptr<chrono::fea::ChElasticityCosserat> melasticity,  /// the elasticity model for this section, ex.ChElasticityCosseratSimple
          std::shared_ptr<chrono::fea::ChPlasticityCosserat> mplasticity, /// the plasticity model for this section, if any
          std::shared_ptr<chrono::fea::ChDampingCosserat> mdamping /// the damping model for this section, if any
      );

      void SetNormalAddedMassCoeff(const double Cm);

      double GetCm() const;

      void SetNormalDragCoeff(const double Cd);

      double GetCd() const;

      void SetVIVAmpFactor(const double &G);

      double GetVIVAmpFactor() const;

     private:
      double m_Cm;
      double m_Cd;
      double m_VIV_amp_factor;

    };
  }  // end namspace frydom::internal

}  // end namespace frydom



#endif //FRYDOM_FRFEACABLESECTION_H
