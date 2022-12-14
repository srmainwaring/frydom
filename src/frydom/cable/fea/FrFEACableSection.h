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
          std::shared_ptr<chrono::fea::ChInertiaCosserat> minertia,         /// the inertia of the section of the cable element
          std::shared_ptr<chrono::fea::ChElasticityCosserat> melasticity   /// the elasticity model for this section, ex.ChElasticityCosseratSimple
      );

      FrFEACableSection(
          std::shared_ptr<chrono::fea::ChInertiaCosserat> minertia,       /// the inertia of this section
          std::shared_ptr<chrono::fea::ChElasticityCosserat> melasticity,  /// the elasticity model for this section, ex.ChElasticityCosseratSimple
          std::shared_ptr<chrono::fea::ChPlasticityCosserat> mplasticity  /// the plasticity model for this section, if any
      );

      FrFEACableSection(
          std::shared_ptr<chrono::fea::ChInertiaCosserat> minertia,        /// the inertia of this section
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
      // FIXME: il faut faire une diff entre les coeffs de drag et de masse ajoutee normaux et tangentiels
      double m_Cm; // TODO: voir si ces coeffs doivent etre gardes en data ou bien un pointeur vers les pptes d'element serait pas mieux...
      double m_Cd;
      double m_VIV_amp_factor;

    };
  }  // end namspace frydom::internal

}  // end namespace frydom



#endif //FRYDOM_FRFEACABLESECTION_H
