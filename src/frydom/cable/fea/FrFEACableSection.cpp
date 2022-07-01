//
// Created by frongere on 05/05/2020.
//

#include "FrFEACableSection.h"


namespace frydom {

  namespace internal {


    FrFEACableSection::FrFEACableSection(std::shared_ptr<chrono::fea::ChInertiaCosserat> minertia,
                                         std::shared_ptr<chrono::fea::ChElasticityCosserat> melasticity)
        : ChBeamSectionCosserat(minertia, melasticity),
        m_Cm(0.),
        m_Cd(0.),
        m_VIV_amp_factor(0.) {}

    FrFEACableSection::FrFEACableSection(std::shared_ptr<chrono::fea::ChInertiaCosserat> minertia,
                                         std::shared_ptr<chrono::fea::ChElasticityCosserat> melasticity,
                                         std::shared_ptr<chrono::fea::ChPlasticityCosserat> mplasticity)
        : ChBeamSectionCosserat(minertia, melasticity, mplasticity),
          m_Cm(0.),
          m_Cd(0.),
          m_VIV_amp_factor(0.) {}

    FrFEACableSection::FrFEACableSection(std::shared_ptr<chrono::fea::ChInertiaCosserat> minertia,
                                         std::shared_ptr<chrono::fea::ChElasticityCosserat> melasticity,
                                         std::shared_ptr<chrono::fea::ChPlasticityCosserat> mplasticity,
                                         std::shared_ptr<chrono::fea::ChDampingCosserat> mdamping)
        : ChBeamSectionCosserat(minertia, melasticity, mplasticity, mdamping),
          m_Cm(0.),
          m_Cd(0.),
          m_VIV_amp_factor(0.) {}

    void FrFEACableSection::SetNormalAddedMassCoeff(const double Cm) {
      m_Cm = Cm;
    }

    double FrFEACableSection::GetCm() const {
      return m_Cm;
    }

    void FrFEACableSection::SetNormalDragCoeff(const double Cd) {
      m_Cd = Cd;
    }

    double FrFEACableSection::GetCd() const {
      return m_Cd;
    }

    void FrFEACableSection::SetVIVAmpFactor(const double &G) {
      m_VIV_amp_factor = G;
    }

    double FrFEACableSection::GetVIVAmpFactor() const {
      return m_VIV_amp_factor;
    }

  }  // end namespace frydom::internal

}  // end namespace frydom
