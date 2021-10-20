//
// Created by frongere on 12/10/2021.
//

#ifndef FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H
#define FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H

#include "frydom/core/force/FrForce.h"

namespace frydom {

  class FrAbkowitzManoeuvringForce : public FrForce {

   public:
    FrAbkowitzManoeuvringForce(const std::string &name, FrBody *body, const std::string &file);

    void Initialize() override;

   private:
    void Compute(double time) override;

    void DefineLogMessages() override;

    void LoadManoeuvringData();

   private:
    std::string c_filepath; ///< path to the JSON file containing the manoeuvring data

    double m_Lpp;
    double m_draft;

    double m_Xvv, m_Xvvvv, m_Xvvu, m_Xrr, m_Xrru, m_Xvr, m_Xvru;
    double m_Yv, m_Yvvv, m_Yvrr, m_Yvu, m_Yvuu, m_Yr, m_Yrrr, m_Yvvr, m_Yru, m_Yruu;
    double m_Nv, m_Nvvv, m_Nvrr, m_Nvu, m_Nvuu, m_Nr, m_Nrrr, m_Nvvr, m_Nru, m_Nruu;

  };

}  // end namespace frydom



#endif //FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H
