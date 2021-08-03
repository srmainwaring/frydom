//
// Created by lletourn on 02/06/2021.
//

#ifndef FRYDOM_FRCPPFORCE_H
#define FRYDOM_FRCPPFORCE_H

#include "frydom/propulsion/FrFourQuadrantPropellerForce.h"
#include "MathUtils/LookupTable2D.h"

namespace frydom {

  class FrCPPForce : public FrFourQuadrantPropellerForce {

   public:

    FrCPPForce(const std::string &name,
               FrBody *body,
               Position propellerPositionInBody,
               const std::string &fileCoefficients,
               FRAME_CONVENTION fc);

    void SetPitchRatio(double P_D) override;

    double GetPitchRatio() const override;

    double Ct(double gamma) const override;

    double Cq(double gamma) const override;

//    mathutils::LookupTable2d<double>& GetCoeff() { return m_coefficients;}

   private:

    void ReadPropellerTable(const json &node) override;

    mathutils::LookupTable2d<double> m_coefficients;
    double m_pitchRatio;

  };

  std::shared_ptr<FrCPPForce>
  make_controllable_pitch_propeller(const std::string &name,
                                    const std::shared_ptr<FrBody> &body,
                                    Position propellerPositionInBody,
                                    const std::string &fileCoefficients,
                                    FRAME_CONVENTION fc);

} // end namespace frydom

#endif //FRYDOM_FRCPPFORCE_H
