//
// Created by lletourn on 03/06/2021.
//

#ifndef FRYDOM_FRFIRSTQUADRANTPROPELLERFORCE_H
#define FRYDOM_FRFIRSTQUADRANTPROPELLERFORCE_H

#include "frydom/propulsion/FrPropellerForce.h"
#include "MathUtils/LookupTable1D.h"

namespace frydom {

  class FrFirstQuadrantPropellerForce : public FrPropellerForce {

   public:

    FrFirstQuadrantPropellerForce(const std::string &name, FrBody *body, Position propellerPositionInBody,
                                  const std::string &fileCoefficients, FRAME_CONVENTION fc);

    virtual double kt(double J) const;

    virtual double kq(double J) const;

    void SetPitchRatio(double P_D) override {};

    double GetPitchRatio() const override { return 0.; };

    double ComputeAdvanceRatio();

   protected:

    GeneralizedForce ComputeGeneralizedForceInBody() override;

    void ReadCoefficientsFile() override;

    std::string c_fileCoefficients;

    mathutils::LookupTable1D<double> m_coefficients;

  };

  std::shared_ptr<FrFirstQuadrantPropellerForce>
  make_first_quadrant_propeller_force(const std::string &name,
                                      const std::shared_ptr<FrBody> &body,
                                      Position propellerPositionInBody,
                                      const std::string &fileCoefficients,
                                      FRAME_CONVENTION fc);
} // end namespace frydom

#endif //FRYDOM_FRFIRSTQUADRANTPROPELLERFORCE_H
