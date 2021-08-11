//
// Created by lletourn on 08/06/2021.
//

#ifndef FRYDOM_FRFOURQUADRANTPROPELLERFORCE_H
#define FRYDOM_FRFOURQUADRANTPROPELLERFORCE_H

#include "frydom/propulsion/FrPropellerForce.h"
#include "MathUtils/LookupTable1D.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/logging/FrEventLogger.h"
#include "FrPropellerRudder.h"


namespace frydom {

  class FrFourQuadrantPropellerForce : public FrPropellerForce {

   public:

    FrFourQuadrantPropellerForce(const std::string &name, FrBody *body, Position propellerPositionInBody,
                                 const std::string &fileCoefficients, FRAME_CONVENTION fc);

    virtual double Ct(double gamma) const;

    virtual double Cq(double gamma) const;

    void SetPitchRatio(double P_D) override {};

    double GetPitchRatio() const override { return 0.; };

//    mathutils::LookupTable2d<double>& GetCoeff() { return m_coefficients;}

    double ComputeAdvanceAngle();

   private:

    GeneralizedForce ComputeGeneralizedForceInBody() override;

    void ReadCoefficientsFile() override;

    virtual void ReadPropellerTable(const json &node);

    std::string c_fileCoefficients;

    mathutils::LookupTable1D<double> m_coefficients;

  };

  std::shared_ptr<FrFourQuadrantPropellerForce>
  make_four_quadrant_propeller_force(const std::string &name,
                                     const std::shared_ptr<FrBody> &body,
                                     Position propellerPositionInBody,
                                     const std::string &fileCoefficients,
                                     FRAME_CONVENTION fc);

}// end namespace frydom
#endif //FRYDOM_FRFOURQUADRANTPROPELLERFORCE_H
