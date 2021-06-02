//
// Created by lletourn on 02/06/2021.
//

#ifndef FRYDOM_FRCPPFORCE_H
#define FRYDOM_FRCPPFORCE_H

#include "FrPropellerForce.h"
#include "MathUtils/LookupTable2D.h"

namespace frydom {

  class FrCPPForce : public FrPropellerForce {

   public:

    enum SCREW_DIRECTION {LEFT_HANDED, RIGHT_HANDED};

    FrCPPForce(const std::string &name, FrBody *body, Position propellerPositionInBody,
               const std::string &fileCoefficients);


    GeneralizedForce ComputeGeneralizedForceInWorld() override;

    void SetPitchRatio(double PD);
    double GetPitchRatio() const;

    void SetScrewDirection(SCREW_DIRECTION dir);

    signed int GetScrewDirectionSign() const;

    double Ct(double gamma, double PD) const;
    double Cq(double gamma, double PD) const;

    mathutils::LookupTable2d<double>& GetCoeff() { return m_coefficients;}

   private:

    void ReadCoefficientsFile() override;

    double ComputeAdvanceAngle();


    std::string c_fileCoefficients;

    std::string m_name;
    std::string m_reference;

    mathutils::LookupTable2d<double> m_coefficients;
    double m_pitchRatio;
    SCREW_DIRECTION m_screwDirection;

  };

} // end namespace frydom

#endif //FRYDOM_FRCPPFORCE_H
