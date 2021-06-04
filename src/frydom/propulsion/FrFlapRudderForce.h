//
// Created by lletourn on 01/06/2021.
//

#ifndef FRYDOM_FRFLAPRUDDERFORCE_H
#define FRYDOM_FRFLAPRUDDERFORCE_H

#include "frydom/propulsion/FrRudderForce.h"
#include "MathUtils/LookupTable2D.h"

namespace frydom{

class FrFlapRudderForce : public FrRudderForce {

 public:

  FrFlapRudderForce(const std::string& name, FrBody* body,
                    const std::shared_ptr<FrNode>& node, const std::string& fileCoefficients);

  virtual double GetFlapAngle() const;

  void ReadCoefficientsFile() override;

  double GetLiftCoefficient(double attackAngle) const override;

  double GetDragCoefficient(double attackAngle) const override;

  double GetTorqueCoefficient(double attackAngle) const override;

//  mathutils::LookupTable2d<double>& GetCoeff() { return m_coefficients;}

 private:

  double m_flapLaw{};

  mathutils::LookupTable2d<double> m_coefficients;

};

} // end namespace frydom
#endif //FRYDOM_FRFLAPRUDDERFORCE_H
