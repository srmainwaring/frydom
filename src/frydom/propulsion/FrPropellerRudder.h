//
// Created by lletourn on 08/06/2021.
//

#ifndef FRYDOM_FRPROPELLERRUDDER_H
#define FRYDOM_FRPROPELLERRUDDER_H

#include "frydom/core/force/FrForce.h"

namespace frydom {

  class FrRudderForce;

  class FrFlapRudderForce;

  class FrPropellerForce;

  class FrFirstQuadrantPropellerForce;

  class FrFourQuadrantPropellerForce;

  class FrCPPForce;

  class FrPropellerRudder : public FrForce {

   public:
    FrPropellerRudder(const std::string &name, FrBody *body);

    FrFirstQuadrantPropellerForce *
    Add_FirstQuadrantPropeller(const std::string &name, Position propellerPositionInBody, const std::string &filename);

    FrFourQuadrantPropellerForce *
    Add_FourQuadrantPropeller(const std::string &name, Position propellerPositionInBody, const std::string &filename);

    FrCPPForce *
    Add_ControllablePitchPropeller(const std::string &name, Position propellerPositionInBody,
                                   const std::string &filename);

    FrRudderForce *
    Add_Rudder(const std::string &name, const std::shared_ptr<FrNode> &node, const std::string &filename);

    FrFlapRudderForce *
    Add_FlapRudder(const std::string &name, const std::shared_ptr<FrNode> &node, const std::string &filename);

   private:

    void Initialize() override;

    void Compute(double time) override;

    std::shared_ptr<FrPropellerForce> m_propellerForce;
    std::shared_ptr<FrRudderForce> m_rudderForce;

    double m_longitudinalDistancePropellerRudder;

  };

  std::shared_ptr<FrPropellerRudder> make_propeller_rudder(const std::string &name, FrBody *body);

} // end namespace frydom
#endif //FRYDOM_FRPROPELLERRUDDER_H
