//
// Created by lletourn on 08/06/2021.
//

#ifndef FRYDOM_FRPROPELLERRUDDER_H
#define FRYDOM_FRPROPELLERRUDDER_H

#include "FrPropulsionActuator.h"

namespace frydom {

  class FrRudderForce;

  class FrFlapRudderForce;

  class FrPropellerForce;

  class FrFirstQuadrantPropellerForce;

  class FrFourQuadrantPropellerForce;

  class FrCPPForce;

  class FrPropellerRudder : public FrPropulsionActuator {

   public:
    FrPropellerRudder(const std::string &name, FrBody *body);

    void SetRudderAngle(double angle, ANGLE_UNIT unit);

    void SetPropellerRotationalVelocity(double omega, mathutils::FREQUENCY_UNIT unit);

    void ActivateInteractions(bool val);

    bool IsInteractions() const;

    FrFirstQuadrantPropellerForce *
    Add_FirstQuadrantPropeller(const std::string &name, Position propellerPositionInBody, const std::string &filename,
                               FRAME_CONVENTION fc);

    FrFourQuadrantPropellerForce *
    Add_FourQuadrantPropeller(const std::string &name, Position propellerPositionInBody, const std::string &filename,
                              FRAME_CONVENTION fc);

    FrCPPForce *
    Add_ControllablePitchPropeller(const std::string &name, Position propellerPositionInBody,
                                   const std::string &filename,
                                   FRAME_CONVENTION fc);

    FrRudderForce *
    Add_Rudder(const std::string &name, const std::shared_ptr<FrNode> &node, const std::string &filename);

    FrFlapRudderForce *
    Add_FlapRudder(const std::string &name, const std::shared_ptr<FrNode> &node, const std::string &filename);

    // Propeller Force

    Force GetPropellerForceInWorld(FRAME_CONVENTION fc) const;

    Force GetPropellerForceInBody(FRAME_CONVENTION fc) const;

    Torque GetPropellerTorqueInWorldAtPropeller(FRAME_CONVENTION fc) const;

    Torque GetPropellerTorqueInBodyAtPropeller(FRAME_CONVENTION fc) const;

    Torque GetPropellerTorqueInWorldAtCOG(FRAME_CONVENTION fc) const;

    Torque GetPropellerTorqueInBodyAtCOG(FRAME_CONVENTION fc) const;

    // Rudder Force

    Force GetRudderForceInWorld(FRAME_CONVENTION fc) const;

    Force GetRudderForceInBody(FRAME_CONVENTION fc) const;

    Torque GetRudderTorqueInWorldAtRudder(FRAME_CONVENTION fc) const;

    Torque GetRudderTorqueInBodyAtRudder(FRAME_CONVENTION fc) const;

    Torque GetRudderTorqueInWorldAtCOG(FRAME_CONVENTION fc) const;

    Torque GetRudderTorqueInBodyAtCOG(FRAME_CONVENTION fc) const;

   private:

    void Initialize() override;

    void Compute(double time) override;

    void ComputeVelocityInSlipStream(double u_0, double u_ra, double u_pa, double thrust, double &u_rp, double &A_rp, double &kd);

    void DefineLogMessages() override;

    std::shared_ptr<FrPropellerForce> m_propellerForce;
    std::shared_ptr<FrRudderForce> m_rudderForce;

    double m_longitudinalDistancePropellerRudder;

    GeneralizedForceTorsor c_propellerForceInBody;
    GeneralizedForceTorsor c_rudderForceInWorld;

    bool has_interactions;

    double c_uPA, c_vPA;
    double c_uRA, c_vRA;
    double c_uRP, c_vRP;
    double c_kd, c_A_RP;

  };

  std::shared_ptr<FrPropellerRudder> make_propeller_rudder(const std::string &name, const std::shared_ptr<FrBody> &body);

} // end namespace frydom

#endif //FRYDOM_FRPROPELLERRUDDER_H
