//
// Created by frongere on 06/02/19.
//

#ifndef FRYDOM_FRACTUATOR_H
#define FRYDOM_FRACTUATOR_H

#include <memory>

#include "frydom/core/link/FrLinkBase.h"

namespace frydom {


  // Forward declaration
  class FrLink;

  class FrFunctionBase;

  class FrActuator : public FrLinkBase, public FrLoggable<FrLink> {

   protected:

    FrLink *m_actuatedLink;


   public:

    FrActuator(const std::string &name, const std::string &type_name, FrLink *actuatedLink);

//        /// Tells if the link is broken, for excess of pulling/pushing.
//        virtual bool IsBroken() const override;
//
//        /// Set the 'broken' status vof this link.
//        virtual void SetBroken(bool broken) override;

    /// Tells if the link is currently active, in general,
    /// that is tells if it must be included into the system solver or not.
    /// This method cumulates the effect of various flags (so a link may
    /// be not active either because disabled, or broken, or not valid)
    bool IsActive() const override;

    /// Set the motor function, to control the motion, velocity or force depending on the control case selected
    /// \param function motor function
    virtual void SetMotorFunction(const FrFunctionBase &function) = 0;

    /// Get the power delivered by the motor
    /// \return power delivered by the motor
    virtual double GetMotorPower() const = 0;


    /// Get the motor force applied on body 1, in the node reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return motor force applied on Body 1
    virtual Force GetMotorForceInNode(FRAME_CONVENTION fc) const = 0;

    /// Get the motor force applied on body 1, in the body 1 reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return motor force applied on Body 1
    Force GetMotorForceInBody1(FRAME_CONVENTION fc) const;

    /// Get the motor force applied on body 1, in the body 2 reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return motor force applied on Body 1
    Force GetMotorForceInBody2(FRAME_CONVENTION fc) const;

    /// Get the motor torque applied on body 1 at the node reference frame origin, in the node reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return motor torque applied on body 1 at the node reference frame origin
    virtual Torque GetMotorTorqueInNode(FRAME_CONVENTION fc) const = 0;

    /// Get the motor torque applied on body 1 at the body 1 COG, in body 1 reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return motor torque applied on body 1 at the body 1 COG, in body 1 reference frame
    virtual Torque GetMotorTorqueAtCOGInBody1(FRAME_CONVENTION fc) const;

    /// Get the motor torque applied on body 1 at the body 2 COG, in body 2 reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return motor torque applied on body 1 at the body 2 COG, in body 2 reference frame
    virtual Torque GetMotorTorqueAtCOGInBody2(FRAME_CONVENTION fc) const;

    /// Get the motor torque applied on body 1 at the the node reference frame origin, in the body 1 reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return motor torque applied on body 1 at the node reference frame origin
    virtual Torque GetMotorTorqueInBody1(FRAME_CONVENTION fc) const;

    /// Get the motor torque applied on body 2 at the node reference frame origin, in the body 2 reference frame
    /// \param fc Frame convention (NED/NWU)
    /// \return motor torque applied on body 2 at the node reference frame origin
    virtual Torque GetMotorTorqueInBody2(FRAME_CONVENTION fc) const;

  };


}  // end namespace frydom


#endif //FRYDOM_FRACTUATOR_H
