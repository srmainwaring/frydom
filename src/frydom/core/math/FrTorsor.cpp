// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#include "FrTorsor.h"

namespace frydom {


  FrTorsor::FrTorsor(const Force &resultant, const Torque &moment,
                     const Position &point, FRAME_CONVENTION fc) {
    Set(resultant, moment, point, fc);
  }

  void FrTorsor::Set(const Force &resultant, const Torque &moment,
                     const Position &point, FRAME_CONVENTION fc) {
    m_resultant = resultant;
    m_moment = moment;
    m_point = point;
    if (IsNED(fc)) {
      m_resultant = internal::SwapFrameConvention(m_resultant);
      m_moment = internal::SwapFrameConvention(m_moment);
      m_point = internal::SwapFrameConvention(m_point);
    }
  }

  Torque FrTorsor::TransportMomentAtPoint(const Position &newPoint, FRAME_CONVENTION fc) const {
    Position tempPos = newPoint;
    if (IsNED(fc)) tempPos = internal::SwapFrameConvention(tempPos);
    Position newToOld = m_point - tempPos;
    Torque tempMoment = m_moment + newToOld.cross(m_resultant);
    if (IsNED(fc)) tempMoment = internal::SwapFrameConvention(tempMoment);
    return tempMoment;
  }

  Position FrTorsor::GetPoint(FRAME_CONVENTION fc) const {
    Position tempPos = m_point;
    if (IsNED(fc)) internal::SwapFrameConvention(tempPos);
    return tempPos;
  }

  std::ostream &FrTorsor::cout(std::ostream &os) const {

    os << std::endl;
    os << "FrTorsor: \n";
    os << "resultant : (" << m_resultant[0] << "," << m_resultant[1] << "," << m_resultant[2] << ")" << std::endl;
    os << "moment : (" << m_moment[0] << "," << m_moment[1] << "," << m_moment[2] << ")" << std::endl;
    os << "expressed at point : (" << m_point[0] << "," << m_point[1] << "," << m_point[2] << ")" << std::endl;
    os << std::endl;

    return os;
  }

  std::ostream &operator<<(std::ostream &os, const FrTorsor &FrTorsor) {
    return FrTorsor.cout(os);
  }





  // GeneralizedForceTorsor

  GeneralizedForceTorsor::GeneralizedForceTorsor(const Force &force, const Torque &torque, const Position &point,
                                                 FRAME_CONVENTION fc) :
      FrTorsor(force, torque, point, fc) {}

  GeneralizedForceTorsor::GeneralizedForceTorsor(const GeneralizedForce generalizedForce, const Position &point,
                                                 FRAME_CONVENTION fc) :
      FrTorsor(generalizedForce.GetForce(), generalizedForce.GetTorque(), point, fc) {
  }

  Force GeneralizedForceTorsor::GetForce() const {
    return m_resultant;
  }

  Torque GeneralizedForceTorsor::GetTorqueAtPoint(const Position &point, FRAME_CONVENTION fc) const {
    return TransportMomentAtPoint(point, fc);
  }

  void
  GeneralizedForceTorsor::Set(const Force &force, const Torque &torque, const Position &point, FRAME_CONVENTION fc) {
    FrTorsor::Set(force, torque, point, fc);
  }





  // GeneralizedVelocityTorsor

  GeneralizedVelocityTorsor::GeneralizedVelocityTorsor(const Velocity &linearvelocity,
                                                       const AngularVelocity &angularVelocity,
                                                       const Position &point, FRAME_CONVENTION fc) :
      FrTorsor(angularVelocity, linearvelocity, point, fc) {

  }

  Force GeneralizedVelocityTorsor::GetAngularVelocity() const {
    return m_resultant;
  }

  Torque GeneralizedVelocityTorsor::GetLinearVelocityAtPoint(const Position &point, FRAME_CONVENTION fc) const {
    return TransportMomentAtPoint(point, fc);
  }

  void GeneralizedVelocityTorsor::Set(const Velocity &linearvelocity, const AngularVelocity &angularVelocity,
                                      const Position &point, FRAME_CONVENTION fc) {
    FrTorsor::Set(angularVelocity, linearvelocity, point, fc);
  }
} //end namespace frydom