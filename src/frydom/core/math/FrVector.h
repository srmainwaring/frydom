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


#ifndef FRYDOM_FRVECTOR_H
#define FRYDOM_FRVECTOR_H

#include <cfloat>

#include "chrono/core/ChVector.h"
#include "MathUtils/Unit.h"

#include "frydom/core/common/FrConvention.h"


namespace frydom {

  /// Class for representing a translation vector in cartesian coordinates
  class Translation : public chrono::ChVectorN<double, 3> {

   public:

    Translation() : chrono::ChVectorN<double, 3>() {}

    Translation(double x, double y, double z) : chrono::ChVectorN<double, 3>(x, y, z) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    Translation(const Eigen::MatrixBase<OtherDerived> &other) :chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    Translation &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double GetDx() const {
      return this->x();
    }

    double &GetDx() {
      return this->x();
    }

    double GetDy() const {
      return this->y();
    }

    double &GetDy() {
      return this->y();
    }

    double GetDz() const {
      return this->z();
    }

    double &GetDz() {
      return this->z();
    }

  };

  /// Class for representing a position vector in cartesian coordinates
  class Position : public chrono::ChVectorN<double, 3> {

   public:

    Position() : chrono::ChVectorN<double, 3>(0., 0., 0.) {}

    Position(double x, double y, double z) : chrono::ChVectorN<double, 3>(x, y, z) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    Position(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    Position &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double GetX() const {
      return this->x();
    }

    double &GetX() {
      return this->x();
    }

    double GetY() const {
      return this->y();
    }

    double &GetY() {
      return this->y();
    }

    double GetZ() const {
      return this->z();
    }

    double &GetZ() {
      return this->z();
    }

  };


  /// Class for stacking cardan angles into a vector (unit is to be managed by user)
  class CardanAngles : public chrono::ChVectorN<double, 3> {

   public:

    CardanAngles() : chrono::ChVectorN<double, 3>(0., 0., 0.) {}

    CardanAngles(double roll, double pitch, double yaw) : chrono::ChVectorN<double, 3>(roll, pitch, yaw) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    CardanAngles(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    CardanAngles &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double GetRoll() const {
      return this->x();
    }

    double &GetRoll() {
      return this->x();
    }

    double GetPitch() const {
      return this->y();
    }

    double &GetPitch() {
      return this->y();
    }

    double GetYaw() const {
      return this->z();
    }

    double &GetYaw() {
      return this->z();
    }

  };


  /// Class to stack linear and
  class GeneralizedPosition : public chrono::ChVectorN<double, 6> {

   public:

    GeneralizedPosition() : chrono::ChVectorN<double, 6>() {}

    GeneralizedPosition(const Position &pos, const CardanAngles &cardanAngles) {
      *this << pos[0], pos[1], pos[2], cardanAngles[0], cardanAngles[1], cardanAngles[2];
    }


    // This constructor allows to construct Vector6d from Eigen expressions
    template<class OtherDerived>
    GeneralizedPosition(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 6>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    GeneralizedPosition &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 6>::operator=(other);
      return *this;
    }

    Position GetPosition() const {
      return this->block<3, 1>(0, 0);
    }

    void SetPosition(const Position &pos) {
      this->block<3, 1>(0, 0) = pos;
    }

    CardanAngles GetCardanAngles() const {
      return this->block<3, 1>(3, 0);
    }

    void SetCardanAngles(const CardanAngles &cardanAngles) {
      this->block<3, 1>(3, 0) = cardanAngles;
    }

  };

  /**
   * \class Direction
   * \brief Class for defining a direction.
   */
  class Direction : public chrono::ChVectorN<double, 3> {

   public:

    Direction() : chrono::ChVectorN<double, 3>(0., 0., 0.) {}

    Direction(double x, double y, double z) : chrono::ChVectorN<double, 3>(x, y, z) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    Direction(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    Direction &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double Getux() const {
      return this->x();
    }

    double &Getux() {
      return this->x();
    }

    double Getuy() const {
      return this->y();
    }

    double &Getuy() {
      return this->y();
    }

    double Getuz() const {
      return this->z();
    }

    double &Getuz() {
      return this->z();
    }

    bool IsUnit() const {
      return std::abs(this->norm() - 1.) < DBL_EPSILON;
    }

  };

//    #define XDIR Direction(1., 0., 0.);
//    #define YDIR Direction(0., 1., 0.);
//    #define ZDIR Direction(0., 0., 1.);


  // TODO : avoir un generalizedDirection ?? (direction lineaire + direction en rotation) --> definir produit vectoriel -> utile pour maillage

  /**
   * \class Velocity
   * \brief Class for defining a linear velocity.
   */
  class Velocity : public chrono::ChVectorN<double, 3> {

   public:

    Velocity() : chrono::ChVectorN<double, 3>(0., 0., 0.) {}

    Velocity(double x, double y, double z) : chrono::ChVectorN<double, 3>(x, y, z) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    Velocity(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    Velocity &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double GetVx() const {
      return this->x();
    }

    double &GetVx() {
      return this->x();
    }

    double GetVy() const {
      return this->y();
    }

    double &GetVy() {
      return this->y();
    }

    double GetVz() const {
      return this->z();
    }

    double &GetVz() {
      return this->z();
    }

    bool IsUnit() const {
      return std::abs(this->norm() - 1.) < DBL_EPSILON;
    }

  };

  /**
   * \class AngularVelocity
   * \brief Class for defining an angular velocity.
   */
  class AngularVelocity : public chrono::ChVectorN<double, 3> {

   public:

    AngularVelocity() : chrono::ChVectorN<double, 3>(0., 0., 0.) {}

    AngularVelocity(double x, double y, double z) : chrono::ChVectorN<double, 3>(x, y, z) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    AngularVelocity(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    AngularVelocity &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double GetWx() const {
      return this->x();
    }

    double &GetWx() {
      return this->x();
    }

    double GetWy() const {
      return this->y();
    }

    double &GetWy() {
      return this->y();
    }

    double GetWz() const {
      return this->z();
    }

    double &GetWz() {
      return this->z();
    }

    bool IsUnit() const {
      return std::abs(this->norm() - 1.) < DBL_EPSILON;
    }

  };

  /**
   * \class GeneralizedVelocity
   * \brief Class for defining a generalized velocity.
   */
  class GeneralizedVelocity : public chrono::ChVectorN<double, 6> {

   public:

    GeneralizedVelocity() : chrono::ChVectorN<double, 6>() {
      this->setZero();
    }

    GeneralizedVelocity(const Velocity &vel, const AngularVelocity &rotVel) {
      *this << vel[0], vel[1], vel[2], rotVel[0], rotVel[1], rotVel[2];
    }

    // This constructor allows to construct Vector6d from Eigen expressions
    template<class OtherDerived>
    GeneralizedVelocity(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 6>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    GeneralizedVelocity &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 6>::operator=(other);
      return *this;
    }

    Velocity GetVelocity() const {
      return this->block<3, 1>(0, 0);
    }

    void SetVelocity(const Velocity &vel) {
      this->block<3, 1>(0, 0) = vel;
    }

    AngularVelocity GetAngularVelocity() const {
      return this->block<3, 1>(3, 0);
    }

    void SetAngularVelocity(const AngularVelocity &rotVel) {
      this->block<3, 1>(3, 0) = rotVel;
    }

  };

  // =================================================================================================================
  // SYMBOLIC VELOCITY FUNCTION EXPRESSED WITH CARDINAL
  // =================================================================================================================

  const Direction NORTH(FRAME_CONVENTION fc);

  const Direction NORTH_EAST(FRAME_CONVENTION fc);

  const Direction EAST(FRAME_CONVENTION fc);

  const Direction SOUTH_EAST(FRAME_CONVENTION fc);

  const Direction SOUTH(FRAME_CONVENTION fc);

  const Direction SOUTH_WEST(FRAME_CONVENTION fc);

  const Direction WEST(FRAME_CONVENTION fc);

  const Direction NORTH_WEST(FRAME_CONVENTION fc);

  const Direction UP(FRAME_CONVENTION fc);

  const Direction DOWN(FRAME_CONVENTION fc);


  /**
   * \class Acceleration
   * \brief Class for defining a linear acceleration.
   */
  class Acceleration : public chrono::ChVectorN<double, 3> {

   public:

    Acceleration() : chrono::ChVectorN<double, 3>(0., 0., 0.) {}

    Acceleration(double x, double y, double z) : chrono::ChVectorN<double, 3>(x, y, z) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    Acceleration(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    Acceleration &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double GetAccX() const {
      return this->x();
    }

    double &GetAccX() {
      return this->x();
    }

    double GetAccY() const {
      return this->y();
    }

    double &GetAccY() {
      return this->y();
    }

    double GetAccZ() const {
      return this->z();
    }

    double &GetAccZ() {
      return this->z();
    }
  };

  /**
   * \class AngularAcceleration
   * \brief Class for defining an angular acceleration.
   */
  class AngularAcceleration : public chrono::ChVectorN<double, 3> {

   public:

    AngularAcceleration() : chrono::ChVectorN<double, 3>(0., 0., 0.) {}

    AngularAcceleration(double x, double y, double z) : chrono::ChVectorN<double, 3>(x, y, z) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    AngularAcceleration(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    AngularAcceleration &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double GetWxp() const {
      return this->x();
    }

    double &GetWxp() {
      return this->x();
    }

    double GetWyp() const {
      return this->y();
    }

    double &GetWyp() {
      return this->y();
    }

    double GetWzp() const {
      return this->z();
    }

    double &GetWzp() {
      return this->z();
    }
  };

  /**
   * \class GeneralizedAcceleration
   * \brief Class for defining a generalized acceleration.
   */
  class GeneralizedAcceleration : public chrono::ChVectorN<double, 6> {

   public:

    GeneralizedAcceleration() : chrono::ChVectorN<double, 6>() {
      this->setZero();
    }

    GeneralizedAcceleration(const Acceleration &acc, const AngularAcceleration &rotAcc) {
      *this << acc[0], acc[1], acc[2], rotAcc[0], rotAcc[1], rotAcc[2];
    }

    // This constructor allows to construct Vector6d from Eigen expressions
    template<class OtherDerived>
    GeneralizedAcceleration(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 6>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    GeneralizedAcceleration &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 6>::operator=(other);
      return *this;
    }

    Acceleration GetAcceleration() const {
      return this->block<3, 1>(0, 0);
    }

    void SetAcceleration(const Acceleration &acc) {
      this->block<3, 1>(0, 0) = acc;
    }

    AngularAcceleration GetAngularAcceleration() const {
      return this->block<3, 1>(3, 0);
    }

    void SetAngularAcceleration(const AngularAcceleration &angAcc) {
      this->block<3, 1>(3, 0) = angAcc;
    }

  };

  /**
   * \class Force
   * \brief Class for defining a force.
   */
  class Force : public chrono::ChVectorN<double, 3> {

   public:

    Force() : chrono::ChVectorN<double, 3>(0., 0., 0.) {}

    Force(double x, double y, double z) : chrono::ChVectorN<double, 3>(x, y, z) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    Force(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    Force &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double GetFx() const {
      return this->x();
    }

    double &GetFx() {
      return this->x();
    }

    double GetFy() const {
      return this->y();
    }

    double &GetFy() {
      return this->y();
    }

    double GetFz() const {
      return this->z();
    }

    double &GetFz() {
      return this->z();
    }
  };

  /**
   * \class Torque
   * \brief Class for defining a torque.
   */
  class Torque : public chrono::ChVectorN<double, 3> {

   public:

    Torque() : chrono::ChVectorN<double, 3>(0., 0., 0.) {}

    Torque(double x, double y, double z) : chrono::ChVectorN<double, 3>(x, y, z) {}

    // This constructor allows to construct Vector2d from Eigen expressions
    template<class OtherDerived>
    Torque(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 3>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    Torque &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 3>::operator=(other);
      return *this;
    }

    double GetMx() const {
      return this->x();
    }

    double &GetMx() {
      return this->x();
    }

    double GetMy() const {
      return this->y();
    }

    double &GetMy() {
      return this->y();
    }

    double GetMz() const {
      return this->z();
    }

    double &GetMz() {
      return this->z();
    }
  };

  /**
   * \class GeneralizedForce
   * \brief Class for defining a generalized force.
   */
  class GeneralizedForce : public chrono::ChVectorN<double, 6> {

   public:

    GeneralizedForce() : chrono::ChVectorN<double, 6>() {
      this->setZero();
    }

    GeneralizedForce(const Force &force, const Torque &torque) {
      *this << force[0], force[1], force[2], torque[0], torque[1], torque[2];
    }

    // This constructor allows to construct Vector6d from Eigen expressions
    template<class OtherDerived>
    GeneralizedForce(const Eigen::MatrixBase<OtherDerived> &other) : chrono::ChVectorN<double, 6>(other) {}

    // This method allows to assign Eigen expressions to Vector3d
    template<class OtherDerived>
    GeneralizedForce &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
      this->chrono::ChVectorN<double, 6>::operator=(other);
      return *this;
    }

    Force GetForce() const {
      return this->block<3, 1>(0, 0);
    }

    void SetForce(const Force &force) {
      this->block<3, 1>(0, 0) = force;
    }

    Torque GetTorque() const {
      return this->block<3, 1>(3, 0);
    }

    void SetTorque(const Torque &moment) {
      this->block<3, 1>(3, 0) = moment;
    }

  };

  template<class Vector>
  double GetProjectedAngleAroundZ(const Vector vector, mathutils::ANGLE_UNIT unit) {
    auto angle = atan2(vector.x(), vector.y());
    if (unit == mathutils::DEG) { angle *= RAD2DEG; }
    return angle;
  }


  namespace internal {
    /// Swap the frame convention of a templated Vector: replace the argument vector and return a reference vector.
    template<class Vector>
    inline Vector &SwapFrameConvention(Vector &vector) {
      vector[1] = -vector[1];
      vector[2] = -vector[2];
      return vector;
    }

    /// Swap the frame convention of a templated Vector: only return a reference vector, does not change the argument vector
    template<class Vector>
    inline Vector SwapFrameConvention(const Vector &vector) {
      Vector out = vector;
      SwapFrameConvention<Vector>(out);
      return out;
    }

    /// Convert a chrono 3D vector into a FRyDoM Vector
    template<class Vector>
    inline Vector ChVectorToVector3d(const chrono::ChVector<double> &vector) {
      return Vector(vector.x(), vector.y(), vector.z()); // Always gives a FRyDoM vector expressed in NWU
    }

    /// Convert a mathutils Vector3d into a Chrono 3D vector
    inline chrono::ChVector<double> Vector3dToChVector(const chrono::ChVectorN<double, 3> &vector3d) {
      return chrono::ChVector<double>(vector3d[0], vector3d[1], vector3d[2]);
    }

    inline void SwapCoordinateConvention(double &x, double &y, double &z) {
      y = -y;
      z = -z;
    }

  }  // end namespace frydom::internal



}  // end namespace frydom

#endif //FRYDOM_FRVECTOR_H
