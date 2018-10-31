//
// Created by frongere on 27/09/18.
//

#ifndef FRYDOM_FRVECTOR_H
#define FRYDOM_FRVECTOR_H

#include "chrono/core/ChVector.h"

#include "MathUtils/Vector3d.h"
#include "MathUtils/Matrix.h"

#include "FrGeographic.h"


namespace frydom {


    class Position : public mathutils::Vector3d<double> {

    public:

        Position() : mathutils::Vector3d<double>() {}

        Position(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        Position(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        Position& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->mathutils::Vector3d<double>::operator=(other);
            return *this;
        }

        double GetX() const {
            return this->at(0);
        }

        double& GetX() {
            return this->at(0);
        }

        double GetY() const {
            return this->at(1);
        }

        double& GetY() {
            return this->at(1);
        }

        double GetZ() const {
            return this->at(2);
        }

        double& GetZ() {
            return this->at(2);
        }

    };


    class Direction : public mathutils::Vector3d<double> {

    public:

        Direction() : mathutils::Vector3d<double>() {}

        Direction(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        Direction(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        Direction& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->mathutils::Vector3d<double>::operator=(other);
            return *this;
        }

        double Getux() const {
            return this->at(0);
        }

        double& Getux() {
            return this->at(0);
        }

        double Getuy() const {
            return this->at(1);
        }

        double& Getuy() {
            return this->at(1);
        }

        double Getuz() const {
            return this->at(2);
        }

        double& Getuz() {
            return this->at(2);
        }

    };

    namespace internal {
        // =================================================================================================================
        // SYMBOLIC DIRECTIONS EXPRESSED IN THE NED FRAME (please do not forget the NED aspect !)
        // =================================================================================================================
        extern const Direction NORTH;        ///< North direction
        extern const Direction NORTH_EAST;   ///< North-East direction
        extern const Direction EAST;         ///< East direction
        extern const Direction SOUTH_EAST;   ///< South-East direction
        extern const Direction SOUTH;        ///< South direction
        extern const Direction SOUTH_WEST;   ///< South-West direction
        extern const Direction WEST;         ///< West direction
        extern const Direction NORTH_WEST;   ///< North-West direction
    } // end namespace internal

    class Velocity : public mathutils::Vector3d<double> {

    public:

        Velocity() : mathutils::Vector3d<double>() {}

        Velocity(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        Velocity(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        Velocity& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->Eigen::Matrix<Scalar, 3, 1>::operator=(other);
            return *this;
        }

        double GetVx() const {
            return this->at(0);
        }

        double& GetVx() {
            return this->at(0);
        }

        double GetVy() const {
            return this->at(1);
        }

        double& GetVy() {
            return this->at(1);
        }

        double GetVz() const {
            return this->at(2);
        }

        double& GetVz() {
            return this->at(2);
        }
    };


    class RotationalVelocity : public mathutils::Vector3d<double> {

    public:

        RotationalVelocity() : mathutils::Vector3d<double>() {}

        RotationalVelocity(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        RotationalVelocity(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        RotationalVelocity& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->Eigen::Matrix<Scalar, 3, 1>::operator=(other);
            return *this;
        }

        double GetWx() const {
            return this->at(0);
        }

        double& GetWx() {
            return this->at(0);
        }

        double GetWy() const {
            return this->at(1);
        }

        double& GetWy() {
            return this->at(1);
        }

        double GetWz() const {
            return this->at(2);
        }

        double& GetWz() {
            return this->at(2);
        }
    };


    class Acceleration : public mathutils::Vector3d<double> {

    public:

        Acceleration() : mathutils::Vector3d<double>() {}

        Acceleration(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        Acceleration(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        Acceleration& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->Eigen::Matrix<Scalar, 3, 1>::operator=(other);
            return *this;
        }

        double GetAccX() const {
            return this->at(0);
        }

        double& GetAccX() {
            return this->at(0);
        }

        double GetAccY() const {
            return this->at(1);
        }

        double& GetAccY() {
            return this->at(1);
        }

        double GetAccZ() const {
            return this->at(2);
        }

        double& GetAccZ() {
            return this->at(2);
        }
    };


    class RotationalAcceleration : public mathutils::Vector3d<double> {

    public:

        RotationalAcceleration() : mathutils::Vector3d<double>() {}

        RotationalAcceleration(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        RotationalAcceleration(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        RotationalAcceleration& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->Eigen::Matrix<Scalar, 3, 1>::operator=(other);
            return *this;
        }

        double GetWxp() const {
            return this->at(0);
        }

        double& GetWxp() {
            return this->at(0);
        }

        double GetWyp() const {
            return this->at(1);
        }

        double& GetWyp() {
            return this->at(1);
        }

        double GetWzp() const {
            return this->at(2);
        }

        double& GetWzp() {
            return this->at(2);
        }
    };


    class Force : public mathutils::Vector3d<double> {

    public:

        Force() : mathutils::Vector3d<double>() {}

        Force(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        Force(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        Force& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->Eigen::Matrix<Scalar, 3, 1>::operator=(other);
            return *this;
        }

        double GetFx() const {
            return this->at(0);
        }

        double& GetFx() {
            return this->at(0);
        }

        double GetFy() const {
            return this->at(1);
        }

        double& GetFy() {
            return this->at(1);
        }

        double GetFz() const {
            return this->at(2);
        }

        double& GetFz() {
            return this->at(2);
        }
    };

    class Moment : public mathutils::Vector3d<double> {

    public:

        Moment() : mathutils::Vector3d<double>() {}

        Moment(double x, double y, double z) : mathutils::Vector3d<double>(x, y, z) {}

        // This constructor allows to construct Vector2d from Eigen expressions
        template <class OtherDerived>
        Moment(const Eigen::MatrixBase<OtherDerived>& other) : mathutils::Vector3d<double>(other) {}

        // This method allows to assign Eigen expressions to Vector3d
        template <class OtherDerived>
        Moment& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
            this->Eigen::Matrix<Scalar, 3, 1>::operator=(other);
            return *this;
        }

        double GetMx() const {
            return this->at(0);
        }

        double& GetMx() {
            return this->at(0);
        }

        double GetMy() const {
            return this->at(1);
        }

        double& GetMy() {
            return this->at(1);
        }

        double GetMz() const {
            return this->at(2);
        }

        double& GetMz() {
            return this->at(2);
        }
    };




    namespace internal {

        template <class Vector>
        inline Vector& SwapFrameConvention(Vector& vector) {
            vector[1] = -vector[1];
            vector[2] = -vector[2];
            return vector;
        }

        template <class Vector>
        inline Vector SwapFrameConvention(const Vector& vector) {
            Vector out = vector;
            return SwapFrameConvention<Vector>(vector);
        }

        inline chrono::ChVector<double>& SwapFrameConvention(chrono::ChVector<double>& vector) {
            vector[1] = -vector[1];
            vector[2] = -vector[2];
            return vector;
        }

        /// Convert a chrono 3D vector into a FRyDoM Vector
        template <class Vector>
        inline Vector ChVectorToVector3d(const chrono::ChVector<double>& vector) {
            return Vector(vector.x(), vector.y(), vector.z()); // Always gives a FRyDoM vector expressed in NWU
        }

        /// Convert a mathutils Vector3d into a Chrono 3D vector
        inline chrono::ChVector<double> Vector3dToChVector(const mathutils::Vector3d<double>& vector3d) {
            return chrono::ChVector<double>(vector3d[0], vector3d[1], vector3d[2]);
        }

        inline chrono::ChVector<double> MakeNWUChVector(double& x, double& y, double& z, FRAME_CONVENTION fc) {
            if (IsNED(fc)) {
                return chrono::ChVector<double>(x, -y, -z);
            } else {
                return chrono::ChVector<double>(x, y, z);
            }
        }

        inline void SwapCoordinateConvention(double& x, double& y, double& z) {
            y = -y;
            z = -z;
        }


    }  // end namespace internal

}  // end namespace frydom

#endif //FRYDOM_FRVECTOR_H
