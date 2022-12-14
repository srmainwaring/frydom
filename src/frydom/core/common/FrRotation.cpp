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


#include "FrRotation.h"

#include "frydom/core/math/FrMatrix.h"


namespace frydom {


  // FrUnitQuaternion

  FrUnitQuaternion::FrUnitQuaternion() : m_chronoQuaternion() {
    SetNullRotation();
  }

  FrUnitQuaternion::FrUnitQuaternion(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc) {
    Set(q0, q1, q2, q3, fc);
  }

  FrUnitQuaternion::FrUnitQuaternion(double q0, double q1, double q2, double q3, bool non_normalized,
                                     FRAME_CONVENTION fc) {
    Set(q0, q1, q2, q3, non_normalized, fc);
  }

  FrUnitQuaternion::FrUnitQuaternion(const Direction &axis, double angleRAD, FRAME_CONVENTION fc) {
    Set(axis, angleRAD, fc);
  }

  FrUnitQuaternion::FrUnitQuaternion(const FrUnitQuaternion &other) {
    Set(other);
  }

  void FrUnitQuaternion::Set(double q0, double q1, double q2, double q3, FRAME_CONVENTION fc) {
    Set(q0, q1, q2, q3, false, fc);
  }

  void FrUnitQuaternion::Set(double q0, double q1, double q2, double q3, bool non_normalized, FRAME_CONVENTION fc) {
    if (IsNED(fc))
      // Internal chrono quaternion must always be in NWU convention
      internal::SwapQuaternionElementsFrameConvention(q0, q1, q2, q3);
    m_chronoQuaternion.Set(q0, q1, q2, q3);
    if (non_normalized) Normalize();
    else
      assert(IsRotation());

  }

  void FrUnitQuaternion::Set(const FrUnitQuaternion &quaternion) {
    m_chronoQuaternion.Set(quaternion.m_chronoQuaternion);
    assert(IsRotation());
  }

  void FrUnitQuaternion::Set(const Direction &axis, double angleRAD, FRAME_CONVENTION fc) {
    assert((axis.norm() - 1.) < 1e-8);
    auto axisTmp = axis;
    if (IsNED(fc)) axisTmp = internal::SwapFrameConvention<Direction>(axisTmp);
    m_chronoQuaternion = chrono::Q_from_AngAxis(angleRAD, internal::Vector3dToChVector(axisTmp));
    assert(IsRotation());
  }

  void FrUnitQuaternion::Set(const Matrix33 &matrix, FRAME_CONVENTION fc) {
    chrono::ChMatrix33<double> chronoMatrix = matrix;
    auto quaternion = chronoMatrix.Get_A_quaternion();
    if (IsNED(fc)) internal::SwapChQuaternionFrameConvention(quaternion);
    m_chronoQuaternion = quaternion;
  }

  void FrUnitQuaternion::SetNullRotation() {
    m_chronoQuaternion.SetUnit();
  }

  void FrUnitQuaternion::Normalize() {
    m_chronoQuaternion.Normalize();
  }

  double FrUnitQuaternion::Norm() const {
    return m_chronoQuaternion.Length();
  }

  bool FrUnitQuaternion::IsRotation() const {
    return (mathutils::IsClose<double>(Norm(), 1.));
  }

  void FrUnitQuaternion::Get(double &q0, double &q1, double &q2, double &q3, FRAME_CONVENTION fc) const {
    q0 = m_chronoQuaternion[0];
    q1 = m_chronoQuaternion[1];
    q2 = m_chronoQuaternion[2];
    q3 = m_chronoQuaternion[3];  // In NWU
    if (IsNED(fc))
      internal::SwapQuaternionElementsFrameConvention(q0, q1, q2,
                                                      q3); // Internal chrono quaternion is always in NWU convention
  }

  void FrUnitQuaternion::Get(Direction &axis, double &angleRAD, FRAME_CONVENTION fc) const {
    chrono::ChVector<double> chronoAxis;
    chrono::Q_to_AngAxis(m_chronoQuaternion, angleRAD, chronoAxis); // In NWU
    axis = chronoAxis.eigen(); // In NWU
    if (IsNED(fc)) axis = internal::SwapFrameConvention<Direction>(axis);
  }

  Direction FrUnitQuaternion::GetXAxis(FRAME_CONVENTION fc) const {
    auto quat = m_chronoQuaternion;
    if (IsNED(fc)) internal::SwapChQuaternionFrameConvention(quat);
    auto ch_axis = quat.GetXaxis();
    auto axis = ch_axis.eigen();
    return axis;
  }

  Direction FrUnitQuaternion::GetYAxis(FRAME_CONVENTION fc) const {
    auto quat = m_chronoQuaternion;
    if (IsNED(fc)) internal::SwapChQuaternionFrameConvention(quat);
    auto ch_axis = quat.GetYaxis();
    auto axis = ch_axis.eigen();
    return axis;
  }

  Direction FrUnitQuaternion::GetZAxis(FRAME_CONVENTION fc) const {
    auto quat = m_chronoQuaternion;
    if (IsNED(fc)) internal::SwapChQuaternionFrameConvention(quat);
    auto ch_axis = quat.GetZaxis();
    auto axis = ch_axis.eigen();
    return axis;
  }

  FrUnitQuaternion &FrUnitQuaternion::operator=(const FrUnitQuaternion &other) {
    this->m_chronoQuaternion = other.m_chronoQuaternion;
    return *this;
  }

  FrUnitQuaternion FrUnitQuaternion::operator*(const FrUnitQuaternion &other) const {
    auto newQuat = FrUnitQuaternion(*this);
    newQuat *= other;
    return newQuat;
  }

  FrUnitQuaternion &FrUnitQuaternion::operator*=(const FrUnitQuaternion &other) {

//        m_chronoQuaternion *= other.m_chronoQuaternion; // FIXME : this Chrono operator is currently buggy (07/11/2018)
    m_chronoQuaternion = m_chronoQuaternion * other.m_chronoQuaternion;
    // TODO : wait for a new release of Chrono for a fix to *= to use it... A message has been sent to the Chrono list by Lucas (07/11/2018)
    return *this;
  }

  bool FrUnitQuaternion::operator==(const frydom::FrUnitQuaternion &other) const {
    return m_chronoQuaternion == other.m_chronoQuaternion;
  }

  const chrono::ChQuaternion<double> &
  FrUnitQuaternion::GetChronoQuaternion() const {  // TODO : supprimer le besoin de cette methode
    return m_chronoQuaternion;
  }

  void FrUnitQuaternion::RotateInParent(const FrUnitQuaternion &leftQuaternion) {
    m_chronoQuaternion >>= leftQuaternion.m_chronoQuaternion;  // FIXME : VERIFIER !!!!

  }

  void FrUnitQuaternion::RotateInFrame(const FrUnitQuaternion &rightQuaternion) {
    m_chronoQuaternion *= rightQuaternion.m_chronoQuaternion;  // FIXME : VERIFIER !!!!
  }

  FrUnitQuaternion &FrUnitQuaternion::Inverse() {
    // We can use only m_chronoQuaternion.Conjugate() and not m_chronoQuaternion.Inverse()
    // because we know for sure our Unit quaternion is already normalized.
    m_chronoQuaternion.Conjugate();
    return *this;
  }

  FrUnitQuaternion FrUnitQuaternion::GetInverse() const {
    return FrUnitQuaternion(*this).Inverse();
  }

  std::ostream &FrUnitQuaternion::cout(std::ostream &os) const {

    os << std::endl;
    os << "Quaternion : q0 = " << m_chronoQuaternion.e0()
       << ", q1 = " << m_chronoQuaternion.e1()
       << ", q2 = " << m_chronoQuaternion.e2()
       << ", q3 = " << m_chronoQuaternion.e3()
       << std::endl;

    return os;
  }

  std::ostream &operator<<(std::ostream &os, const FrUnitQuaternion &quaternion) {
    return quaternion.cout(os);
  }

  Matrix33 FrUnitQuaternion::GetRotationMatrix() const {
    chrono::ChMatrix33<double> chronoMatrix;
    chronoMatrix.Set_A_quaternion(m_chronoQuaternion);
    return chronoMatrix;
  }

  Matrix33 FrUnitQuaternion::GetInverseRotationMatrix() const {
    chrono::ChMatrix33<double> chronoMatrix;
    chronoMatrix.Set_A_quaternion(m_chronoQuaternion.GetInverse());
    return chronoMatrix;
  }

  Matrix33 FrUnitQuaternion::LeftMultiply(const Matrix33 &matrix) const {
    return GetRotationMatrix() * matrix;
  }

  Matrix33 FrUnitQuaternion::RightMultiply(const Matrix33 &matrix) const {
    return matrix * GetRotationMatrix();
  }

  Matrix33 FrUnitQuaternion::LeftMultiplyInverse(const Matrix33 &matrix) const {
    return GetInverseRotationMatrix() * matrix;
  }

  Matrix33 FrUnitQuaternion::RightMultiplyInverse(const Matrix33 &matrix) const {
    return matrix * GetInverseRotationMatrix();
  }

  bool FrUnitQuaternion::IsApprox(const FrUnitQuaternion &other, double prec) const {
    return GetChronoQuaternion().Equals(other.GetChronoQuaternion(), prec);
  }

  bool FrUnitQuaternion::IsZero(double prec) const {
    return (*this == FrUnitQuaternion(1., 0., 0., 0., NWU));
  }

  /*
   *
   *
   * FrRotation
   *
   *
   */

  FrRotation::FrRotation() : m_frQuaternion() {}

  FrRotation::FrRotation(FrUnitQuaternion quaternion) : m_frQuaternion(quaternion) {}

  FrRotation::FrRotation(const Direction &axis, double angleRAD, FRAME_CONVENTION fc) :
      m_frQuaternion(axis, angleRAD, fc) {}

  FrRotation::FrRotation(const Direction &xaxis, const Direction &yaxis, const Direction &zaxis, FRAME_CONVENTION fc) {
    this->Set(xaxis, yaxis, zaxis, fc);
  }

  void FrRotation::SetNullRotation() {
    m_frQuaternion.SetNullRotation();
  }

  void FrRotation::Set(const FrUnitQuaternion &quat) {
    m_frQuaternion.Set(quat);
  }

  FrUnitQuaternion &FrRotation::GetQuaternion() {
    return m_frQuaternion;
  }

  const FrUnitQuaternion &FrRotation::GetQuaternion() const {
    return m_frQuaternion;
  }

  void FrRotation::SetAxisAngle(const Direction &axis, double angleRAD, FRAME_CONVENTION fc) {
    m_frQuaternion.Set(axis, angleRAD, fc);
  }

  void FrRotation::GetAxisAngle(Direction &axis, double &angleRAD, FRAME_CONVENTION fc) const {
    m_frQuaternion.Get(axis, angleRAD, fc);
  }

  Vector3d<double> FrRotation::GetRotationVector(FRAME_CONVENTION fc) const {
    Direction axis;
    double angle;
    GetAxisAngle(axis, angle, fc);
    return axis * angle;
  }

  void FrRotation::GetAxis(Direction &axis, FRAME_CONVENTION fc) {
    double angle = 0.;
    return GetAxisAngle(axis, angle, fc);
  }

  void FrRotation::GetAngle(double &angle) const {
    Direction axis;
    return GetAxisAngle(axis, angle, NWU);
  }

  double FrRotation::GetAngle() const {
    double angle;
    GetAngle(angle);
    return angle;
  }

  Matrix33 FrRotation::GetRotationMatrix() const {
    return m_frQuaternion.GetRotationMatrix();
  }

  Matrix33 FrRotation::GetInverseRotationMatrix() const {
    return m_frQuaternion.GetInverseRotationMatrix();
  }

  void FrRotation::Set(const Direction &xaxis, const Direction &yaxis, const Direction &zaxis, FRAME_CONVENTION fc) {

    // Verifying the directions are orthogonal
    assert(mathutils::IsClose<double>(xaxis.dot(yaxis), 0.));
    assert(mathutils::IsClose<double>(xaxis.dot(zaxis), 0.));
    assert(mathutils::IsClose<double>(yaxis.dot(zaxis), 0.));

    // Verifying the directions are unit vectors
    assert(xaxis.norm() - 1 == 0);
    assert(yaxis.norm() - 1 == 0);
    assert(zaxis.norm()  -1 == 0);

    // Verifying the directions form a direct frame
    assert((xaxis.cross(yaxis) - zaxis).isZero());

    Matrix33 matrix;
    matrix << xaxis.Getux(), yaxis.Getux(), zaxis.Getux(),
        xaxis.Getuy(), yaxis.Getuy(), zaxis.Getuy(),
        xaxis.Getuz(), yaxis.Getuz(), zaxis.Getuz();

    m_frQuaternion.Set(matrix, fc);
  }

  void FrRotation::SetXaxis(const Direction &xdir, const Direction &singular_dir, FRAME_CONVENTION fc) {

    Direction xdir_nwu = (IsNWU(fc)) ? xdir : internal::SwapFrameConvention(xdir);

    Direction xaxis, yaxis, zaxis;

    // set Vx.
    if (mathutils::IsClose(xdir_nwu.norm(), 0.)) {
      xaxis = {1., 0., 0.};
    } else {
      xaxis = xdir_nwu.normalized();
    }

    zaxis = xaxis.cross(singular_dir);
    double zlen = zaxis.norm();

    // if near singularity, change the singularity reference vector.
    if (zlen < 0.0001) {
      Direction singular_dir_tmp;

      if (fabs(singular_dir.Getux()) < 0.9)
        singular_dir_tmp = {1., 0., 0.};
      else if (fabs(singular_dir.Getuy()) < 0.9)
        singular_dir_tmp = {0., 1., 0.};
      else if (fabs(singular_dir.Getuz()) < 0.9)
        singular_dir_tmp = {0., 0., 1.};

      zaxis = xaxis.cross(singular_dir);
      zlen = zaxis.norm();  // now should be nonzero length.
    }

    // normalize Vz.
    zaxis *= 1 / zlen;

    // compute Vy.
    yaxis = zaxis.cross(xaxis);

    Set(xaxis, yaxis, zaxis, fc);

  }

  void
  FrRotation::SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {

    assert(seq == CARDAN || seq == XYZ);
    if (IsNED(fc)) {  // FIXME : la convention n'est valable que pour une sequence se terminant par yz...
      // Convert it to NWU convention
      theta = -theta;
      psi = -psi;
    }

    m_frQuaternion.Set(internal::Ch2FrQuaternion(internal::euler_to_quat(phi, theta, psi, seq)));

  }

  void
  FrRotation::SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) {
    SetEulerAngles_RADIANS(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD, seq, fc);
  }

  void FrRotation::SetCardanAngles_RADIANS(double phi, double theta, double psi, FRAME_CONVENTION fc) {
    SetEulerAngles_RADIANS(phi, theta, psi, EULER_SEQUENCE::CARDAN, fc);
  }

  void FrRotation::SetCardanAngles_DEGREES(double phi, double theta, double psi, FRAME_CONVENTION fc) {
    SetCardanAngles_RADIANS(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD, fc);
  }

  void FrRotation::GetEulerAngles_RADIANS(double &phi, double &theta, double &psi, EULER_SEQUENCE seq,
                                          FRAME_CONVENTION fc) const {
    // FIXME : voir a supprimer ce besoin de friend method !!!
    // Par exemple en ajoutant des fonctions ne prenant que des coefficients de quaternions dans EulerAngles.h
    auto vec = internal::quat_to_euler(m_frQuaternion.GetChronoQuaternion(),
                                       seq);  // In NWU  // TODO : avoir methode ne manipulat pas de ChVector
    phi = vec[0];
    theta = vec[1];
    psi = vec[2];

    if (IsNED(fc)) {  // FIXME : la convention n'est valable que pour une sequence se terminant par yz...
      // Convert it to NED convention
      theta = -theta;
      psi = -psi;
    }
  }

  void FrRotation::GetEulerAngles_DEGREES(double &phi, double &theta, double &psi, EULER_SEQUENCE seq,
                                          FRAME_CONVENTION fc) const {
    GetEulerAngles_RADIANS(phi, theta, psi, seq, fc);
    phi *= RAD2DEG;
    theta *= RAD2DEG;
    psi *= RAD2DEG;
  }

  void FrRotation::GetCardanAngles_RADIANS(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {
    GetEulerAngles_RADIANS(phi, theta, psi, EULER_SEQUENCE::CARDAN, fc);
  }

  void FrRotation::GetCardanAngles_DEGREES(double &phi, double &theta, double &psi, FRAME_CONVENTION fc) const {
    GetEulerAngles_DEGREES(phi, theta, psi, EULER_SEQUENCE::CARDAN, fc);
  }

  void FrRotation::GetFixedAxisAngles_RADIANS(double &rx, double &ry, double &rz, FRAME_CONVENTION fc) const {
    // TODO : utiliser
  }

  void FrRotation::GetFixedAxisAngles_DEGREES(double &rx, double &ry, double &rz, FRAME_CONVENTION fc) const {
    // TODO
  }

  FrRotation &FrRotation::operator=(const FrRotation &other) {
    this->m_frQuaternion = other.m_frQuaternion;
    return *this;
  }

  FrRotation FrRotation::operator*(const FrRotation &other) const {
    return FrRotation(m_frQuaternion * other.m_frQuaternion);
  }

  FrRotation &FrRotation::operator*=(const FrRotation &other) {
    this->m_frQuaternion *= other.m_frQuaternion;
    return *this;
  }

  bool FrRotation::operator==(const FrRotation &other) const {
    return m_frQuaternion == other.m_frQuaternion;
  }

  Matrix33 FrRotation::LeftMultiply(const Matrix33 &matrix) const {
    return GetRotationMatrix() * matrix;
  }

  Matrix33 FrRotation::LeftMultiplyInverse(const Matrix33 &matrix) const {
    return GetInverseRotationMatrix() * matrix;
  }

  Matrix33 FrRotation::RightMultiply(const Matrix33 &matrix) const {
    return matrix * GetRotationMatrix();
  }

  Matrix33 FrRotation::RightMultiplyInverse(const Matrix33 &matrix) const {
    return matrix * GetInverseRotationMatrix();
  }

  FrRotation &FrRotation::RotAxisAngle_RADIANS(const Direction &axis, double angle, FRAME_CONVENTION fc) {
    *this *= FrRotation(axis, angle, fc);
    return *this;
  }

  FrRotation &FrRotation::RotAxisAngle_DEGREES(const Direction &axis, double angle, FRAME_CONVENTION fc) {
    return RotAxisAngle_RADIANS(axis, angle * DEG2RAD, fc);
  }

  FrRotation &FrRotation::RotX_RADIANS(double angle, FRAME_CONVENTION fc) {
    return RotAxisAngle_RADIANS(Direction(1., 0., 0.), angle, fc);
  }

  FrRotation &FrRotation::RotX_DEGREES(double angle, FRAME_CONVENTION fc) {
    return RotX_RADIANS(angle * DEG2RAD, fc);
  }

  FrRotation &FrRotation::RotY_RADIANS(double angle, FRAME_CONVENTION fc) {
    return RotAxisAngle_RADIANS(Direction(0., 1., 0.), angle, fc);
  }

  FrRotation &FrRotation::RotY_DEGREES(double angle, FRAME_CONVENTION fc) {
    return RotY_RADIANS(angle * DEG2RAD, fc);
  }

  FrRotation &FrRotation::RotZ_RADIANS(double angle, FRAME_CONVENTION fc) {
    return RotAxisAngle_RADIANS(Direction(0., 0., 1.), angle, fc);
  }

  FrRotation &FrRotation::RotZ_DEGREES(double angle, FRAME_CONVENTION fc) {
    return RotZ_RADIANS(angle * DEG2RAD, fc);
  }

  void FrRotation::RotateInParent(const FrUnitQuaternion &leftQuaternion) {
    m_frQuaternion.RotateInParent(leftQuaternion);
  }

  void FrRotation::RotateInFrame(const FrUnitQuaternion &rightQuaternion) {
    m_frQuaternion.RotateInFrame(rightQuaternion);
  }

  void FrRotation::RotateInParent(const FrRotation &leftRotation) {
    RotateInParent(leftRotation.GetQuaternion());
  }

  void FrRotation::RotateInFrame(const FrRotation &rightRotation) {
    RotateInFrame(rightRotation.GetQuaternion());
  }

  Direction FrRotation::GetXAxis(FRAME_CONVENTION fc) const {
    return m_frQuaternion.GetXAxis(fc);
  }

  Direction FrRotation::GetYAxis(FRAME_CONVENTION fc) const {
    return m_frQuaternion.GetYAxis(fc);
  }

  Direction FrRotation::GetZAxis(FRAME_CONVENTION fc) const {
    return m_frQuaternion.GetZAxis(fc);
  }

  std::ostream &FrRotation::cout(std::ostream &os) const {

    double phi, theta, psi;
    GetCardanAngles_DEGREES(phi, theta, psi, NWU);

    os << std::endl;
    os << "Rotation (cardan angles in deg, NWU convention) :\n";
    os << "Cardan angle (deg) : ";
    os << "phi = " << phi;
    os << "; theta = " << theta;
    os << "; psi = " << psi;
    os << std::endl;

    return os;
  }

  std::ostream &operator<<(std::ostream &os, const FrRotation &rotation) {
    return rotation.cout(os);
  }

}  // end namespace frydom
