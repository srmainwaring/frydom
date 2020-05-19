//
// Created by frongere on 16/04/2020.
//

#include <cfloat>
#include <cassert>

#include "MathUtils/Constants.h"

#include "FrCableProperties.h"


namespace frydom {

  FrCableProperties::FrCableProperties(double diameter, double linearDensity, double youngModulus) :
      m_youngModulus(youngModulus), m_linearDensity(linearDensity) {
    SetDiameter(diameter);
  }

  FrCableProperties::FrCableProperties(double diameter, double linearDensity, double youngModulus,
                                       double rayleighDamping) :
      FrCableProperties(diameter, linearDensity, youngModulus) {
    m_rayleighDamping = rayleighDamping;
  }

  void FrCableProperties::SetYoungModulus(double E) {
    m_youngModulus = E;
  }

  double FrCableProperties::GetYoungModulus() const {
    return m_youngModulus;
  }

  void FrCableProperties::SetSectionArea(double A) {
    m_section = A;
  }

  double FrCableProperties::GetSectionArea() const {
    return m_section;
  }

  void FrCableProperties::SetDiameter(double d) {
    m_section = 0.25 * MU_PI * d * d;
  }

  double FrCableProperties::GetDiameter() const {
    return sqrt(4. * m_section / MU_PI);
  }

  double FrCableProperties::GetRadius() const {
    return 0.5 * GetDiameter();
  }

  void FrCableProperties::SetEA(double EA) {
    m_youngModulus = EA / m_section;
  }

  double FrCableProperties::GetEA() const {
    return m_youngModulus * m_section;
  }

  void FrCableProperties::SetLinearDensity(double lambda) {
    m_linearDensity = lambda;
  }

  double FrCableProperties::GetLinearDensity() const {
    return m_linearDensity;
  }

  void FrCableProperties::SetDensity(double rho) {
    m_linearDensity = rho * m_section;
  }

  double FrCableProperties::GetDensity() const {
    return m_linearDensity / m_section;
  }

  void FrCableProperties::SetBreakingTension(double breakingTension) {
    assert(breakingTension > DBL_EPSILON);
    m_breakingTension = breakingTension;
  }

  double FrCableProperties::GetBreakingTension() const {
    return m_breakingTension;
  }

  double FrCableProperties::GetRayleighDamping() const {
    return m_rayleighDamping;
  }

  void FrCableProperties::SetRayleighDamping(double damping) { m_rayleighDamping = damping; }

  void FrCableProperties::SetHydrodynamicDiameter(double d) { m_hydroDiameter = d; }

  double FrCableProperties::GetHydrodynamicDiameter() const { return m_hydroDiameter; }

  void FrCableProperties::SetDragCoefficients(double transverse_drag_coeff, double tangential_drag_coeff) {
    m_transverseDragCoefficient = transverse_drag_coeff;
    m_tangentialDragCoefficient = tangential_drag_coeff;
  }

  double FrCableProperties::GetTransverseDragCoefficient() const { return m_transverseDragCoefficient; }

  double FrCableProperties::GetTangentialDragCoefficient() const { return m_tangentialDragCoefficient; }

  void
  FrCableProperties::SetAddedMassCoefficients(double transverse_added_mass_coeff, double tangential_added_mass_coeff) {
    m_transverseAddedMassCoefficient = transverse_added_mass_coeff;
    m_tangentialAddedMassCoefficient = tangential_added_mass_coeff;
  }

  double FrCableProperties::GetTransverseAddedMassCoefficient() const { return m_transverseAddedMassCoefficient; }

  double FrCableProperties::GetTangentialAddedMassCoefficient() const { return m_tangentialAddedMassCoefficient; }

  void FrCableProperties::SetVIVAmpFactor(const double &G) { m_VIVAmpFactor = G; }

  double FrCableProperties::GetVIVAmpFactor() const { return m_VIVAmpFactor; }


  std::shared_ptr<FrCableProperties> make_cable_properties() {
    return std::make_shared<FrCableProperties>();
  }

  std::shared_ptr<FrCableProperties> make_cable_properties(double diameter, double linearDensity, double youngModulus) {
    return std::make_shared<FrCableProperties>(diameter, linearDensity, youngModulus);
  }

  std::shared_ptr<FrCableProperties> make_cable_properties(double diameter,
                                                           double linearDensity,
                                                           double youngModulus,
                                                           double rayleighDamping) {
    return std::make_shared<FrCableProperties>(diameter, linearDensity, youngModulus, rayleighDamping);
  }

}  // end namespace frydom
