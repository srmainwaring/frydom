//
// Created by frongere on 16/04/2020.
//

#ifndef FRYDOM_FRCABLEPROPERTIES_H
#define FRYDOM_FRCABLEPROPERTIES_H

#include <memory>


namespace frydom {

  class FrCableProperties {

   private:

    // Cable properties
    // FIXME: mettre des valeurs par defaut non verolees !!!
    double m_section;                    ///< Section area of the cable, in m²
    double m_youngModulus;          ///< Young modulus of the cable, in Pa
    double m_linearDensity;           ///< Linear density of the cable, in kg/m

    double m_hydroDiameter;

    double m_transverseDragCoefficient;
    double m_tangentialDragCoefficient;
    double m_transverseAddedMassCoefficient;
    double m_tangentialAddedMassCoefficient;

    double m_rayleighDamping;               ///< Rayleigh damping of the cable (for dynamic cable only)
    double m_breakingTension;               ///< breaking tension, in N (for visualization purpose for now)

   public:

    /// Default constructor
    FrCableProperties() :
        m_section(0.),
        m_youngModulus(0.),
        m_linearDensity(0.),
        m_hydroDiameter(0.),
        m_transverseDragCoefficient(0.),
        m_tangentialDragCoefficient(0.),
        m_transverseAddedMassCoefficient(0.),
        m_tangentialAddedMassCoefficient(0.),
        m_rayleighDamping(0.),
        m_breakingTension(0.) {

    };

    /// Cable properties constructor from Young modulus, diameter and linear density
    /// \param diameter diameter of the cable, in m
    /// \param linearDensity Linear density of the cable, in kg/m
    /// \param youngModulus Young modulus of the cable, in Pa
    /// \param elastic Is the cable elastic (for catenary lines only)
    FrCableProperties(double diameter, double linearDensity, double youngModulus);

    FrCableProperties(double diameter, double linearDensity, double youngModulus, double RayleighDamping);

    // cable properties accessors
    ///Set the Young modulus of the cable
    /// \param E Young modulus
    void SetYoungModulus(double E);

    /// Get the Young modulus of the cable
    /// \return Young modulus
    double GetYoungModulus() const;

    /// Set the section area of the cable
    /// \param A section area
    void SetSectionArea(double A);

    /// Get the section area of the cable
    /// \return section area
    double GetSectionArea() const;

    /// Set the section area through the diameter of the cable
    /// \param d diameter
    void SetDiameter(double d);

    /// Get the diameter of the cable
    /// \return diameter
    double GetDiameter() const;

    /// Get the radius of the cable
    /// \return radius
    double GetRadius() const;

    void SetEA(double EA);

    /// Get the product of the Young modulus and the section area
    /// \return product of the Young modulus and the section area
    double GetEA() const;

    /// Set the linear density of the cable
    /// \param lambda linear density
    void SetLinearDensity(double lambda);

    /// Get the linear density of the cable
    /// \return linear density
    double GetLinearDensity() const;

    /// Set the density of the cable (lambda = A.rho)
    /// \param rho density
    void SetDensity(double rho);

    /// Get the density of the cable
    /// \return density
    double GetDensity() const;

    /// Set the breeaking tension of the cable
    /// \param breakingTension Breaking tension value
    void SetBreakingTension(double breakingTension);

    /// Return the breaking tension of the cable
    double GetBreakingTension() const;

    void SetRayleighDamping(double damping) { m_rayleighDamping = damping; }

    double GetRayleighDamping() const;

    void SetHydrodynamicDiameter(double d) { m_hydroDiameter = d; }

    double GetHydrodynamicDiameter() const { return m_hydroDiameter; }

    void SetDragCoefficients(double transverse_drag_coeff, double tangential_drag_coeff) {
      m_transverseDragCoefficient = transverse_drag_coeff;
      m_tangentialDragCoefficient = tangential_drag_coeff;
    }

    inline double GetTransverseDragCoefficient() const { return m_transverseDragCoefficient; }

    inline double GetTangentialDragCoefficient() const { return m_tangentialDragCoefficient; }

    void SetAddedMassCoefficients(double transverse_added_mass_coeff, double tangential_added_mass_coeff) {
      m_transverseAddedMassCoefficient = transverse_added_mass_coeff;
      m_tangentialAddedMassCoefficient = tangential_added_mass_coeff;
    }

    inline double GetTransverseAddedMassCoefficient() const { return m_transverseAddedMassCoefficient; }

    inline double GetTangentialAddedMassCoefficient() const { return m_tangentialAddedMassCoefficient; }


  };

  std::shared_ptr<FrCableProperties> make_cable_properties();

  std::shared_ptr<FrCableProperties>
  make_cable_properties(double diameter, double linearDensity, double youngModulus);

  std::shared_ptr<FrCableProperties>
  make_cable_properties(double diameter, double linearDensity, double youngModulus, double rayleighDamping);

}  // end namespace frydom



#endif //FRYDOM_FRCABLEPROPERTIES_H
