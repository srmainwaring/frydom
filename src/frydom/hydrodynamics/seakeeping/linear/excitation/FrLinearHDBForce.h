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


#ifndef FRYDOM_FRLINEARHDBFORCE_H
#define FRYDOM_FRLINEARHDBFORCE_H

#include "frydom/core/math/FrVector.h"
#include "MathUtils/Interp1d.h"
#include "frydom/core/force/FrForce.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"

namespace frydom {

  // Forward declarations.
  class FrHydroDB;

  /**
   * \class FrLinearHDBForce
   * \brief Virtual class for defining a hydrodynamic linear model force :
   * see FrLinearDiffractionForce, FrLinearExcitationForce, FrLinearFroudeKrylovForce, for derived implementations.
   */
  class FrLinearHDBForce : public FrForce {

   protected:

    /// Interpolator in waves frequencies and directions.
    std::vector<std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators;
    std::vector<std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators_forward_speed;

    /// Hydrodynamic database.
    std::shared_ptr<FrHydroDB> m_HDB;

    /// Hydrodynamic components.
    std::vector<Eigen::MatrixXcd> m_Fhdb; // Zero-speed component.
//    std::vector<Eigen::MatrixXcd> m_Fhdb_forward_speed; // Forward-speed correction.

   public:

    /// Constructor.
    FrLinearHDBForce(const std::string &name,
                     const std::string &type_name,
                     FrBody *body,
                     const std::shared_ptr<FrHydroDB> &HDB);

    /// Getter for the Froude-Krylov, diffraction or exciting loads for a given wave direction.
    virtual Eigen::MatrixXcd GetHDBData(unsigned int iangle) const = 0;

    /// Getter for the x-derivative of the Froude-Krylov, diffraction or exciting loads for a given wave direction.
    virtual Eigen::MatrixXcd GetHDBDataXDerivative(unsigned int iangle) const = 0;

    /// Getter for the Froude-Krylov, diffraction or exciting loads for a given wave direction and dof.
    virtual Eigen::VectorXcd GetHDBData(unsigned int iangle, unsigned iforce) const = 0;

    /// Getter for the x-derivative of the Froude-Krylov, diffraction or exciting loads for a given wave direction and dof.
    virtual Eigen::VectorXcd GetHDBDataXDerivative(unsigned int iangle, unsigned iforce) const = 0;

    /// This method creates the interpolator for the excitation loads (Froude-Krylov or diffraction) with respect to the wave frequencies and wave directions.
    virtual void BuildHDBInterpolators();

//    /// This method creates the interpolator for the x-derivatives of the excitation loads (Froude-Krylov or diffraction) with respect to the wave frequencies and wave directions.
//    virtual void BuildHDBInterpolatorsXDerivative();

    /// This method returns the Froude-Krylov, diffraction or excitation loads from the interpolator.
    std::vector<Eigen::MatrixXcd> GetHDBInterp(std::vector<double> waveFrequencies, std::vector<double> waveDirections);

    /// This method returns the x-derivative of the Froude-Krylov, diffraction or excitation loads from the interpolator.
    std::vector<Eigen::MatrixXcd> GetHDBInterpXDerivative(std::vector<double> waveFrequencies, std::vector<double> waveDirections);

    void Initialize() override;

   protected:

    void Compute(double time) override;

    /// This function computes the excitation loads (linear excitation) or the diffraction loads (nonlinear excitation).
    void Compute_F_HDB();

    FrMask GetBodyMask() const;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };


} // end namespace frydom

#endif //FRYDOM_FRLINEARHDBFORCE_H
