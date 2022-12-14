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

#include "FrLinearHDBForce.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"

namespace frydom {

  void FrLinearHDBForce::Initialize() {

    auto body = GetBody();

    // Wave field.
    auto waveField = body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

    // BEMBody.
    auto BEMBody = m_HDB->GetBody(body);

    // Interpolation of the excitation loads with respect to the wave direction.
    BuildHDBInterpolators();
//    if(m_HDB->GetIsXDerivative()) {
//      BuildHDBInterpolatorsXDerivative();
//    }

    // Frequency and wave direction discretization.
    auto freqs = waveField->GetWaveFrequencies(RADS);
    auto directions = waveField->GetWaveDirections(RAD, NWU, GOTO);

    // Interpolation of the exciting loads if not already done.
    if (m_Fhdb.empty()) {
      m_Fhdb = GetHDBInterp(freqs, directions);
//      if(m_HDB->GetIsXDerivative() and c_FScorrection_simple_model and c_FScorrection_extended_model) {
//        m_Fhdb_forward_speed = GetHDBInterpXDerivative(freqs, directions);
//      }
    }

    // Initialization of the parent class.
    FrForce::Initialize();

  }

  std::vector<Eigen::MatrixXcd>
  FrLinearHDBForce::GetHDBInterp(std::vector<double> waveFrequencies, std::vector<double> waveDirections) {

    // This method returns the Froude-Krylov, diffraction or excitation loads from the interpolator.

    // BEMBody.
    auto BEMBody = m_HDB->GetBody(GetBody());

    // --> Getting sizes

    auto nbFreqInterp = waveFrequencies.size();
    auto nbFreqBDD = m_HDB->GetFrequencyDiscretization().size();// GetNbFrequencies();
    auto nbDirInterp = waveDirections.size();
    auto nbForceDOFs = GetBodyMask().GetNbDOF(); //->GetNbForceMode();

    // Wave direction is expressed between 0 and 2*pi.
    for (auto &dir : waveDirections) dir = mathutils::Normalize_0_2PI(dir);

    std::vector<Eigen::MatrixXcd> Fexc;
    Fexc.reserve(nbDirInterp);

    // -> Building interpolator and return vector

//    auto freqsBDD = std::make_shared<std::vector<double>>(BEMBody->GetFrequencies());
    auto frequencies = m_HDB->GetFrequencyDiscretization();
    std::vector<double> vec(frequencies.data(), frequencies.data() + frequencies.rows() * frequencies.cols());
    auto freqsBDD = std::make_shared<std::vector<double>>(vec);

    auto freqCoeffs = std::make_shared<std::vector<std::complex<double>>>();
    freqCoeffs->reserve(nbFreqBDD);

    for (auto direction: waveDirections) {

      auto excitationForceDir = Eigen::MatrixXcd(nbForceDOFs, nbFreqInterp);

      for (unsigned int idof = 0; idof < nbForceDOFs; ++idof) {

        freqCoeffs->clear();
        for (unsigned int ifreq = 0; ifreq < nbFreqBDD; ++ifreq) {
          freqCoeffs->push_back(m_waveDirInterpolators[idof][ifreq](direction));
        }

        auto freqInterpolator = mathutils::Interp1dLinear<double, std::complex<double>>();
        freqInterpolator.Initialize(freqsBDD, freqCoeffs);

        auto freqCoeffsInterp = freqInterpolator(waveFrequencies);
        for (unsigned int ifreq = 0; ifreq < nbFreqInterp; ++ifreq) {
          excitationForceDir(idof, ifreq) = freqCoeffsInterp[ifreq];
        }
      }
      Fexc.push_back(excitationForceDir);
    }
    return Fexc;
  }

  std::vector<Eigen::MatrixXcd>
  FrLinearHDBForce::GetHDBInterpXDerivative(std::vector<double> waveFrequencies, std::vector<double> waveDirections) {

    // This method returns the x-derivative of the Froude-Krylov, diffraction or excitation loads from the interpolator.

    // BEMBody.
    auto BEMBody = m_HDB->GetBody(GetBody());

    // --> Getting sizes

    auto nbFreqInterp = waveFrequencies.size();
    auto nbFreqBDD = m_HDB->GetFrequencyDiscretization().size();// GetNbFrequencies();
    auto nbDirInterp = waveDirections.size();
    auto nbForceDOFs = GetBodyMask().GetNbDOF(); //->GetNbForceMode();

    // Wave direction is expressed between 0 and 2*pi.
    for (auto &dir : waveDirections) dir = mathutils::Normalize_0_2PI(dir);

    std::vector<Eigen::MatrixXcd> Fexc;
    Fexc.reserve(nbDirInterp);

    // -> Building interpolator and return vector

//    auto freqsBDD = std::make_shared<std::vector<double>>(BEMBody->GetFrequencies());
    auto frequencies = m_HDB->GetFrequencyDiscretization();
    std::vector<double> vec(frequencies.data(), frequencies.data() + frequencies.rows() * frequencies.cols());
    auto freqsBDD = std::make_shared<std::vector<double>>(vec);

    auto freqCoeffs = std::make_shared<std::vector<std::complex<double>>>();
    freqCoeffs->reserve(nbFreqBDD);

    for (auto direction: waveDirections) {

      auto excitationForceDir = Eigen::MatrixXcd(nbForceDOFs, nbFreqInterp);

      for (unsigned int idof = 0; idof < nbForceDOFs; ++idof) {

        freqCoeffs->clear();
        for (unsigned int ifreq = 0; ifreq < nbFreqBDD; ++ifreq) {
          freqCoeffs->push_back(m_waveDirInterpolators_forward_speed[idof][ifreq](direction));
        }

        auto freqInterpolator = mathutils::Interp1dLinear<double, std::complex<double>>();
        freqInterpolator.Initialize(freqsBDD, freqCoeffs);

        auto freqCoeffsInterp = freqInterpolator(waveFrequencies);
        for (unsigned int ifreq = 0; ifreq < nbFreqInterp; ++ifreq) {
          excitationForceDir(idof, ifreq) = freqCoeffsInterp[ifreq];
        }
      }
      Fexc.push_back(excitationForceDir);
    }
    return Fexc;
  }

  void FrLinearHDBForce::BuildHDBInterpolators() {

    // This method creates the interpolator for the excitation loads (Froude-Krylov or diffraction) with respect to the wave frequencies and wave directions.

    // BEMBody.
    auto BEMBody = m_HDB->GetBody(GetBody());

    // Parameters.
    auto nbWaveDirections = m_HDB->GetWaveDirectionDiscretization().size(); //BEMBody->GetNbWaveDirections();
    auto nbFreq = m_HDB->GetFrequencyDiscretization().size(); //BEMBody->GetNbFrequencies();
    auto nbForceDOFs = GetBodyMask().GetNbDOF(); //BEMBody->GetNbForceMode();

    // Initialization.
    m_waveDirInterpolators.clear();
    m_waveDirInterpolators.reserve(nbForceDOFs);

    auto waveDirections = m_HDB->GetWaveDirectionDiscretization();
    std::vector<double> vec(waveDirections.data(), waveDirections.data() + waveDirections.rows() * waveDirections.cols());
    auto angles = std::make_shared<std::vector<double>>(vec);

    auto interpolators = std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>();
    interpolators.reserve(nbFreq);

    for (unsigned int idof = 0; idof < nbForceDOFs; ++idof) {

      interpolators.clear();

      for (unsigned int ifreq = 0; ifreq < nbFreq; ++ifreq) {

        auto coeffs = std::make_shared<std::vector<std::complex<double>>>();
        coeffs->reserve(nbWaveDirections);

        for (unsigned int iangle = 0; iangle < nbWaveDirections; ++iangle) {
          auto data = GetHDBData(iangle);
          coeffs->push_back(data(GetBodyMask().GetDOFs()[idof].GetIndex(), ifreq));
        }

        auto interpolator = mathutils::Interp1dLinear<double, std::complex<double>>();
        interpolator.Initialize(angles, coeffs);
        interpolators.push_back(interpolator);
      }
      m_waveDirInterpolators.push_back(interpolators);
    }
  }

//  void FrLinearHDBForce::BuildHDBInterpolatorsXDerivative() {
//
//    // This method creates the interpolator for the x-derivatives of the excitation loads (Froude-Krylov or diffraction) with respect to the wave frequencies and wave directions.
//
//    // BEMBody.
//    auto BEMBody = m_HDB->GetBody(GetBody());
//
//    // Parameters.
//    auto nbWaveDirections = m_HDB->GetWaveDirectionDiscretization().size(); //BEMBody->GetNbWaveDirections();
//    auto nbFreq = m_HDB->GetFrequencyDiscretization().size(); //BEMBody->GetNbFrequencies();
//    auto nbForceDOFs = GetBodyMask().GetNbDOF(); //BEMBody->GetNbForceMode();
//
//    // Initialization.
//    m_waveDirInterpolators_forward_speed.clear();
//    m_waveDirInterpolators_forward_speed.reserve(nbForceDOFs);
//
//    auto waveDirections = m_HDB->GetWaveDirectionDiscretization();
//    std::vector<double> vec(waveDirections.data(), waveDirections.data() + waveDirections.rows() * waveDirections.cols());
//    auto angles = std::make_shared<std::vector<double>>(vec);
//
//    auto interpolators = std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>();
//    interpolators.reserve(nbFreq);
//
//    for (unsigned int idof = 0; idof < nbForceDOFs; ++idof) {
//
//      interpolators.clear();
//
//      for (unsigned int ifreq = 0; ifreq < nbFreq; ++ifreq) {
//
//        auto coeffs = std::make_shared<std::vector<std::complex<double>>>();
//        coeffs->reserve(nbWaveDirections);
//
//        for (unsigned int iangle = 0; iangle < nbWaveDirections; ++iangle) {
//          auto data = GetHDBDataXDerivative(iangle);
//          coeffs->push_back(data(GetBodyMask().GetDOFs()[idof].GetIndex(), ifreq));
//        }
//
//        auto interpolator = mathutils::Interp1dLinear<double, std::complex<double>>();
//        interpolator.Initialize(angles, coeffs);
//        interpolators.push_back(interpolator);
//      }
//      m_waveDirInterpolators_forward_speed.push_back(interpolators);
//    }
//  }

  void FrLinearHDBForce::Compute_F_HDB() {

    auto body = GetBody();

    // This function computes the excitation loads (linear excitation) or the diffraction loads (nonlinear excitation).

    // Equilibrium frame.
    auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(body);

    // Forward speed.
    auto meanSpeed = eqFrame->GetFrameVelocityInFrame(NWU);

    // Wave field structure.
    auto waveField = body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

    // Wave elevation.
    auto complexElevations = waveField->GetComplexElevation(eqFrame->GetFrame().GetX(NWU),
                                                            eqFrame->GetFrame().GetY(NWU),
                                                            NWU);

    // Number of wave frequencies.
    auto nbFreq = waveField->GetWaveFrequencies(RADS).size();

    // Number of wave directions.
    auto nbWaveDir = waveField->GetWaveDirections(RAD, NWU, GOTO).size();

    // Fexc(t) = eta*Fexc(Helios).

    // From vector to force and torque structures.
    Force force;
    force.setZero();
    Torque torque;
    torque.setZero();

    unsigned int idof = 0;
    for (auto &dof:GetBodyMask().GetDOFs()) {
      double tempforce = 0;
      for (unsigned int ifreq = 0; ifreq < nbFreq; ++ifreq) {
        for (unsigned int idir = 0; idir < nbWaveDir; ++idir) {
//          if(m_HDB->GetIsXDerivative() and c_FScorrection_simple_model) {
//            auto omega = waveField->GetWaveFrequencies(RADS).at(ifreq);
//            tempforce += std::imag(complexElevations[idir][ifreq] * (m_Fhdb[idir](idof, ifreq)
//                + (meanSpeed.norm() / (JJ * omega)) * m_Fhdb_forward_speed[idir](idof, ifreq)));
//            tempforce += std::imag(complexElevatextended_forward_speed_modelions[idir][ifreq] * (m_Fhdb[idir](idof, ifreq)
//                - (meanSpeed.norm() / (JJ * omega)) * m_Fhdb_forward_speed[idir](idof, ifreq))); // Minus sign due to transformation from the body frame to the world frame.
//          } else {
          tempforce += std::imag(complexElevations[idir][ifreq] * m_Fhdb[idir](idof, ifreq));
//          }
        }
      }
      Direction direction = dof.GetDirection();
      switch (dof.GetType()) {
        case hdb5_io::DOF::LINEAR:
          force += direction * tempforce;
          break;
        case hdb5_io::DOF::ANGULAR:
          torque += direction * tempforce;
          break;
      }
      idof++;
    }

    // Projection of the loads in the equilibrium frame.
    auto forceInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(force, NWU);
    auto torqueInWorldAtCOG = eqFrame->GetFrame().ProjectVectorFrameInParent(torque, NWU);

    // Setting the nonlinear excitation loads in world at the CoG in world.
    SetForceTorqueInWorldAtCOG(forceInWorld, torqueInWorldAtCOG, NWU);

  }

  void FrLinearHDBForce::Compute(double time) {
    Compute_F_HDB();
  }

  FrLinearHDBForce::FrLinearHDBForce(const std::string &name,
                                     const std::string &type_name,
                                     FrBody *body,
                                     const std::shared_ptr<FrHydroDB> &HDB) :
      FrForce(name, type_name, body),
      m_HDB(HDB) {}

  FrMask FrLinearHDBForce::GetBodyMask() const {
    return m_HDB->GetBodyDOFMask(m_HDB->GetBody(GetBody()));
  }


} // end namespace frydom
