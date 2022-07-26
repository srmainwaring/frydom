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

#include "FrRadiationConvolutionModel.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/logging/FrEventLogger.h"
#include "FrRadiationForce.h"

namespace frydom {

  Vector6d<double> TrapzLoc(std::vector<double> &x, std::vector<Vector6d<double>> &y) {

    // TODO : generaliser la méthode de mathutils pour les vecteurs Eigen

    unsigned long N = y.size();

    //assert(N > 1);
    if (N <= 1) {
      auto res = Vector6d<double>();
      res.SetNull();
      return res;
    }

    assert(x.size() == N);

    double dx1 = x[1] - x[0];
    double dxN_1 = x[N - 1] - x[N - 2];

    Vector6d<double> sum;
    sum.SetNull();
    double dxi, dxii;

    dxii = dx1;

    for (unsigned long i = 1; i < N - 1; i++) {
      dxi = dxii;
      dxii = x[i + 1] - x[i];
      sum += y[i] * (dxi + dxii);
    }

    return 0.5 * (sum + y[0] * dx1 + y[N - 1] * dxN_1);

  }

  FrRadiationConvolutionModel::FrRadiationConvolutionModel(const std::string &name,
                                                           FrOffshoreSystem *system,
                                                           std::shared_ptr<FrHydroDB> HDB) :
      FrRadiationModel(name, system, HDB) {

    /// Construtor of the class.

    // Loop over every body subject to hydrodynamic loads.
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      auto body = m_HDB->GetBody(BEMBody->first);
      body->AddExternalForce(std::make_shared<FrRadiationConvolutionForce>("radiation_force", body, this));
    }
  }

  void FrRadiationConvolutionModel::Initialize() {

    FrRadiationModel::Initialize();

    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      // Check if the recorder was instanciated for this BEM body.
      if (m_recorder.find(BEMBody->first) == m_recorder.end()) {
        auto interpK = BEMBody->first->GetIRFInterpolator("K");
        auto Te = interpK->at(0)->GetXmax(BEMBody->first->GetName());
        auto dt = GetParent()->GetTimeStep();
        m_recorder[BEMBody->first] = FrTimeRecorder<GeneralizedVelocity>(Te, dt);
      }
      m_recorder[BEMBody->first].Initialize();
    }
  }

  void FrRadiationConvolutionModel::Clear() {
    for (auto &BEMBody : *m_HDB) {
      m_recorder[BEMBody.first].Clear();
    }
  }

  void FrRadiationConvolutionModel::Compute(double time) {

    // Computation of the radiation loads only once per time step.
    if (std::abs(time - GetSystem()->GetTime()) < 0.1 * GetSystem()->GetTimeStep() and
        time > FLT_EPSILON)
      return;

    // Update speed recorder
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); BEMBody++) {
      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      m_recorder[BEMBody->first].Record(time, eqFrame->GetPerturbationGeneralizedVelocityInFrame(NWU));
    }

    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      auto radiationForce = GeneralizedForce();
      radiationForce.setZero();

      for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

        auto radiationMask = m_HDB->GetBodyRadiationMask(BEMBody->first, BEMBodyMotion->first);
//        auto radiationMask = BEMBody->first->GetRadiationMask(BEMBodyMotion->first);
//        // Applying the BEMBody DOFMask also on the radiationMask to ensure all forces on locked dofs are zero
//        hdb5_io::Mask DOFMask;
//        DOFMask.SetMask(m_HDB->GetBody(BEMBody->first)->GetDOFMask()->GetLockedDOFs());
//        for (auto idof : DOFMask.GetListDOF()) {
//          radiationMask.row(idof) *= false;
//        }

        auto velocity = m_recorder[BEMBodyMotion->first].GetData();

        auto vtime = m_recorder[BEMBodyMotion->first].GetTime();

        auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(BEMBodyMotion->first);

        for (auto idof : BodyMotionDOFMask.GetListDOF()) {

          //idof applied here to BEMBodyMotion, even if it's not really explicit. The BEMBodyMotion is now called at the
          // Eval below. So it's equivalent to the next line, previously written for the old container.
          auto interpK = BEMBody->first->GetIRFInterpolator("K")->at(idof);

          std::vector<mathutils::Vector6d<double>> kernel;
          kernel.reserve(vtime.size());
          for (unsigned int it = 0; it < vtime.size(); ++it) {
            auto irf = interpK->Eval(BEMBodyMotion->first->GetName(), vtime[it]) * velocity.at(it)(idof);
            // radiation mask applied
            kernel.push_back(irf.cwiseProduct(radiationMask.col(idof).cast<double>()));
          }
          radiationForce += TrapzLoc(vtime, kernel);
        }
      }

      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      auto forceInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetForce(), NWU);
      auto TorqueInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetTorque(), NWU);

      // Application of the minus sign.
      m_radiationForce[BEMBody->first] = -GeneralizedForce(forceInWorld, TorqueInWorld);
    }
  }

  void FrRadiationConvolutionModel::StepFinalize() {
    // Serialize and send the message log
//        FrObject::SendLog();
  }

  void FrRadiationConvolutionModel::SetImpulseResponseSize(FrBEMBody *BEMBody, double Te, double dt) {
    //TODO : check it is not already instanciated
    auto Te_IRF = BEMBody->GetIRFInterpolator("K")->at(0)->GetXmax(BEMBody->GetName());
    if (Te > Te_IRF) {
      event_logger::error("FrRadiationConvolutionModel", GetName(),
                          fmt::format("SetImpulseResponseSize specified Te = {} is larger than the IRF final time = {}.", Te, Te_IRF));
      exit(1);
    }
    m_recorder[BEMBody] = FrTimeRecorder<GeneralizedVelocity>(Te, dt);
  }

  void FrRadiationConvolutionModel::SetImpulseResponseSize(FrBody *body, double Te, double dt) {
    this->SetImpulseResponseSize(m_HDB->GetBody(body), Te, dt);
  }

  void FrRadiationConvolutionModel::SetImpulseResponseSize(double Te, double dt) {
    assert(Te > DBL_EPSILON);
    assert(dt > DBL_EPSILON);
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      this->SetImpulseResponseSize(BEMBody->first, Te, dt);
    }
  }

  void FrRadiationConvolutionModel::GetImpulseResponseSize(FrBEMBody *body, double &Te, double &dt) const {
    Te = m_recorder.at(body).GetTimePersistence();
    dt = m_recorder.at(body).GetTimeStep();
  }

  void FrRadiationConvolutionModel::GetImpulseResponseSize(FrBody *body, double &Te, double &dt) const {
    GetImpulseResponseSize(m_HDB->GetBody(body), Te, dt);
  }

  std::shared_ptr<FrRadiationConvolutionModel>
  make_radiation_convolution_model(const std::string &name,
                                   FrOffshoreSystem *system,
                                   std::shared_ptr<FrHydroDB> HDB) {

    // This method creates and adds the radiation convolution model without forward speed correction to the offshore system from the HDB.

    // Construction and initialization of the classes dealing with radiation models.
    auto radiationModel = std::make_shared<FrRadiationConvolutionModel>(name, system, HDB);

    // Addition to the system.
    system->Add(radiationModel);

    return radiationModel;
  }

}