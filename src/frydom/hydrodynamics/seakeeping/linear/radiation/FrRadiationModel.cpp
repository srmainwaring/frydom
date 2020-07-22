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


#include "FrRadiationModel.h"

#include "FrRadiationModelBase.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

#include "FrRadiationForce.h"


namespace frydom {

  // TODO : generaliser la méthode de mathutils pour les vecteurs Eigen

  Vector6d<double> TrapzLoc(std::vector<double> &x, std::vector<Vector6d<double>> &y) {

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

  };

  // ----------------------------------------------------------------
  // Radiation model
  // ----------------------------------------------------------------

  FrRadiationModel::FrRadiationModel(const std::string &name,
                                     FrOffshoreSystem *system,
                                     std::shared_ptr<FrHydroDB> HDB) :
      FrTreeNode(name, system),
      m_HDB(HDB) {

    // Creation of an AddedMassBase object.
    m_chronoPhysicsItem = std::make_shared<internal::FrRadiationModelBase>(this);
  }

  void FrRadiationModel::Initialize() {
    FrPhysicsItem::Initialize();
    m_chronoPhysicsItem->SetupInitial();
  }

  FrHydroMapper *FrRadiationModel::GetMapper() const {
    return m_HDB->GetMapper();
  }

//  FrOffshoreSystem *FrRadiationModel::GetSystem() const {
//    return GetParent();
//  }

  Force FrRadiationModel::GetRadiationForce(FrBEMBody *BEMBody) const {
    return m_radiationForce.at(BEMBody).GetForce();
  }

  Force FrRadiationModel::GetRadiationForce(FrBody *body) const {
    auto BEMBody = m_HDB->GetBody(body);
    return m_radiationForce.at(BEMBody).GetForce();
  }

  Torque FrRadiationModel::GetRadiationTorque(FrBEMBody *BEMBody) const {
    return m_radiationForce.at(BEMBody).GetTorque();
  }

  Torque FrRadiationModel::GetRadiationTorque(FrBody *body) const {
    auto BEMBody = m_HDB->GetBody(body);
    return m_radiationForce.at(BEMBody).GetTorque();
  }

  void FrRadiationModel::Compute(double time) {

  }

  // ----------------------------------------------------------------
  // Radiation model with convolution
  // ----------------------------------------------------------------

  FrRadiationConvolutionModel::FrRadiationConvolutionModel(const std::string &name,
                                                           FrOffshoreSystem *system,
                                                           std::shared_ptr<FrHydroDB> HDB) :
      FrRadiationModel(name, system, HDB) {

    // Loop over every body subject to hydrodynamic loads.
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      auto body = m_HDB->GetBody(BEMBody->first);
      body->AddExternalForce(std::make_shared<FrRadiationConvolutionForce>("radiation_force", body, this));
    }
  }

  void FrRadiationConvolutionModel::Initialize() {

    FrRadiationModel::Initialize();

    double Te, dt;
    m_HDB->GetImpulseResponseSize(GetParent()->GetTimeStep(), Te, dt);

    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      if (m_recorder.find(BEMBody->first) == m_recorder.end()) {
        m_recorder[BEMBody->first] = FrTimeRecorder<GeneralizedVelocity>(Te, dt);
      }
      m_recorder[BEMBody->first].Initialize();
    }
  }

  GeneralizedForce FrRadiationConvolutionModel::GetRadiationInertiaPart(FrBody *body) const {

    auto HDB = GetHydroDB();
    auto BEMBody = HDB->GetBody(body);

    auto force = GeneralizedForce();

    for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion != HDB->end(); BEMBodyMotion++) {

      auto infiniteAddedMass = BEMBody->GetInfiniteAddedMass(BEMBodyMotion->first);
      auto acc = GeneralizedAcceleration(BEMBodyMotion->second->GetCOGAccelerationInBody(NWU),
                                         BEMBodyMotion->second->GetAngularAccelerationInBody(NWU));
      force += -infiniteAddedMass * acc;
    }

    return force;
  }

  void FrRadiationConvolutionModel::Clear() {
    for (auto &BEMBody : *m_HDB) {
      m_recorder[BEMBody.first].Clear();
    }
  }

  void FrRadiationConvolutionModel::Compute(double time) {

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
      radiationForce.SetNull();

      for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

        auto radiationMask = BEMBody->first->GetRadiationMask(BEMBodyMotion->first);

        auto velocity = m_recorder[BEMBodyMotion->first].GetData();

        auto vtime = m_recorder[BEMBodyMotion->first].GetTime();

        auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(BEMBodyMotion->first);

        for (auto idof : BodyMotionDOFMask.GetListDOF()) {

          //idof applied here to BEMBodyMotion, even if it's not really explicit. The BEMBodyMotion is now called at the
          // Eval below. So it's equivalent to the next line, previously written for the old container.
          auto interpK = BEMBody->first->GetHDBInterpolator(HDB5_io::Body::IRF_K)->at(idof);
//          auto interpK = BEMBody->first->GetIRFInterpolatorK(BEMBodyMotion->first, idof);

          std::vector<mathutils::Vector6d<double>> kernel;
          kernel.reserve(vtime.size());
          for (unsigned int it = 0; it < vtime.size(); ++it) {
            auto irf = interpK->Eval(BEMBodyMotion->first->GetName(),vtime[it]) * velocity.at(it).at(idof);
            // radiation mask applied
            kernel.push_back(irf.cwiseProduct(radiationMask.col(idof).cast<double>()));
          }
          radiationForce += TrapzLoc(vtime, kernel);
        }
      }
      radiationForce += ForwardSpeedCorrection(BEMBody->first);

      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      auto forceInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetForce(), NWU);
      auto TorqueInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetTorque(), NWU);

      m_radiationForce[BEMBody->first] = -GeneralizedForce(forceInWorld, TorqueInWorld);
    }
  }

  void FrRadiationConvolutionModel::StepFinalize() {
    // Serialize and send the message log
//        FrObject::SendLog();
  }

  GeneralizedForce FrRadiationConvolutionModel::ForwardSpeedCorrection(FrBEMBody *BEMBody) const {

    auto radiationForce = GeneralizedForce();
    radiationForce.SetNull();

    auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody);
    auto meanSpeed = eqFrame->GetFrameVelocityInFrame(NWU);
    if (meanSpeed.squaredNorm() < FLT_EPSILON)
      return radiationForce;

    auto angular = eqFrame->GetPerturbationAngularVelocityInFrame(NWU);

    auto Ainf = BEMBody->GetSelfInfiniteAddedMass();

    auto velocity = m_recorder.at(BEMBody).GetData();
    auto vtime = m_recorder.at(BEMBody).GetTime();

    for (unsigned int idof = 4; idof < 6; idof++) {

      auto interpKu = BEMBody->GetHDBInterpolator(FrBEMBody::IRF_KU)->at(idof);

      std::vector<mathutils::Vector6d<double>> kernel;
      for (unsigned int it = 0; it < vtime.size(); ++it) {
        kernel.push_back(interpKu->Eval(BEMBody->GetName(), vtime[it]) * velocity[it].at(idof));
      }
      radiationForce += TrapzLoc(vtime, kernel) * meanSpeed.norm();
    }

    auto damping = Ainf.col(2) * angular.y() - Ainf.col(1) * angular.z();
    radiationForce += meanSpeed.norm() * damping;

    return radiationForce;

  }

  void FrRadiationConvolutionModel::SetImpulseResponseSize(FrBEMBody *BEMBody, double Te, double dt) {
    //TODO : check it is not already instanciated
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

    // This subroutine creates and adds the radiation convulation model to the offshore system from the HDB.

    // Construction and initialization of the classes dealing with radiation models.
    auto radiationModel = std::make_shared<FrRadiationConvolutionModel>(name, system, HDB);

    // Addition to the system.
    system->Add(radiationModel);

    return radiationModel;
  }

}  // end namespace frydom

