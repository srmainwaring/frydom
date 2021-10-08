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

  GeneralizedForce FrRadiationModel::GetRadiationInertiaPart(FrBody *body) const {

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

  void FrRadiationModel::Compute(double time) {

  }

  // ----------------------------------------------------------------
  // Radiation model with recursive convolution
  // ----------------------------------------------------------------

  FrRadiationRecursiveConvolutionModel::FrRadiationRecursiveConvolutionModel(const std::string &name,
                                                                             FrOffshoreSystem *system,
                                                                             std::shared_ptr<FrHydroDB> HDB) :
      FrRadiationModel(name, system, HDB) {

    // Loop over every body subject to hydrodynamic loads.
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      auto body = m_HDB->GetBody(BEMBody->first);
      body->AddExternalForce(std::make_shared<FrRadiationConvolutionForce>("radiation_force", body, this));
    }
  }

  void FrRadiationRecursiveConvolutionModel::Compute(double time) {

    auto deltaT = time - c_time;
    c_time = time;

    if (deltaT < FLT_EPSILON)
      return;

    auto velocities = GetVelocities();

    if (abs(deltaT - c_deltaT) > 1E-6) {
      c_deltaT = deltaT;
      //TODO : what if poles change?
      Compute_PieceWiseLinearCoefficients(c_poles, deltaT);
    }

    c_states = c_alpha * c_states + c_beta0 * c_velocities + c_beta1 * velocities;

//    std::cout<<"c_states :"<<std::endl<<c_states<<std::endl;
    c_velocities = velocities;

    int indice = 0;
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      auto radiationForce = Compute_RadiationForce(BEMBody->first, indice);

      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      auto forceInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetForce(), NWU);
      auto TorqueInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetTorque(), NWU);

      m_radiationForce[BEMBody->first] = -GeneralizedForce(forceInWorld, TorqueInWorld);
    }

  }

  template<typename T>
  T FrRadiationRecursiveConvolutionModel::TrapezoidaleIntegration(T previousState, double velocity,
                                                                  double previousVelocity,
                                                                  hdb5_io::PoleResiduePair<T> poleResiduePair,
                                                                  double DeltaT) {
    auto alpha = exp(DeltaT * poleResiduePair.pole());
    double beta = 0.5 * DeltaT;
    return alpha * previousState + alpha * beta * previousVelocity + beta * velocity;
  }

  template<typename T>
  T FrRadiationRecursiveConvolutionModel::PiecewiseLinearIntegration(T previousState, double velocity,
                                                                     double previousVelocity,
                                                                     hdb5_io::PoleResiduePair<T> poleResiduePair,
                                                                     double DeltaT) {
    auto pole = poleResiduePair.pole();
    auto alpha = exp(DeltaT * pole);
    auto inverseNumerator = 1. / (pole * pole * DeltaT);
    auto beta0 = (1 + (pole * DeltaT - 1) * alpha) * inverseNumerator;
    auto beta1 = (-1 - pole * DeltaT + alpha) * inverseNumerator;
    return alpha * previousState + beta0 * previousVelocity + beta1 * velocity;
  }

  template<typename T>
  Vector3d<T>
  FrRadiationRecursiveConvolutionModel::PiecewiseLinearIntegration(Vector3d<T> previousStates, double velocity,
                                                                   double previousVelocity, Vector3d<T> poles,
                                                                   double DeltaT) {
    //TODO:: check this !
    auto alpha = exp(DeltaT * poles);
    auto inverseNumerator = 1. / (poles * poles * DeltaT);
    auto beta0 = (1 + (poles * DeltaT - 1) * alpha) * inverseNumerator;
    auto beta1 = (-1 - poles * DeltaT + alpha) * inverseNumerator;
    return alpha * previousStates + beta0 * previousVelocity + beta1 * velocity;
  }

  void FrRadiationRecursiveConvolutionModel::Initialize() {
    FrRadiationModel::Initialize();

    c_deltaT = GetSystem()->GetTimeStep();
    c_time = GetSystem()->GetTime() - c_deltaT;

    c_N_poles = 0;

    std::vector<Eigen::VectorXcd> temp_Poles;
    std::vector<Eigen::VectorXcd> temp_Residues;

    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

        auto radiationMask = BEMBody->first->GetRadiationMask(BEMBodyMotion->first);
        auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(BEMBodyMotion->first);

        for (auto idof : BodyMotionDOFMask.GetListDOF()) {

          FrMask radiationMaskForIDOF;
          radiationMaskForIDOF.SetMask(radiationMask.col(idof));

          for (auto iforce : radiationMaskForIDOF.GetListDOF()) {

            // TODO : virer les états auxiliaires de la HDB !
            auto poleResidue = BEMBody->first->GetModalCoefficients(BEMBodyMotion->first, idof, iforce);
            auto n_poles = poleResidue.nb_real_poles() + poleResidue.nb_cc_poles();
            c_N_poles += n_poles;
            temp_Poles.push_back(poleResidue.GetPoles());
            temp_Residues.push_back(poleResidue.GetResidues());

          }

        }

      }

    }

    c_states.setZero(c_N_poles);
    c_velocities.setZero(c_N_poles);
    c_alpha.setZero(c_N_poles);
    c_beta0.setZero(c_N_poles);
    c_beta1.setZero(c_N_poles);
    c_poles.resize(c_N_poles);
    c_residues.resize(c_N_poles);

    int indice = 0;
    for (auto& poles : temp_Poles) {
      auto npoles = poles.size();
      c_poles.segment(indice, npoles) = poles;
      indice += npoles;
    }

    indice = 0;
    for (auto& res : temp_Residues) {
      auto npoles = res.size();
      c_residues.segment(indice, npoles) = res;
      indice += npoles;
    }

    Compute_PieceWiseLinearCoefficients(c_poles,c_deltaT);


  }

  void FrRadiationRecursiveConvolutionModel::Compute_PieceWiseLinearCoefficients(const Eigen::ArrayXcd& poles, double dt) {
    assert(poles.size() == c_N_poles);
    auto q2Dt = poles * poles * dt;
    c_alpha = Eigen::exp(poles * dt);
    c_beta0 = (1 + (poles * dt - 1) * c_alpha) / q2Dt.array();
    c_beta1 = (-1 - poles * dt + c_alpha) / q2Dt.array();
  }

  Eigen::ArrayXd FrRadiationRecursiveConvolutionModel::GetVelocities() const {

    Eigen::ArrayXd velocities(c_N_poles);

    int indice = 0;

    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

        auto radiationMask = BEMBody->first->GetRadiationMask(BEMBodyMotion->first);

        auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(BEMBodyMotion->first);

        auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBodyMotion->first);
        auto currentVelocity = eqFrame->GetPerturbationGeneralizedVelocityInFrame(NWU);

        for (auto idof : BodyMotionDOFMask.GetListDOF()) {

          FrMask radiationMaskForIDOF;
          radiationMaskForIDOF.SetMask(radiationMask.col(idof));

          //TODO: ne pas calculer les composantes sur les dof fixés de BEMBody?
//          FrMask DOFMask;
//          DOFMask.SetMask(m_HDB->GetMapper()->GetBody(BEMBody->first)->GetDOFMask()->GetFreeDOFs());
//
//          auto BEMBodyMask = radiationMaskForIDOF && DOFMask;
//
//          for (auto iforce : BEMBodyMask.GetListDOF()) {
          for (auto iforce : radiationMaskForIDOF.GetListDOF()) {

            // TODO : virer les états auxiliaires de la HDB !
            auto poleResidue = BEMBody->first->GetModalCoefficients(BEMBodyMotion->first, idof, iforce);
            auto n_poles = poleResidue.nb_real_poles() + poleResidue.nb_cc_poles();
            velocities.segment(indice, n_poles) = Eigen::ArrayXd::Constant(n_poles, currentVelocity[idof]);
            indice += n_poles;

          }

        }

      }

    }

    return velocities;
  }


  GeneralizedForce FrRadiationRecursiveConvolutionModel::Compute_RadiationForce(FrBEMBody* body, int &indice) const {

    auto radiationForce = GeneralizedForce();
    radiationForce.SetNull();

    for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

      auto radiationMask = body->GetRadiationMask(BEMBodyMotion->first);

      auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(BEMBodyMotion->first);

      for (auto idof : BodyMotionDOFMask.GetListDOF()) {FrMask radiationMaskForIDOF;
        radiationMaskForIDOF.SetMask(radiationMask.col(idof));

        for (auto iforce : radiationMaskForIDOF.GetListDOF()) {

          // FIXME:: GetModalCoefficients not const
          auto poleResidue = body->GetModalCoefficients(BEMBodyMotion->first, idof, iforce);
          auto n_poles = poleResidue.nb_real_poles() + poleResidue.nb_cc_poles();
//            Eigen::VectorXcd v1(c_states.segment(indice, n_poles));
//            std::cout<<"residues.segment(indice,n_poles).transpose() :"<<std::endl<<residues.segment(indice,n_poles).transpose()<<std::endl;
//            std::cout<<"truc :"<<std::endl<<residues.segment(indice,n_poles).transpose() * c_states.matrix().segment(indice, n_poles)<<std::endl;
//            auto n_poles = c_residues[i_res].size();
          std::complex<double> temp = c_residues.segment(indice, n_poles).transpose() * c_states.matrix().segment(indice, n_poles);
          radiationForce[iforce] += temp.real();

          indice += n_poles;
        }
      }
    }

    return radiationForce;
  }

  std::shared_ptr<FrRadiationRecursiveConvolutionModel>
  make_recursive_convolution_model(const std::string &name,
                                   FrOffshoreSystem *system,
                                   std::shared_ptr<FrHydroDB> HDB) {

    // This subroutine creates and adds the radiation recursive convolution model to the offshore system from the HDB.

    // Construction and initialization of the classes dealing with radiation models.
    auto radiationModel = std::make_shared<FrRadiationRecursiveConvolutionModel>(name, system, HDB);

    // Addition to the system.
    system->Add(radiationModel);

    return radiationModel;
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
          auto interpK = BEMBody->first->GetIRFInterpolator("K")->at(idof);

          std::vector<mathutils::Vector6d<double>> kernel;
          kernel.reserve(vtime.size());
          for (unsigned int it = 0; it < vtime.size(); ++it) {
            auto irf = interpK->Eval(BEMBodyMotion->first->GetName(), vtime[it]) * velocity.at(it).at(idof);
            // radiation mask applied
            kernel.push_back(irf.cwiseProduct(radiationMask.col(idof).cast<double>()));
          }
          radiationForce += TrapzLoc(vtime, kernel);
        }
      }

      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      auto meanSpeed = eqFrame->GetFrameVelocityInFrame(NWU);

      if (meanSpeed.squaredNorm() > FLT_EPSILON and c_FScorrection) {
        radiationForce += ForwardSpeedCorrection(BEMBody->first);
      }

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

    // This method computes the forward speed correction of the radiation loads.

    //TODO: Il faudrait ajouter des termes de raideur dependant de la vitesse d'avance.

    // Initialization.
    auto radiationForce = GeneralizedForce();
    radiationForce.SetNull();

    // Forward speed.
    auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody);
    auto meanSpeed = eqFrame->GetFrameVelocityInFrame(NWU);

    auto angular = eqFrame->GetPerturbationAngularVelocityInFrame(NWU);

    // Infinite frequency added mass.
    auto Ainf = BEMBody->GetSelfInfiniteAddedMass();

    // Stored body velocity for the convolution.
    auto velocity = m_recorder.at(BEMBody).GetData();

    // Stored time for the convolution.
    auto vtime = m_recorder.at(BEMBody).GetTime();

    // KU.
    for (unsigned int idof = 4; idof < 6; idof++) {
      // Convolution.
      auto interpKu = BEMBody->GetIRFInterpolator("KU")->at(idof);
      std::vector<mathutils::Vector6d<double>> kernel;
      for (unsigned int it = 0; it < vtime.size(); ++it) {
        kernel.push_back(interpKu->Eval(BEMBody->GetName(), vtime[it]) * velocity[it].at(idof)); // KU(t-tau)*v(tau).
      }
      radiationForce += TrapzLoc(vtime, kernel) * meanSpeed.norm();
    }

    // KUXDerivative.
    if(m_HDB->GetIsXDerivative()) {
      auto KUXderivativeForce = GeneralizedForce();
      KUXderivativeForce.SetNull();
      auto KU2Force = GeneralizedForce();
      KU2Force.SetNull();
      for (unsigned int idof = 0; idof < 6; idof++) {
        // Convolution.
        auto interpKuXderivative = BEMBody->GetIRFInterpolator("KUXDerivative")->at(idof);
        auto interpKu2 = BEMBody->GetIRFInterpolator("KU2")->at(idof);
        std::vector<mathutils::Vector6d<double>> kernel_KUXDerivative;
        std::vector<mathutils::Vector6d<double>> kernel_KU2;
        for (unsigned int it = 0; it < vtime.size(); ++it) {
          kernel_KUXDerivative.push_back(interpKuXderivative->Eval(BEMBody->GetName(), vtime[it]) * velocity[it].at(idof)); // KUXderivative(t-tau)*v(tau).
          if(idof == 4 or idof == 5){
            kernel_KU2.push_back(interpKu2->Eval(BEMBody->GetName(), vtime[it]) * velocity[it].at(idof)); // KU2(t-tau)*v(tau).
          }
        }
        KUXderivativeForce += TrapzLoc(vtime, kernel_KUXDerivative) * meanSpeed.norm();
        KU2Force += TrapzLoc(vtime, kernel_KU2) * meanSpeed.norm() * meanSpeed.norm();
      }
      radiationForce += KUXderivativeForce + KU2Force;
    }

    // Infinite frequency damping.
    auto damping = Ainf.col(2) * angular.y() - Ainf.col(1) * angular.z(); // -A(inf)*L*V.
    radiationForce += meanSpeed.norm() * damping; // -U*A(inf)*L*V.
    if(m_HDB->GetIsXDerivative()) {
      auto dAdxinf = BEMBody->GetSelfXDerivativeInfiniteAddedMass();
      radiationForce += - meanSpeed.norm() * dAdxinf * eqFrame->GetPerturbationGeneralizedVelocityInFrame(NWU); // -U*dAdx(inf)*V.
    }

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

    // This subroutine creates and adds the radiation convolution model to the offshore system from the HDB.

    // Construction and initialization of the classes dealing with radiation models.
    auto radiationModel = std::make_shared<FrRadiationConvolutionModel>(name, system, HDB);

    // Addition to the system.
    system->Add(radiationModel);

    return radiationModel;
  }

}  // end namespace frydom

