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

#include <chrono/physics/ChLinkMotorRotation.h>
#include <chrono/physics/ChLinkMotorLinear.h>
#include <frydom/mesh/FrHydroMesh.h>


#include "FrOffshoreSystem.h"

#include "chrono/utils/ChProfiler.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/solver/ChIterativeSolverVI.h"

#include "frydom/core/link/links_lib/FrLink.h"
#include "frydom/core/link/links_lib/actuators/FrActuator.h"
#include "frydom/core/link/links_lib/actuators/FrLinearActuator.h"
#include "frydom/core/link/links_lib/actuators/FrAngularActuator.h"
#include "frydom/core/link/constraint/FrConstraint.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrFEAMesh.h"
#include "frydom/core/statics/FrStaticAnalysis.h"

#include "frydom/core/math/functions/ramp/FrCosRampFunction.h"

#include "frydom/cable/fea/FrFEACable.h"
#include "frydom/cable/fea/FrFEALink.h"
#include "frydom/cable/mooring_components/FrClumpWeight.h"
#include "frydom/cable/catenary/FrCatenaryLine.h"

#include "frydom/environment/FrEnvironment.h"

#ifndef H5_NO_IRRLICHT
  #include "frydom/utils/FrIrrApp.h"
#endif

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"

#include "frydom/logging/FrLogManager.h"
#include "frydom/logging/FrPathManager.h"

#include "frydom/logging/FrTypeNames.h"

#include "frydom/logging/FrEventLogger.h"
#include "frydom/logging/FrSerializerFactory.h"

#include "frydom/hydrodynamics/morison/FrMorisonElements.h"



namespace frydom {


  namespace internal {

    template <class SystemType>
    FrSystemBase<SystemType>::FrSystemBase(frydom::FrOffshoreSystem *offshoreSystem) :
        SystemType(),
        m_offshoreSystem(offshoreSystem) {}

    template <class SystemType>
    void FrSystemBase<SystemType>::Initialize() {
      SystemType::SetupInitial();
    }

    template <class SystemType>
    void FrSystemBase<SystemType>::Update(bool update_assets) {

      // Note : there is no ChAssembly::Update() as it is better expanded here...

      CH_PROFILE("Update");

      if (!SystemType::is_initialized)
        SystemType::SetupInitial();

      SystemType::timer_update.start();  // Timer for profiling

      // Pre updates that are not about multibody dynamics
      m_offshoreSystem->PreUpdate();

      // Physics item that have to be updated before all
      m_offshoreSystem->PrePhysicsUpdate(SystemType::GetChTime(), update_assets);

      // Bodies updates  // FIXME : appeler les updates directement des objets frydom !
      for (auto &body : SystemType::GetAssembly().Get_bodylist()) {
        body->Update(SystemType::GetChTime(), update_assets);
//            body->Update(ChTime, update_assets);  // FIXME : Appel redondant
      }

      for (auto &physics_item : SystemType::GetAssembly().Get_otherphysicslist()) {
        physics_item->Update(SystemType::GetChTime(), update_assets);
      }

      // Links updates  // FIXME : appeler les updates directement des objets frydom !
      for (auto &link : SystemType::GetAssembly().Get_linklist()) {
        link->Update(SystemType::GetChTime(), update_assets);
      }

      for (auto &mesh : SystemType::GetAssembly().Get_meshlist()) {
        mesh->Update(SystemType::GetChTime(), update_assets);
      }

      // Update all contacts, if any
      SystemType::contact_container->Update(SystemType::GetChTime(), update_assets);

      // Post updates that are not about multibody dynamics
      m_offshoreSystem->PostUpdate();

      SystemType::timer_update.stop();

    }

//    void FrSystemBaseSMC::CustomEndOfStep() {
//      m_offshoreSystem->StepFinalize();
//    }

//        // -----------------------------------------------------------------------------
//        // **** PERFORM THE STATIC ANALYSIS, FINDING THE STATIC
//        // **** EQUILIBRIUM OF THE SYSTEM, WITH ITERATIVE SOLUTION
//        // -----------------------------------------------------------------------------
//
//        bool FrSystemBaseSMC::DoQuasiStatic(int niter, int nsteps) {
//
//            double m_undotime = GetChTime();
//            bool reach_tolerance = false;
//
//            if ((ncoords > 0) && (ndof >= 0)) {
//                for (int m_iter = 0; m_iter < niter; m_iter++) {
//                    // Get the speed of the bodies to check the convergence
//                    double bodyVel = 0;
//                    for (auto &body : bodylist) {
//                        bodyVel += body->GetPos_dt().Length2();
//                    }
//
//                    // Set no speed and accel. on bodies, meshes and other physics items
//                    Relax();
//
//                    std::cout<<m_iter<<", "<<GetChTime()<<", "<<bodyVel<<std::endl;
//                    // TODO : introduce a tolerance parameter
//                    if (bodyVel < 1E-5 && GetChTime()>m_undotime+step*nsteps) {
//                        reach_tolerance = true;
//                        break;
//                    }
//                    DoFrameDynamics(m_undotime + m_iter * step * nsteps);
//                }
//
//                // Set no speed and accel. on bodies, meshes and other physics items
//                Relax();
//            }
//
//            SetChTime(m_undotime);
//            return reach_tolerance;
//        }

    template <class SystemType>
    bool FrSystemBase<SystemType>::DoStaticLinear() {
      // Set no speed and accel. on bodies, meshes and other physics items
      for (auto &body : SystemType::GetAssembly().Get_bodylist()) {
        body->SetNoSpeedNoAcceleration();
      }
      for (auto &mesh : SystemType::GetAssembly().Get_meshlist()) {
        mesh->SetNoSpeedNoAcceleration();
      }
      for (auto &ip : SystemType::GetAssembly().Get_otherphysicslist()) {
        ip->SetNoSpeedNoAcceleration();
      }
      return true;
    }

//    int FrSystemBaseSMC::DoStepDynamics(double m_step) {
//      chrono::ChSystem::DoStepDynamics(m_step);
//      m_offshoreSystem->StepFinalize();
//    }

    template <class SystemType>
    bool FrSystemBase<SystemType>::Integrate_Y() {
      chrono::ChSystem::ForceUpdate();
      auto output = chrono::ChSystem::Integrate_Y();
      m_offshoreSystem->StepFinalize();
      return output;
    }

    void AddPhysicsItem(FrOffshoreSystem& system, std::shared_ptr<FrPhysicsItem> item) {
      system.m_chronoSystem->AddOtherPhysicsItem(internal::GetChronoPhysicsItem(item));
      system.m_physicsItemsList.push_back(item);
      event_logger::info(system.GetTypeName(), system.GetName(), "A physics item has been ADDED to the system");
    }

    chrono::ChSystem *GetChronoSystem(FrOffshoreSystem *system) {
      return system->m_chronoSystem.get();
    }
//    FrSystemBaseSMC::FrSystemBaseSMC(FrOffshoreSystem *offshoreSystem) : FrSystemBase(offshoreSystem) {}


  }  // end namespace frydom::internal



  /// Default constructor
  /// \param systemType contact method system (SMOOTH_CONTACT/NONSMOOTH_CONTACT)
  /// \param timeStepper time stepper type
  /// \param solver solver type
  FrOffshoreSystem::FrOffshoreSystem(const std::string &name,
                                     SYSTEM_TYPE systemType,
                                     TIME_STEPPER timeStepper,
                                     SOLVER solver,
                                     const std::string& logFolderName) :
      FrLoggable(name, TypeToString(this), nullptr),
      m_monitor_real_time(false),
      m_config_file() {

    // Creating the chrono System backend. It drives the way contact are modelled
    SetSystemType(systemType, false);

    // Creating the log manager service
    m_logManager = std::make_unique<FrLogManager>(this, logFolderName);

    // Setting the time stepper
    SetTimeStepper(timeStepper, false);

    // Setting the constraints solver
    SetSolver(solver, false);


    // Check compatibility between system contact model, time stepper and constraint solver
    CheckCompatibility();

    // Creating the environment
    m_environment = std::make_unique<FrEnvironment>(this);

    // Creating the path manager service
    m_pathManager = std::make_unique<FrPathManager>();
    m_pathManager->RegisterTreeNode(this);

//    // Creating the log manager service
//    m_logManager = std::make_unique<FrLogManager>(this);

    // Creating the static analysis
    m_statics = std::make_unique<FrStaticAnalysis>(this);

    // Creating a fixed world body to be able to attach anything to it (anchors...)
    CreateWorldBody();
  }

  FrOffshoreSystem::~FrOffshoreSystem() = default;

  const FrConfig &FrOffshoreSystem::config_file() {
    return m_config_file;
  }

// ***** Body *****

  void FrOffshoreSystem::AddBody(std::shared_ptr<FrBody> body) {

//    // TODO : voir si on set pas d'autorite le mode de contact a celui du systeme plutot que de faire un if...
//    if (!CheckBodyContactMethod(body)) {
//      body->SetContactMethod(m_systemType);
//    }

    m_chronoSystem->AddBody(internal::GetChronoBody(body));  // Authorized because this method is a friend of FrBody
    m_bodyList.push_back(body);

    event_logger::info(GetTypeName(), GetName(),
                       "Body {} has been ADDED to the system", body->GetName());
  }

  FrOffshoreSystem::BodyContainer &FrOffshoreSystem::GetBodyList() {
    return m_bodyList;
  }

  void FrOffshoreSystem::RemoveBody(std::shared_ptr<FrBody> body) {

    m_chronoSystem->RemoveBody(internal::GetChronoBody(body));

    auto it = std::find(body_begin(), body_end(), body);
    assert(it != body_end());
    m_bodyList.erase(it);

    body->RemoveAllForces();
    body->RemoveAllNodes();



    event_logger::info(GetTypeName(), GetName(), "Body {} has been REMOVED from the system", body->GetName());

    // FIXME : we should launch removal of FrNode and FrForce objects attached to this body from the logManager and path manager

  }

  void FrOffshoreSystem::RemoveAllBodies() {

    for (auto &body: m_bodyList)
      Remove(body);

    event_logger::info(GetTypeName(), GetName(), "Every bodies have been removed from the system");

  }


// ***** Link *****

  void FrOffshoreSystem::AddLink(std::shared_ptr<FrLink> link) {
    m_chronoSystem->AddLink(internal::GetChronoLink(link));
    m_linkList.push_back(link);
    event_logger::info(GetTypeName(), GetName(), "Link {} has been ADDED to the system", link->GetName());
  }

  void FrOffshoreSystem::RemoveLink(std::shared_ptr<FrLink> link) {

    m_chronoSystem->RemoveLink(internal::GetChronoLink(link));

    auto it = std::find(link_begin(), link_end(), link);
    assert(it != link_end());
    if (it == link_end()) {
      event_logger::error(GetTypeName(), GetName(),
                          "Fail to remove link {} as it is not registered", link->GetName());
      return;
    }

    m_linkList.erase(it);
    event_logger::info(GetTypeName(), GetName(), "Link {} has been REMOVED from the system", link->GetName());
  }

  void FrOffshoreSystem::RemoveAllLinks() {

    for (auto &link: m_linkList)
      Remove(link);
  }


// ***** Constraint *****

  void FrOffshoreSystem::AddConstraint(std::shared_ptr<FrConstraint> constraint) {
    m_chronoSystem->AddLink(internal::GetChronoConstraint(constraint));
    m_constraintList.push_back(constraint);
    event_logger::info(GetTypeName(), GetName(), "Constraint {} has been ADDED to the system", constraint->GetName());
  }

  void FrOffshoreSystem::RemoveConstraint(std::shared_ptr<FrConstraint> constraint) {

    m_chronoSystem->RemoveLink(internal::GetChronoConstraint(constraint));

    auto it = std::find(constraint_begin(), constraint_end(), constraint);
    assert(it != constraint_end());
    if (it == constraint_end()) {
      event_logger::error(GetTypeName(), GetName(),
                          "Fail to remove constraint {} as it is not registered", constraint->GetName());
      return;
    }

    m_constraintList.erase(it);
    event_logger::info(GetTypeName(), GetName(), "Constraint {} has been REMOVED from the system",
                       constraint->GetName());
  }

  void FrOffshoreSystem::RemoveAllConstraints() {

    for (auto &constraint: m_constraintList)
      Remove(constraint);
  }


// ***** Actuator *****

  void
  FrOffshoreSystem::AddActuator(std::shared_ptr<FrActuator> actuator) {

    if (auto linear_actuator = std::dynamic_pointer_cast<FrLinearActuator>(actuator)) {
      m_chronoSystem->AddLink(internal::GetChronoActuator(linear_actuator));
    }

    if (auto angular_actuator = std::dynamic_pointer_cast<FrAngularActuator>(actuator)) {
      m_chronoSystem->AddLink(internal::GetChronoActuator(angular_actuator));
    }

    m_actuatorList.push_back(actuator);
    event_logger::info(GetTypeName(), GetName(), "Actuator {} has been ADDED to the system", actuator->GetName());
  }

  void
  FrOffshoreSystem::RemoveActuator(std::shared_ptr<FrActuator> actuator) {

    if (auto linear_actuator = std::dynamic_pointer_cast<FrLinearActuator>(actuator)) {
      m_chronoSystem->RemoveLink(internal::GetChronoActuator(linear_actuator));
    }

    if (auto angular_actuator = std::dynamic_pointer_cast<FrAngularActuator>(actuator)) {
      m_chronoSystem->RemoveLink(internal::GetChronoActuator(angular_actuator));
    }

    auto it = std::find(actuator_begin(), actuator_end(), actuator);
    assert(it != actuator_end());
    if (it == actuator_end()) {
      event_logger::error(GetTypeName(), GetName(),
                          "Fail to remove actuator {} as it is not registered", actuator->GetName());
      return;
    }

    m_actuatorList.erase(it);
    event_logger::info(GetTypeName(), GetName(), "Actuator {} has been REMOVED from the system", actuator->GetName());
  }

  void FrOffshoreSystem::RemoveAllActuators() {

    for (auto &actuator: m_actuatorList)
      Remove(actuator);
  }


  void FrOffshoreSystem::AddCatenaryLineBase(std::shared_ptr<FrCatenaryLineBase> catenary_line_base) {
    m_chronoSystem->AddOtherPhysicsItem(internal::GetChronoPhysicsItem(catenary_line_base));
    m_physicsItemsList.push_back(catenary_line_base);
    event_logger::info(GetTypeName(), GetName(), "A Catenary line has been ADDED to the system");
  }

  void FrOffshoreSystem::AddEquilibriumFrame(std::shared_ptr<FrEquilibriumFrame> equilibrium_frame) {
    m_chronoSystem->AddOtherPhysicsItem(internal::GetChronoPhysicsItem(equilibrium_frame));
    m_physicsItemsList.push_back(equilibrium_frame);
    event_logger::info(GetTypeName(), GetName(), "An equilibrium frame has been ADDED to the system");
  }

  void FrOffshoreSystem::AddRadiationModel(std::shared_ptr<FrRadiationModel> radiation_model) {
    m_chronoSystem->AddOtherPhysicsItem(internal::GetChronoPhysicsItem(radiation_model));
    m_physicsItemsList.push_back(radiation_model);

    auto chrono_mesh = internal::GetChronoAddedMass(radiation_model);
    m_chronoSystem->AddMesh(chrono_mesh);
    //for (auto link : chrono_mesh->GetAddedMass()->GetLinks()) {
    //  m_chronoSystem->AddLink(link);
    //}

    event_logger::info(GetTypeName(), GetName(), "A radiation model has been ADDED to the system");
  }

  void FrOffshoreSystem::AddHydroMesh(std::shared_ptr<FrHydroMesh> hydro_mesh) {
    m_chronoSystem->AddOtherPhysicsItem(internal::GetChronoPhysicsItem(hydro_mesh));
    m_physicsItemsList.push_back(hydro_mesh);
    event_logger::info(GetTypeName(), GetName(), "An hydrodynamic mesh has been ADDED to the system");
  }

  void FrOffshoreSystem::RemoveHydroMesh(std::shared_ptr<FrHydroMesh> hydro_mesh) {

    m_chronoSystem->RemoveOtherPhysicsItem(internal::GetChronoPhysicsItem(hydro_mesh));

    auto it = std::find(physics_item_begin(), physics_item_end(), hydro_mesh);
    assert(it != physics_item_end());
    if (it == physics_item_end()) {
      event_logger::error(GetTypeName(), GetName(),
                          "Fail to remove hydro mesh {} as it is not registered", hydro_mesh->GetName());
      return;
    }

    m_physicsItemsList.erase(it);
    event_logger::info(GetTypeName(), GetName(), "Hydro mesh {} has been REMOVED from the system", hydro_mesh->GetName());
  }

  FrOffshoreSystem::PhysicsContainer FrOffshoreSystem::GetPhysicsItemList() {
    return m_physicsItemsList;
  }

  void FrOffshoreSystem::AddMorisonElements(std::shared_ptr<FrMorisonCompositeElement> morison_elements) {
    m_chronoSystem->AddOtherPhysicsItem(internal::GetChronoPhysicsItem(morison_elements));
    m_physicsItemsList.push_back(morison_elements);

    auto chrono_mesh = internal::GetChronoMorisonAddedMass(morison_elements);
    if (chrono_mesh)
      m_chronoSystem->AddMesh(chrono_mesh);
    event_logger::info(GetTypeName(), GetName(), "Morison elements have been ADDED to the system");
  }

// ***** FEAMesh *****

  void FrOffshoreSystem::AddFEAMesh(std::shared_ptr<FrFEAMesh> feaMesh) {

    m_chronoSystem->AddMesh(internal::GetChronoFEAMesh(feaMesh));

//      feaMesh->m_system = this;
    m_feaMeshList.push_back(feaMesh);
  }

  FrOffshoreSystem::FEAMeshContainer FrOffshoreSystem::GetFEAMeshList() {
    return m_feaMeshList;
  }

  void FrOffshoreSystem::RemoveFEAMesh(std::shared_ptr<FrFEAMesh> feamesh) {

    m_chronoSystem->RemoveMesh(internal::GetChronoFEAMesh(feamesh));

    auto it = std::find(m_feaMeshList.begin(), m_feaMeshList.end(), feamesh);
    assert(it != m_feaMeshList.end());
    m_feaMeshList.erase(it);
//      feamesh->m_system = nullptr;

  }

  void FrOffshoreSystem::AddFEACable(std::shared_ptr<FrFEACable> cable) {

    // Add the FEA mesh
    AddFEAMesh(cable);

    // Add the links
    auto chrono_mesh = internal::GetChronoFEAMesh(cable);
    m_chronoSystem->Add(chrono_mesh->GetStartLink());
    m_chronoSystem->Add(chrono_mesh->GetEndLink());

  }

  void FrOffshoreSystem::AddClumpWeight(std::shared_ptr<FrClumpWeight> clump_weight) {
    m_physicsItemsList.push_back(clump_weight);

    // Add the link...
    m_chronoSystem->AddLink(internal::GetClumpWeightConstraint(clump_weight));

  }

  void FrOffshoreSystem::RemoveFEACable(std::shared_ptr<FrFEACable> cable) {

    Remove(cable);

    // Remove the links
    auto chrono_mesh = internal::GetChronoFEAMesh(cable);
    m_chronoSystem->RemoveOtherPhysicsItem(chrono_mesh->GetStartLink());
    m_chronoSystem->RemoveOtherPhysicsItem(chrono_mesh->GetEndLink());

  }

//  void FrOffshoreSystem::AddLumpedMassNode(std::shared_ptr<internal::FrLMNode> lm_node) {
//    m_chronoSystem->AddBody(lm_node->GetBody());
//  }
//
//  void FrOffshoreSystem::AddLumpedMassElement(std::shared_ptr<internal::FrLMElement> lm_element) {
//    m_chronoSystem->AddLink(lm_element->GetLink());
//  }
//
//  void FrOffshoreSystem::AddLumpedMassCable(std::shared_ptr<FrLumpedMassCable> lm_cable) {
//    // Nothing to do ??
//  }

  void FrOffshoreSystem::MonitorRealTimePerfs(bool val) {
    m_monitor_real_time = val;
    event_logger::info(GetTypeName(), GetName(), "Monitoring time performance set to {}", val);
  }


// ***** Environment *****

  FrEnvironment *FrOffshoreSystem::GetEnvironment() const {
    return m_environment.get();
  }

  std::shared_ptr<FrBody> FrOffshoreSystem::GetWorldBody() const {
    return m_worldBody;
  }

  std::shared_ptr<FrNode> FrOffshoreSystem::NewWorldFixedNode(const std::string &name) {
    return m_worldBody->NewNode(name);
  }

  void FrOffshoreSystem::PreUpdate() {
    // TODO : voir si on ne met pas l'environnement comme un physics Item update en tant que PrePhysicsItem
    m_environment->Update(m_chronoSystem->GetChTime());
  }

  void FrOffshoreSystem::PostUpdate() {
    // TODO
  }

  void FrOffshoreSystem::PrePhysicsUpdate(double time, bool update_assets) {
    for (auto &item : m_physicsItemsList) {
      item->Update(time);
    }
  }

  void FrOffshoreSystem::Initialize() {

    if (m_isInitialized)
      return;


    event_logger::info(GetTypeName(), GetName(), "BEGIN OffshoreSystem initialization");
    event_logger::flush();

    // Initializing environment before bodies
    m_environment->Initialize();

    for (auto &item : m_physicsItemsList) {
      item->Initialize();
    }

    for (auto &item : m_bodyList) {
      item->Initialize();
    }

    for (auto &item : m_linkList) {
      item->Initialize();
    }

    for (auto &item : m_constraintList) {
      item->Initialize();
    }

    for (auto &item : m_actuatorList) {
      item->Initialize();
    }

    for (auto &item : m_feaMeshList) {
      item->Initialize();
    }

    m_chronoSystem->Update();

    m_logManager->Initialize();

    m_isInitialized = true;

    event_logger::info(GetTypeName(), GetName(), "END OffshoreSystem initialization");
    event_logger::flush();

  }

  void FrOffshoreSystem::ForceInitialize() {
    m_isInitialized = false;
    Initialize();
  }

  void FrOffshoreSystem::DefineLogMessages() {  // FIXME : doit etre appele par logManager !!!

    // Solver related messages

    auto msg = NewMessage("solver", "Messages relative to the dynamic solver and constraint solvers");

    msg->AddField<double>("time", "s", "Current time of the simulation", [this]() { return GetTime(); });

    if (auto solver = std::dynamic_pointer_cast<chrono::ChIterativeSolverVI>(m_chronoSystem->GetSolver())) {
      msg->AddField<int>("iter", "", "number of total iterations taken by the solver", [this]() {
        return dynamic_cast<chrono::ChIterativeSolverVI *>(m_chronoSystem->GetSolver().get())->GetIterations();
      });
    }

    msg->AddField<int>("iter_log", "", "number of total iteractions make by the iterative solver", [this]() { return m_chronoSystem->GetSolver()->GetIterLog(); });

    msg->AddField<double>("residual_log", "", "residual of the iterative solver", [this]() { return m_chronoSystem->GetSolver()->GetResidualLog(); }); 

    msg->AddField<double>("max_delta_unknowns", "", "", [this]() { return m_chronoSystem->GetSolver()->GetMaxDeltaUnknowns(); });

    if (auto solver_iter = std::dynamic_pointer_cast<chrono::ChIterativeSolverVI>(m_chronoSystem->GetSolver())) {

      msg->AddField<int>("iter", "", "number of total iterations taken by the solver", [this]() {
        return dynamic_cast<chrono::ChIterativeSolverVI *>(m_chronoSystem->GetSolver().get())->GetIterations();
      });

      //msg->AddField<double>("violationResidual", "", "constraint violation", [this]() {
      //  return dynamic_cast<chrono::ChIterativeSolverVI *>(m_chronoSystem->GetSolver().get())->GetViolationHistory().back();
      //});

      //msg->AddField<double>("LagrangeResidual", "", "maximum change in Lagrange multipliers", [this]() {
      //  return dynamic_cast<chrono::ChIterativeSolverVI *>(m_chronoSystem->GetSolver().get())->GetDeltalambdaHistory().back();
      //});

    }
  }

  void FrOffshoreSystem::StepFinalize() {

    m_environment->StepFinalize();

    for (auto &item : m_physicsItemsList) {
      item->StepFinalize();
    }

    for (auto &item : m_bodyList) {
      item->StepFinalize();
    }

    for (auto &item : m_linkList) {
      item->StepFinalize();
    }

    for (auto &item : m_constraintList) {
      item->StepFinalize();
    }

    for (auto &item : m_actuatorList) {
      item->StepFinalize();
    }

    for (auto &item : m_feaMeshList) {
      item->StepFinalize();
    }

    // Logging
    m_logManager->StepFinalize();

    if (m_monitor_real_time) {
      double ratio = m_chronoSystem->GetTimerStep() / GetTimeStep();

      std::string msg = (ratio > 1.) ? "slower that real time" : "faster than real time";

      std::cout << "At time : "
                << GetTime()
                << ";\t"
                << msg
                << "(1 physical second computed in "
                << ratio
                << " seconds)"
                << std::endl;
    }

  }

  void FrOffshoreSystem::SetSystemType(SYSTEM_TYPE type, bool checkCompat) {

    if (m_chronoSystem) Clear(); // Clear the system from every bodies etc...

    // Creating the chrono System backend. It drives the way contact are modelled
    switch (type) {
      case SMOOTH_CONTACT:
        m_chronoSystem = std::make_unique<internal::FrSystemBaseSMC>(this);
        break;
      case NONSMOOTH_CONTACT:
        std::cout << "NSC systems is not tested for now !!!!" << std::endl;
        m_chronoSystem = std::make_unique<internal::FrSystemBaseNSC>(this);
        break;
    }

    m_systemType = type;

    if (checkCompat) CheckCompatibility();
  }

  void FrOffshoreSystem::CheckCompatibility() const {
    // TODO : verifier la compatibilite entre type systeme, solveur et integrateur temporel

  }

//  bool FrOffshoreSystem::CheckBodyContactMethod(std::shared_ptr<FrBody> body) {
//    return m_systemType == body->GetContactType();
//  }

  FrOffshoreSystem::SYSTEM_TYPE FrOffshoreSystem::GetSystemType() const {
    return m_systemType;
  }

  void FrOffshoreSystem::SetSolver(SOLVER solver, bool checkCompat) {

    using SOLVERS = chrono::ChSolver::Type;

    switch (solver) {
      case PSOR:
        m_chronoSystem->SetSolverType(SOLVERS::PSOR);
        break;
      case PSSOR:
        m_chronoSystem->SetSolverType(SOLVERS::PSSOR);
        break;
      case PJACOBI:
        m_chronoSystem->SetSolverType(SOLVERS::PJACOBI);
        break;
      case PMINRES:
        m_chronoSystem->SetSolverType(SOLVERS::PMINRES);
        break;
      case BARZILAIBORWEIN:
        m_chronoSystem->SetSolverType(SOLVERS::BARZILAIBORWEIN);
        break;
      case APGD:
        m_chronoSystem->SetSolverType(SOLVERS::APGD);
        break;
      case GMRES:
        m_chronoSystem->SetSolverType(SOLVERS::GMRES);
        break;
      case MINRES:
        m_chronoSystem->SetSolverType(SOLVERS::MINRES);
        break;
      case PARDISO_MKL:
        //m_chronoSystem->SetSolverType(SOLVERS::PARDISO_MKL);
        std::cout << "warning : PARDISO_MKL not enabled" << std::endl;
        exit(EXIT_FAILURE);
        break;
    }

    m_solverType = solver;

    if (checkCompat) CheckCompatibility();
  }

  void FrOffshoreSystem::SetSolverMaxIterations(int max_iters) {
    m_chronoSystem->SetSolverMaxIterations(max_iters);
    event_logger::info(GetTypeName(), GetName(), "Set Solver max iterations {}", max_iters);
  }

  int FrOffshoreSystem::GetSolverMaxIterations() const {
    return m_chronoSystem->GetSolverMaxIterations();
  }

  void FrOffshoreSystem::SetSolverTolerance(double tolerance) {
    m_chronoSystem->SetSolverTolerance(tolerance);
    event_logger::info(GetTypeName(), GetName(), "Set solver tolerance {}", tolerance);
  }

  double FrOffshoreSystem::GetSolverTolerance() const {
    return m_chronoSystem->GetSolverTolerance();
  }


  void FrOffshoreSystem::SetSolverWarmStarting(bool useWarm) {
    if (auto solver = std::dynamic_pointer_cast<chrono::ChIterativeSolverVI>(m_chronoSystem->GetSolver())) {
      solver->EnableWarmStart(useWarm);

      std::string active;
      if (useWarm) {
        active = "activated";
      } else {
        active = "deactivated";
      }
      event_logger::info(GetTypeName(), GetName(), "Solver warm start {}", active);
    } else {
      std::cout << "warning : warm starting enabled only on Iterative Solver.\nWarm Starting is not activated." << std::endl;
    }
  }

  void FrOffshoreSystem::SetSolverOverrelaxationParam(double omega) {
    if (auto solver = std::dynamic_pointer_cast<chrono::ChIterativeSolverVI>(m_chronoSystem->GetSolver())) {
      solver->SetOmega(omega);
      event_logger::info(GetTypeName(), GetName(),
                         "Solver over relaxation parameter set to {}", omega);
    } else {
      std::cout << "warnin : Overrelaxation param enable only on Iterative Solver VI.\n Overrelaxation is not activated." << std::endl;
    }
  }

  void FrOffshoreSystem::SetSolverDiagonalPreconditioning(bool val) {
    if (m_solverType == MINRES) {
      auto solver = dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get());
      solver->EnableDiagonalPreconditioner(val);
      std::string active;
      if (val) {
        active = "activated";
      } else {
        active = "deactivated";
      }
      event_logger::info(GetTypeName(), GetName(), "Solver diagonal preconditioning {}.", active);
    } else {
      event_logger::warn(GetTypeName(), GetName(),
          "Solver diagonal preconditioning is currently only available for MINRES solver");
    }
  }

  void FrOffshoreSystem::SetSolverSharpnessParam(double momega) {
    if (auto solver = std::dynamic_pointer_cast<chrono::ChIterativeSolverVI>(m_chronoSystem->GetSolver())) {
      solver->SetSharpnessLambda(momega);
      event_logger::info(GetTypeName(), GetName(),
                         "Solver sharpness parameter set to {}", momega);
    } else {
      std::cout << "warning : Solver sharpness param enabled only on Iterative solver VI.\nSolver sharpness param is not activated/" << std::endl;
    }
  }

  void FrOffshoreSystem::SetParallelThreadNumber(int nbThreads) {
    m_chronoSystem->SetNumThreads(nbThreads);
    event_logger::info(GetTypeName(), GetName(),
                       "Number of threads for simulation set to {}", nbThreads);
  }

  //void FrOffshoreSystem::SetSolverMaxIterStab(int maxIter) {
  //  if (auto solver : std::dynamic_pointer_cast<m_chronoSystem->SetMaxItersSolverStab(maxIter);
  //  event_logger::info(GetTypeName(), GetName(),
  //                     "Solver maximum number of iterations for stabilization set to {}", maxIter);
  //}

  void FrOffshoreSystem::SetSolverMaxIterAssembly(int maxIter) {
    m_chronoSystem->SetMaxiter(maxIter);
    event_logger::info(GetTypeName(), GetName(),
                       "Solver maximum number of iterations for constraint assembly the system set to {}", maxIter);
  }

  //void FrOffshoreSystem::SetSolverGeometricTolerance(double tol) {
  //  m_chronoSystem->GetSolver()->SetTol(tol);
  //  event_logger::info(GetTypeName(), GetName(),
  //                      "Solver geometric tolerance set to {}", tol);
  //}

  void FrOffshoreSystem::SetSolverForceTolerance(double tol) {
    m_chronoSystem->SetSolverForceTolerance(tol);
    event_logger::info(GetTypeName(), GetName(),
                       "Solver force tolerance set to {}", tol);
  }

  void FrOffshoreSystem::UseMaterialProperties(bool use) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->UseMaterialProperties(use);
      std::string active;
      if (use) {
        active = "activated";
      } else {
        active = "deactivated";
      }
      event_logger::info(GetTypeName(), GetName(),"Material properties {}", active);
    } else {
      event_logger::error(GetTypeName(), GetName(),
          "The use of material properties is only for SMOOTH_CONTACT systems");
    }
  }

  void FrOffshoreSystem::SetContactForceModel(FrOffshoreSystem::CONTACT_MODEL model) {
    if (m_systemType == SMOOTH_CONTACT) {
      auto systemSMC = dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get());
      using ContactForceModel = chrono::ChSystemSMC::ContactForceModel;
      std::string model_str;
      switch (model) {
        case HOOKE:
          systemSMC->SetContactForceModel(ContactForceModel::Hooke);
          model_str = "HOOKE";
          break;
        case HERTZ:
          systemSMC->SetContactForceModel(ContactForceModel::Hertz);
          model_str = "HERTZ";
          break;
        case COULOMB:
          systemSMC->SetContactForceModel(ContactForceModel::PlainCoulomb);
          model_str = "COULOMB";
          break;
      }
      event_logger::info(GetTypeName(), GetName(), "Constact force model set to {}", model_str);
    } else {
      event_logger::error(GetTypeName(), GetName(),
                          "Contact force model is only for SMOOTH_CONTACT systems");
    }
  }

  void FrOffshoreSystem::SetAdhesionForceModel(FrOffshoreSystem::ADHESION_MODEL model) {
    if (m_systemType == SMOOTH_CONTACT) {
      auto systemSMC = dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get());
      using AdhesionForceModel = chrono::ChSystemSMC::AdhesionForceModel;
      switch (model) {
        case CONSTANT:
          systemSMC->SetAdhesionForceModel(AdhesionForceModel::Constant);
          break;
        case DMT:
          systemSMC->SetAdhesionForceModel(AdhesionForceModel::DMT);
          break;
      }
    } else {
      std::cerr << "Adhesion force model is only for SMOOTH_CONTACT systems" << std::endl;
    }
  }

  void FrOffshoreSystem::SetTangentialDisplacementModel(FrOffshoreSystem::TANGENTIAL_DISP_MODEL model) {
    if (m_systemType == SMOOTH_CONTACT) {
      auto systemSMC = dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get());
      using TangentialDisplacementModel = chrono::ChSystemSMC::TangentialDisplacementModel;
      switch (model) {
        case NONE:
          systemSMC->SetTangentialDisplacementModel(TangentialDisplacementModel::None);
          break;
        case ONE_STEP:
          systemSMC->SetTangentialDisplacementModel(TangentialDisplacementModel::OneStep);
          break;
        case MULTI_STEP:
          systemSMC->SetTangentialDisplacementModel(TangentialDisplacementModel::MultiStep);
          break;
      }
    } else {
      std::cerr << "Adhesion force model is only for SMOOTH_CONTACT systems" << std::endl;
    }
  }

  void FrOffshoreSystem::SetStiffContact(bool isStiff) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->SetStiffContact(isStiff);
      event_logger::info(GetTypeName(), GetName(),
                         "Stiff contact {}", (isStiff) ? "activated" : "deactivated");
    } else {
      event_logger::error(GetTypeName(), GetName(),
                          "StiffContact is only for SMOOTH_CONTACT systems. Action ignored");
    }
  }

  void FrOffshoreSystem::SetSlipVelocityThreshold(double velocity) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->SetSlipVelocityThreshold(velocity);
      event_logger::info(GetTypeName(), GetName(),
                         "Slip Velocity Threshold set to {} m/s", velocity);
    } else {
      event_logger::error(GetTypeName(), GetName(),
                          "Slip Velocity Threshold is only for SMOOTH_CONTACT systems. Action ignored");
    }
  }

  void FrOffshoreSystem::SetCharacteristicImpactVelocity(double velocity) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->SetCharacteristicImpactVelocity(velocity);
      event_logger::info(GetTypeName(), GetName(),
                         "Characteristic Impact Velocity set to {} m/s", velocity);
    } else {
      event_logger::error(GetTypeName(), GetName(),
                          "Characteristic Impact Velocity is only for SMOOTH_CONTACT systems. Action ignored");
    }
  }

  void FrOffshoreSystem::SetMinBounceSpeed(double speed) {
    m_chronoSystem->SetMinBounceSpeed(speed);
  }

  void FrOffshoreSystem::SetMaxPenetrationRecoverySpeed(double speed) {
    m_chronoSystem->SetMaxPenetrationRecoverySpeed(speed);
  }

  Force FrOffshoreSystem::GetContactForceOnBodyInWorld(FrBody* body, FRAME_CONVENTION fc) const {
    auto ChForce = m_chronoSystem->GetContactContainer()->GetContactableForce(internal::GetChronoBody(body).get());
    auto force = internal::ChVectorToVector3d<Force>(ChForce);
    if(IsNED(fc)) internal::SwapFrameConvention(force);
    return force;
  }

  int FrOffshoreSystem::GetNbPositionCoords() const {
    return m_chronoSystem->GetNcoords();
  }

  int FrOffshoreSystem::GetNbVelocityCoords() const {
    return m_chronoSystem->GetNcoords_w();
  }

  int FrOffshoreSystem::GetNbConstraintsCoords() const {
    return m_chronoSystem->GetNdoc_w();
  }

  int FrOffshoreSystem::GetNbDOF() const {
    return m_chronoSystem->GetNdof();
  }

  int FrOffshoreSystem::GetNbBodies() const {
    return m_chronoSystem->GetNbodies();
  }

  int FrOffshoreSystem::GetNbFixedBodies() const {
    return m_chronoSystem->GetNbodiesFixed();
  }

  int FrOffshoreSystem::GetNbSleepingBodies() const {
    return m_chronoSystem->GetNbodiesSleeping();
  }

  void FrOffshoreSystem::SetUseSleepingBodies(bool useSleeping) {
    m_chronoSystem->SetUseSleeping(useSleeping);
  }

  double FrOffshoreSystem::GetGravityAcceleration() const {
    return fabs(m_chronoSystem->Get_G_acc()[2]);
  }

  void FrOffshoreSystem::SetGravityAcceleration(double gravityAcceleration) {
    event_logger::info(GetTypeName(), GetName(),
                       "Gravity acceleration set to {} m/s2", gravityAcceleration);
    m_chronoSystem->Set_G_acc(chrono::ChVector<double>(0., 0., -gravityAcceleration));
  }

  bool FrOffshoreSystem::DoAssembly() {
    event_logger::info(GetTypeName(), GetName(), "Solving assembly");
    return m_chronoSystem->DoFullAssembly();
  }

  FrStaticAnalysis *FrOffshoreSystem::GetStaticAnalysis() const {
    return m_statics.get();
  }

  bool FrOffshoreSystem::SolveStaticWithRelaxation() {

    event_logger::info(GetTypeName(), GetName(), "Static analysis by dynamic relaxation STARTED");
    Initialize();
    return m_statics->SolveStatic();
  }

  void FrOffshoreSystem::Relax(FrStaticAnalysis::RELAXTYPE relax) {

    for (auto &body:m_bodyList) {
      switch (relax) {
        case FrStaticAnalysis::NORELAX :
          break;
        case FrStaticAnalysis::VELOCITY :
          body->SetVelocityInWorldNoRotation(Velocity(), NWU);
          break;
        case FrStaticAnalysis::ACCELERATION :
          body->SetAccelerationInBodyNoRotation(Acceleration(), NWU);
          break;
        case FrStaticAnalysis::VELOCITYANDACCELERATION :
          body->SetVelocityInWorldNoRotation(Velocity(), NWU);
          body->SetAccelerationInBodyNoRotation(Acceleration(), NWU);
          break;
      }
    }

    for (auto &mesh : m_feaMeshList) {
      mesh->Relax();
    }

  }

  void FrOffshoreSystem::SetTimeStepper(TIME_STEPPER type, bool check_compatibility) {

    using timeStepperType = chrono::ChTimestepper::Type;

    std::string timestepper_str;

    switch (type) {
      case EULER_IMPLICIT_LINEARIZED:
        m_chronoSystem->SetTimestepperType(timeStepperType::EULER_IMPLICIT_LINEARIZED);
        timestepper_str = "EULER_IMPLICIT_LINEARIZED";
        break;
      case EULER_IMPLICIT_PROJECTED:
        m_chronoSystem->SetTimestepperType(timeStepperType::EULER_IMPLICIT_PROJECTED);
        timestepper_str = "EULER_IMPLICIT_PROJECTED";
        break;
//      case EULER_IMPLICIT:
//        m_chronoSystem->SetTimestepperType(timeStepperType::EULER_IMPLICIT);
//        timestepper_str = "EULER_IMPLICIT";
//        break;
      case TRAPEZOIDAL:
        m_chronoSystem->SetTimestepperType(timeStepperType::TRAPEZOIDAL);
        timestepper_str = "TRAPEZOIDAL";
        break;
      case TRAPEZOIDAL_LINEARIZED:
        m_chronoSystem->SetTimestepperType(timeStepperType::TRAPEZOIDAL_LINEARIZED);
        timestepper_str = "TRAPEZOIDAL_LINEARIZED";
        break;
      case HHT:
        m_chronoSystem->SetTimestepperType(timeStepperType::HHT);
        timestepper_str = "HHT";
        break;
      case RUNGEKUTTA45:
        m_chronoSystem->SetTimestepperType(timeStepperType::RUNGEKUTTA45);
        timestepper_str = "RUNGEKUTTA45";
        break;
      case EULER_EXPLICIT:
        m_chronoSystem->SetTimestepperType(timeStepperType::EULER_EXPLICIT);
        timestepper_str = "EULER_EXPLICIT";
        break;
      case NEWMARK:
        m_chronoSystem->SetTimestepperType(timeStepperType::NEWMARK);
        timestepper_str = "NEWMARK";
        break;
    }

    m_timeStepper = type;

    event_logger::info(GetTypeName(), GetName(), "Time stepper set to {}", timestepper_str);

    if (check_compatibility) CheckCompatibility();

  }

  void FrOffshoreSystem::SetTimeStepper(TIME_STEPPER type) {
    SetTimeStepper(type, true);
  }

  void FrOffshoreSystem::SetTimeStep(double timeStep) {
    m_chronoSystem->SetStep(timeStep);
    event_logger::info(GetTypeName(), GetName(), "Time step set to {} s", timeStep);
  }

  double FrOffshoreSystem::GetTimeStep() const {
    return m_chronoSystem->GetStep();
  }

  void FrOffshoreSystem::SetMinTimeStep(double minTimeStep) {
    m_chronoSystem->SetStepMin(minTimeStep);
    event_logger::info(GetTypeName(), GetName(), "Set minimum time step to {} s", minTimeStep);
  }

  void FrOffshoreSystem::SetMaxTimeStep(double maxTimeStep) {
    m_chronoSystem->SetStepMax(maxTimeStep);
    event_logger::info(GetTypeName(), GetName(), "Set maximum time step to {} s", maxTimeStep);
  }

  double FrOffshoreSystem::GetTime() const {
    return m_chronoSystem->GetChTime();
  }

  void FrOffshoreSystem::SetTime(double time) {
    m_chronoSystem->SetChTime(time);
  }

  bool FrOffshoreSystem::AdvanceOneStep(double stepSize) {
    Initialize();
    auto is_advance =  m_chronoSystem->DoStepDynamics(stepSize);
    m_chronoSystem->Update(true);
    return is_advance;
  }

  bool FrOffshoreSystem::AdvanceTo(double nextTime) {
    Initialize();
    return m_chronoSystem->DoFrameDynamics(nextTime);
  }

  bool FrOffshoreSystem::RunDynamics(double frameStep) {
    Initialize();
    event_logger::info(GetTypeName(), GetName(), "Dynamic simulation STARTED");
    event_logger::switch_to_simulation_formatter(this);
    event_logger::flush();
    m_chronoSystem->Setup();
    m_chronoSystem->DoAssembly(chrono::AssemblyLevel::POSITION |
                               chrono::AssemblyLevel::VELOCITY |
                               chrono::AssemblyLevel::ACCELERATION);

    while (true) {
      double nextTime = m_chronoSystem->GetChTime() + frameStep;
      if (!AdvanceTo(nextTime))
        break;
    }
    FinalizeDynamicSimulation();
    return true;
  }

  void FrOffshoreSystem::CreateWorldBody() {
    m_worldBody = std::make_shared<FrBody>("world_body", this);
    m_worldBody->SetFixedInWorld(true);
//      m_worldBody->SetName("WorldBody");
//      m_worldBody->SetLogged(false);
//    switch (m_systemType) {
//      case SMOOTH_CONTACT:
//        m_worldBody->SetSmoothContact();
//        break;
//      case NONSMOOTH_CONTACT:
//        m_worldBody->SetNonSmoothContact();
//        break;
//    }
    Add(m_worldBody);
    m_worldBody->LogThis(false);  // No log for the world body
  }

  std::shared_ptr<FrBody> FrOffshoreSystem::NewBody(const std::string &name) {
    auto body = std::make_shared<FrBody>(name, this);
    // TODO : suivant le type de systeme SMC ou NSC, regler le type de surface...

//    switch (m_systemType) {
//      case SMOOTH_CONTACT:
//        body->SetSmoothContact();
//        break;
//      case NONSMOOTH_CONTACT:
//        body->SetNonSmoothContact();
//        break;
//    }

    Add(body);
    return body;
  }

  void FrOffshoreSystem::Clear() {
    m_chronoSystem->Clear();

    m_bodyList.clear();
    m_linkList.clear();
    m_constraintList.clear();
    m_actuatorList.clear();
    m_feaMeshList.clear();
    m_physicsItemsList.clear();

    m_isInitialized = false;
  }


#ifndef H5_NO_IRRLICHT
// Irrlicht visualization

  FrIrrApp *FrOffshoreSystem::GetIrrApp() const {
    return m_irrApp.get();
  }

  void FrOffshoreSystem::RunInViewer(double endTime, double dist, bool recordVideo, int videoFrameSaveInterval) {

    // Initialization of the system if not already done.
    Initialize();

    // Definition and initialization of the Irrlicht application.
    m_irrApp = std::make_unique<FrIrrApp>(this, m_chronoSystem.get(), dist);

    m_irrApp->SetTimestep(m_chronoSystem->GetStep());
    m_irrApp->SetVideoframeSave(recordVideo);
    m_irrApp->SetVideoframeSaveInterval(videoFrameSaveInterval);

    event_logger::info(GetTypeName(), GetName(),
                       "Dynamic simulation STARTED in viewer with endTime = {} s, video recording set to {}",
                       endTime, recordVideo);
    event_logger::switch_to_simulation_formatter(this);
    event_logger::flush();

    m_irrApp->Run(endTime); // The temporal loop is here.

    FinalizeDynamicSimulation();

  }

  void FrOffshoreSystem::RunInViewer(double endTime, double dist, bool recordVideo) {
    RunInViewer(endTime, dist, recordVideo, 10);
  }

  void FrOffshoreSystem::RunInViewer(double endTime, double dist) {
    RunInViewer(endTime, dist, false);
  }

  void FrOffshoreSystem::RunInViewer(double endTime) {
    RunInViewer(endTime, 100);
  }

  void FrOffshoreSystem::RunInViewer() {
    RunInViewer(0);
  }

  void FrOffshoreSystem::Visualize(double dist, bool recordVideo) {

    Initialize();

    FrIrrApp app(this, m_chronoSystem.get(), dist);

    app.SetTimestep(m_chronoSystem->GetStep());
    app.SetVideoframeSave(recordVideo);
    app.Visualize();

  }

  void FrOffshoreSystem::Visualize(double dist) {
    Visualize(dist, false);
  }

  void FrOffshoreSystem::Visualize() {
    Visualize(100);
  }

  void FrOffshoreSystem::VisualizeStaticAnalysis(double dist, bool recordVideo) {

    Initialize();  // So that system is automatically initialized when run in viewer mode

    FrIrrApp app(this, m_chronoSystem.get(), dist);

    app.SetTimestep(m_chronoSystem->GetStep());
    app.SetVideoframeSave(recordVideo);
    app.VisualizeStaticAnalysis();

  }

  void FrOffshoreSystem::VisualizeStaticAnalysis(double dist) {
    VisualizeStaticAnalysis(dist, false);
  }

  void FrOffshoreSystem::VisualizeStaticAnalysis() {
    VisualizeStaticAnalysis(100);
  }
#endif


  FrLogManager *FrOffshoreSystem::GetLogManager() const {
    return m_logManager.get();
  }

  FrPathManager *FrOffshoreSystem::GetPathManager() const {
    return m_pathManager.get();
  }

  void FrOffshoreSystem::FinalizeDynamicSimulation() const {
    event_logger::back_to_default_formatter(GetName());
    event_logger::info(GetTypeName(), GetName(),
                       "END OF DYNAMIC SIMULATION AT SIMULATION TIME: {} s", GetTime());
  }

// Iterators

  FrOffshoreSystem::BodyIter FrOffshoreSystem::body_begin() {
    return m_bodyList.begin();
  }

  FrOffshoreSystem::ConstBodyIter FrOffshoreSystem::body_begin() const {
    return m_bodyList.cbegin();
  }

  FrOffshoreSystem::BodyIter FrOffshoreSystem::body_end() {
    return m_bodyList.end();
  }

  FrOffshoreSystem::ConstBodyIter FrOffshoreSystem::body_end() const {
    return m_bodyList.cend();
  }

  FrOffshoreSystem::LinkIter FrOffshoreSystem::link_begin() {
    return m_linkList.begin();
  }

  FrOffshoreSystem::ConstLinkIter FrOffshoreSystem::link_begin() const {
    return m_linkList.cbegin();
  }

  FrOffshoreSystem::LinkIter FrOffshoreSystem::link_end() {
    return m_linkList.end();
  }

  FrOffshoreSystem::ConstLinkIter FrOffshoreSystem::link_end() const {
    return m_linkList.cend();
  }

  FrOffshoreSystem::ConstraintIter FrOffshoreSystem::constraint_begin() {
    return m_constraintList.begin();
  }

  FrOffshoreSystem::ConstConstraintIter FrOffshoreSystem::constraint_begin() const {
    return m_constraintList.cbegin();
  }

  FrOffshoreSystem::ConstraintIter FrOffshoreSystem::constraint_end() {
    return m_constraintList.end();
  }

  FrOffshoreSystem::ConstConstraintIter FrOffshoreSystem::constraint_end() const {
    return m_constraintList.cend();
  }

  FrOffshoreSystem::ActuatorIter FrOffshoreSystem::actuator_begin() {
    return m_actuatorList.begin();
  }

  FrOffshoreSystem::ConstActuatorIter FrOffshoreSystem::actuator_begin() const {
    return m_actuatorList.cbegin();
  }

  FrOffshoreSystem::ActuatorIter FrOffshoreSystem::actuator_end() {
    return m_actuatorList.end();
  }

  FrOffshoreSystem::ConstActuatorIter FrOffshoreSystem::actuator_end() const {
    return m_actuatorList.cend();
  }

  FrOffshoreSystem::PhysicsIter FrOffshoreSystem::physics_item_begin() {
    return m_physicsItemsList.begin();
  }

  FrOffshoreSystem::ConstPhysicsIter FrOffshoreSystem::physics_item_begin() const {
    return m_physicsItemsList.cbegin();
  }

  FrOffshoreSystem::PhysicsIter FrOffshoreSystem::physics_item_end() {
    return m_physicsItemsList.end();
  }

  FrOffshoreSystem::ConstPhysicsIter FrOffshoreSystem::physics_item_end() const {
    return m_physicsItemsList.cend();
  }


  void FrOffshoreSystem::SetSolverVerbose(bool verbose) {
    m_chronoSystem->GetSolver()->SetVerbose(verbose);
    dynamic_cast<chrono::ChIterativeSolverVI *>(m_chronoSystem->GetSolver().get())->SetRecordViolation(verbose);
  }


  bool FrOffshoreSystem::Add(std::shared_ptr<FrTreeNodeBase> item) {

    // FIXME : mettre des gardes au cas ou RegisterTreeNode renvoie false !!!

    bool added = true;

    // BODY
    if (auto body = std::dynamic_pointer_cast<FrBody>(item)) {
      AddBody(body);
      m_pathManager->RegisterTreeNode(body.get());

      // LINK
    } else if (auto link = std::dynamic_pointer_cast<FrLink>(item)) {
      AddLink(link);
      m_pathManager->RegisterTreeNode(link.get());

      // CONSTRAINT
    } else if (auto constraint = std::dynamic_pointer_cast<FrConstraint>(item)) {
      AddConstraint(constraint);
      m_pathManager->RegisterTreeNode(constraint.get());

      // ACTUATOR
    } else if (auto actuator = std::dynamic_pointer_cast<FrActuator>(item)) {
      AddActuator(actuator);
      m_pathManager->RegisterTreeNode(actuator.get());

      // CATENARY LINE
      // MUST BE BEFORE PHYSICS ITEM
    } else if (auto catenary_line = std::dynamic_pointer_cast<FrCatenaryLineBase>(item)) {
      AddCatenaryLineBase(catenary_line);
      m_pathManager->RegisterTreeNode(catenary_line.get());

    } else if (auto equilibrium_frame = std::dynamic_pointer_cast<FrEquilibriumFrame>(item)) {
      AddEquilibriumFrame(equilibrium_frame);
      m_pathManager->RegisterTreeNode(equilibrium_frame.get());

    } else if (auto hydro_mesh = std::dynamic_pointer_cast<FrHydroMesh>(item)) {
      AddHydroMesh(hydro_mesh);
      m_pathManager->RegisterTreeNode(hydro_mesh.get());
//
//    } else if (auto catenary_line_seabed = std::dynamic_pointer_cast<FrCatenaryLineSeabed>(item)) {
//      AddPhysicsItem(catenary_line_seabed);
//      m_pathManager->RegisterTreeNode(catenary_line_seabed.get());
//
//    } else if (auto equilibrium_frame = std::dynamic_pointer_cast<FrEquilibriumFrame>(item)) {
//      AddPhysicsItem(equilibrium_frame);
//      m_pathManager->RegisterTreeNode(equilibrium_frame.get());

      // MORISON MODEL
    } else if (auto morison = std::dynamic_pointer_cast<FrMorisonCompositeElement>(item)) {
      AddMorisonElements(morison);
      m_pathManager->RegisterTreeNode(morison.get());

      // RADIATION MODEL
    } else if (auto model = std::dynamic_pointer_cast<FrRadiationModel>(item)) {
      AddRadiationModel(model);
      m_pathManager->RegisterTreeNode(model.get());


//      //PHYSICS ITEM
//    } else if (auto physics_item = std::dynamic_pointer_cast<FrPhysicsItem>(item)) {
//      AddPhysicsItem(physics_item);
//      m_pathManager->RegisterTreeNode(physics_item.get());


      // FEA CABLE
      // MUST BE BEFORE FEAMESH CASE (dynamic cable is also feamesh, however the AddFEACable also add the hinges)
    } else if (auto fea_cable = std::dynamic_pointer_cast<FrFEACable>(item)) {
      AddFEACable(fea_cable);
      m_pathManager->RegisterTreeNode(fea_cable.get());
      //##CC
      std::cout << "debug : FrOffshoreSystem : add fea cable" << std::endl;
      auto fea_mesh = internal::GetChronoFEAMesh(fea_cable);
      auto start_link = fea_mesh->GetStartLink();
      auto end_link = fea_mesh->GetEndLink();
      m_pathManager->RegisterTreeNode(start_link.get());
      m_pathManager->RegisterTreeNode(end_link.get());
      std::cout << "debug : FrOffshoreSystem : ad fea link" << std::endl;

      if (auto loggable = std::dynamic_pointer_cast<FrLoggableBase>(start_link)) {
        m_logManager->Add(loggable);
      }
      if (auto loggable = std::dynamic_pointer_cast<FrLoggableBase>(end_link)) {
        m_logManager->Add(loggable);
      }
      //##

    } else if (auto clump_weight = std::dynamic_pointer_cast<FrClumpWeight>(item)) {
      AddClumpWeight(clump_weight);
      m_pathManager->RegisterTreeNode(clump_weight.get());

//      // FEA MESH
//    } else if (auto fea_mesh = std::dynamic_pointer_cast<FrFEAMesh>(item)) {
//      AddFEAMesh(fea_mesh, fea_mesh->GetFEAMeshBase());
////      m_pathManager->RegisterTreeNode(fea_mesh.get());

      // LUMPED MASS NODE
//    } else if (auto lumped_mass_node = std::dynamic_pointer_cast<internal::FrLMNode>(item)) {
//      AddLumpedMassNode(lumped_mass_node);
//
//      // LUMPED MASS ELEMENT
//    } else if (auto lumped_mass_element = std::dynamic_pointer_cast<internal::FrLMElement>(item)) {
//      AddLumpedMassElement(lumped_mass_element);
//
//      // LUMPED MASS CABLE
//    } else if (auto lumped_mass_cable = std::dynamic_pointer_cast<FrLumpedMassCable>(item)) {
//      AddLumpedMassCable(lumped_mass_cable);
//      m_pathManager->RegisterTreeNode(lumped_mass_cable.get());

      // UNKNOWN
    } else {
      added = false;
      event_logger::info(GetTypeName(), GetName(), "object added to the system not recognized : {}", item->GetName());
    }
//    else {
//      std::cerr << "Unknown object type " << std::endl;
//      event_logger::error(GetTypeName(), GetName(),
//          "Trying to add object {} with unknown type", item->GetName());
//      exit(EXIT_FAILURE);
//    }

    if (added) {
//      m_pathManager->RegisterTreeNode(item.get());

      if (auto loggable = std::dynamic_pointer_cast<FrLoggableBase>(item)) {
        m_logManager->Add(loggable);
      }
    }

    return added;
  }

  void FrOffshoreSystem::Remove(std::shared_ptr<FrTreeNodeBase> item) {

    // BODY
    if (auto body = std::dynamic_pointer_cast<FrBody>(item)) {
      RemoveBody(body);

      // LINK
    } else if (auto link = std::dynamic_pointer_cast<FrLink>(item)) {
      RemoveLink(link);

      // CONSTRAINT
    } else if (auto constraint = std::dynamic_pointer_cast<FrConstraint>(item)) {
      RemoveConstraint(constraint);

      // ACTUATOR
    } else if (auto actuator = std::dynamic_pointer_cast<FrActuator>(item)) {
      RemoveActuator(actuator);

      // HYDRO MESH
    } else if (auto hydro_mesh = std::dynamic_pointer_cast<FrHydroMesh>(item)) {
      RemoveHydroMesh(hydro_mesh);

      // UNKNOWN
    } else {
      std::cerr << "FrOffshoreSystem::Remove : Unknown object type " << std::endl;
      exit(EXIT_FAILURE);
    }

    if (auto loggable = std::dynamic_pointer_cast<FrLoggableBase>(item)) {
      m_logManager->Remove(loggable);
    }
  }

}  // end namespace frydom
