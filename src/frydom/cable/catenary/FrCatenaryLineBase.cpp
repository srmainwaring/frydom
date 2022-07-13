//
// Created by frongere on 03/03/2020.
//

#include "FrCatenaryLineBase.h"

#include "frydom/logging/FrTypeNames.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/core/common/FrNode.h"

#include "frydom/cable/common/FrCableProperties.h"


namespace frydom {


  FrCatenaryForce::FrCatenaryForce(const std::string &name,
                                   FrBody *body,
                                   FrCatenaryLineBase *line,
                                   FrCatenaryLineBase::LINE_SIDE side) :
      FrForce(name, TypeToString(this), body),
      m_line(line),
      m_line_side(side) {}

  bool FrCatenaryForce::IncludedInStaticAnalysis() const { return true; }

  void FrCatenaryForce::Compute(double time) {

    //##CC
    std::cout << "debug : FrCatenaryForce : Compute " << std::endl;
    //##CC

    Position relative_position;
    Force force_in_world;

    // Get the line tension from the corresponding node
    switch (m_line_side) {
      case FrCatenaryLineBase::LINE_START:
        force_in_world = m_line->GetTension(0., NWU);
        relative_position = m_line->GetPositionInWorld(0., NWU);
        break;

      case FrCatenaryLineBase::LINE_END:
        double L = m_line->GetUnstretchedLength();
        force_in_world = -m_line->GetTension(L, NWU);
        relative_position = m_line->GetPositionInWorld(L, NWU);
        break;
    }

    // Set the tension in the world reference frame and NWU frame convention
    SetForceTorqueInWorldAtPointInWorld(force_in_world, Torque(), relative_position, NWU);

  }

  void FrCatenaryLineBase::Initialize() {

    if (!m_use_for_shape_initialization) {
      // Building the catenary forces and adding them to bodies
      if (!m_startingForce) {
        m_startingForce = std::make_shared<FrCatenaryForce>(GetName() + "_start_force", m_startingNode->GetBody(), this,
                                                            FrCatenaryLineBase::LINE_START);
        auto starting_body = m_startingNode->GetBody();
        starting_body->AddExternalForce(m_startingForce);
      }

      if (!m_endingForce) {
        m_endingForce = std::make_shared<FrCatenaryForce>(GetName() + "_end_force", m_endingNode->GetBody(), this,
                                                          FrCatenaryLineBase::LINE_END);
        auto ending_body = m_endingNode->GetBody();
        ending_body->AddExternalForce(m_endingForce);
      }

      FrCatenaryAssetOwner::Initialize();
    }

    FrCableBase::Initialize();
  }

  void FrCatenaryLineBase::StepFinalize() {
    UpdateAsset();
  }

  FrCatenaryLineBase::FrCatenaryLineBase(const std::string &name, const std::string &type,
                                         const std::shared_ptr<FrNode> &startingNode,
                                         const std::shared_ptr<FrNode> &endingNode,
                                         const std::shared_ptr<FrCableProperties> &properties, bool elastic,
                                         double unstretchedLength) :
      FrLoggable(name, type, startingNode->GetSystem()),
      FrPhysicsItem(),
      FrCableBase(startingNode, endingNode, properties, unstretchedLength),
      m_elastic(elastic),
      m_use_for_shape_initialization(false),
      m_tolerance(1e-6),
      m_maxiter(100),
      m_q(0.),
      m_startingForce(nullptr),
      m_endingForce(nullptr) {}

  void FrCatenaryLineBase::SetSolverTolerance(double tol) { m_tolerance = tol; }

  void FrCatenaryLineBase::SetSolverMaxIter(unsigned int maxiter) { m_maxiter = maxiter; }

  void FrCatenaryLineBase::UseForShapeInitialization(bool use) {
    m_use_for_shape_initialization = use;
  }

  std::shared_ptr<FrCatenaryForce> FrCatenaryLineBase::GetStartingForce() {
    return m_startingForce;
  }

  std::shared_ptr<FrCatenaryForce> FrCatenaryLineBase::GetEndingForce() {
    return m_endingForce;
  }

  double FrCatenaryLineBase::GetTotalMass() const {
    return GetUnstretchedLength() * m_properties->GetLinearDensity();
  }

}  // end namespace frydom
