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

#include "FrCatenaryLine.h"
#include "frydom/cable/common/FrCableProperties.h"

#include "frydom/core/common/FrNode.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"

#include "frydom/logging/FrTypeNames.h"
#include "frydom/logging/FrEventLogger.h"

namespace frydom {

  FrCatenaryLine::FrCatenaryLine(const std::string &name,
                                 const std::shared_ptr<FrNode> &startingNode,
                                 const std::shared_ptr<FrNode> &endingNode,
                                 const std::shared_ptr<FrCableProperties> &properties,
                                 bool elastic,
                                 double unstretchedLength,
                                 FLUID_TYPE fluid_type) :
      FrCatenaryLineBase(name,
                         TypeToString(this),
                         startingNode,
                         endingNode,
                         properties,
                         elastic,
                         unstretchedLength),
      c_qL(0.), c_fluid(fluid_type) {
    m_point_forces.emplace_back(internal::PointForce{this, 0., Force()});
  }

  FrCatenaryLine::FrCatenaryLine(const std::string &name, FrCableBase *cable, bool elastic, FLUID_TYPE fluid_type) :
      FrCatenaryLine(name,
                     cable->GetStartingNode(),
                     cable->GetEndingNode(),
                     cable->GetProperties(),
                     elastic,
                     cable->GetUnstretchedLength(),
                     fluid_type) {}

  void FrCatenaryLine::AddClumpWeight(double s, const double &mass, bool reversed) {
    if (reversed) s = m_unstretchedLength - s;
    AddPointMass(s / m_unstretchedLength,
                 Force(0, 0, -mass * GetSystem()->GetEnvironment()->GetGravityAcceleration()));
  }

  void FrCatenaryLine::AddBuoy(double s, const double &mass, bool reversed) {
    if (reversed) s = m_unstretchedLength - s;
    AddPointMass(s / m_unstretchedLength,
                 Force(0, 0, mass * GetSystem()->GetEnvironment()->GetGravityAcceleration()));
  }

  bool FrCatenaryLine::IsSingular() const {
    return rho(0, 0.) == 0. || rho(N(), 1.) == 0.; // TODO: voir dans catway pour les tests...
  }

  void FrCatenaryLine::Initialize() {

    //##CC
    std::cout << "debug : FrCatenaryLine : Initialize" << std::endl;
    //##

    // TODO : check avec Francois pour l'hydrostatique
    // TODO : ajouter un assert sur la grandeur de q
    m_q = m_properties->GetLinearDensity() -
          m_properties->GetSectionArea() * GetSystem()->GetEnvironment()->GetFluidDensity(c_fluid);
    m_q *= GetSystem()->GetGravityAcceleration();
//    m_q = m_properties->GetLinearDensity() *
//          GetSystem()->GetEnvironment()->GetGravityAcceleration(); // FIXME: reintroduire l'hydrostatique !!!
    m_pi = {0., 0., -1.}; // FIXME: en dur pour le moment... (voir aussi dans FrCatenaryLineSeabed::Initialize())


    m_startingNode->Initialize();
    m_endingNode->Initialize();

    BuildCache();

    FirstGuess();
    solve();

    FrCatenaryLineBase::Initialize();
  }
//  void FrCatenaryLine::StepFinalize() {}

  Force FrCatenaryLine::GetTension(const double &s, FRAME_CONVENTION fc) const {
    Tension tension = t(s / m_unstretchedLength) * c_qL;
    if (IsNED(fc))
      internal::SwapFrameConvention(tension);
    return tension;
  }

  Force FrCatenaryLine::GetStartingNodeTension(FRAME_CONVENTION fc) const {
    return GetTension(0, NWU);
  }

  Force FrCatenaryLine::GetEndingNodeTension(FRAME_CONVENTION fc) const {
    return -GetTension(GetUnstretchedLength(), NWU);
  }

  Direction FrCatenaryLine::GetTangent(const double s, FRAME_CONVENTION fc) const {
    Direction direction = t(s / m_unstretchedLength).normalized();
    if (IsNED(fc)) {
      internal::SwapFrameConvention(direction);
    }
    return direction;
  }

  Position FrCatenaryLine::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
    Position position;
    p(s / m_unstretchedLength, position);
    if (IsNED(fc))
      internal::SwapFrameConvention(position);

    Position pos = position * m_unstretchedLength; // FIXME retirer
    return m_startingNode->GetPositionInWorld(fc) + position * m_unstretchedLength;
  }

  double FrCatenaryLine::GetUnstretchedLength() const {
    return m_unstretchedLength;
  }

  void FrCatenaryLine::solve() {

    // Defining the linear solver
    Eigen::FullPivLU<Eigen::Matrix3d> linear_solver;

    Tension t0_prec = m_t0;

    Residue3 residue = GetResidue();

    linear_solver.compute(GetJacobian());
    Tension dt0 = linear_solver.solve(-residue);

    m_t0 += dt0;

    double tension_criterion = ((m_t0.array() - t0_prec.array()).abs() /
                                (1. + m_t0.array().abs().min(t0_prec.array().abs()))).maxCoeff();

    double pos_error = residue.array().abs().maxCoeff();

    unsigned int iter = 1;

    while (tension_criterion > m_tolerance && iter < m_maxiter) {
      iter++;
      residue = GetResidue();
      linear_solver.compute(GetJacobian());
      dt0 = linear_solver.solve(-residue);
      t0_prec = m_t0;
      m_t0 += dt0;

      pos_error = residue.array().abs().maxCoeff();
      tension_criterion = ((m_t0.array() - t0_prec.array()).abs() /
                           (1. + m_t0.array().abs().min(t0_prec.array().abs()))).maxCoeff();

    }

    c_iter = iter;
    c_pos_error = pos_error;
    c_criterion = tension_criterion;

    if (iter == m_maxiter) {
      event_logger::warn(GetTypeName(), GetName(),
                         "No convergence of the solver after {} max iterations. Position error is {} m.",
                         m_maxiter, pos_error);
    } else {
//      std::cout << "CONVERGENCE IN " << iter << std::endl;
    }



//    unsigned int iter = 0;
//
//    // Defining the linear solver
//    Eigen::FullPivLU<Eigen::Matrix3d> linear_solver;
//
//    Residue3 residue = GetResidue();
//    double err = residue.norm();
//
//    if (err < m_tolerance) {
//      // Already at equilibrium
//      return;
//    }
//
//    while (iter < m_maxiter) {
//
//      iter++;
//
//      linear_solver.compute(GetJacobian());
//      Tension dt0 = linear_solver.solve(-residue);
//
//      m_t0 += dt0;
//
//      residue = GetResidue();
//      err = residue.norm();
//
//      if (err < m_tolerance) {
//        break;
//      }
//
//    }
//
//    if (iter == m_maxiter) {
//      event_logger::warn(GetTypeName(), GetName(),
//                         "No convergence of the solver after {} max iterations", m_maxiter);
//    } else {
////      std::cout << "CONVERGENCE IN " << iter << std::endl;
//    }

  }

  bool FrCatenaryLine::HasSeabedInteraction() const {
    double s;
    Position lowest_position;
    GetLowestPoint(lowest_position, s, NWU, 1e-3, 20);

    return !(GetSystem()->GetEnvironment()->GetOcean()->GetSeabed()->IsAboveSeabed(lowest_position, NWU));
  }

  void FrCatenaryLine::GetLowestPoint(Position &position,
                                      double &s,
                                      FRAME_CONVENTION fc,
                                      const double tol,
                                      const unsigned int maxIter) const {

    // Using a bisection algorithm to find the lowest point on the catenary line

    double s0 = 0.;
    double s1 = m_unstretchedLength;

    auto p0 = GetStartingNode()->GetPositionInWorld(fc);
    auto p1 = GetEndingNode()->GetPositionInWorld(fc);

    double dz0 = GetTangent(s0, fc).z();
    double dz1 = GetTangent(s1, fc).z();

    // Dealing with border cases where the minimum point is at one of the boundary node
    if (dz0 * dz1 > 0.) {
      if (dz0 > 0.) {
        position = p0;
        s = s0;
      } else {
        position = p1;
        s = s1;
      }
      return;
    } else if (dz0 == 0.) {
      position = p0;
      s = s0;
      return;
    } else if (dz1 == 0.) {
      position = p1;
      s = s1;
      return;
    }

    // Bisection algorithm
    unsigned int iter = 0;
    double dz = 0.;
    while (s1 - s0 > tol && iter < maxIter) {  // FIXME : dz ne change pas pendant les iterations !!!!!

      iter++;

      s = s0 + 0.5 * (s1 - s0);
      position = GetPositionInWorld(s, fc);

      dz = GetTangent(s, fc)[2];

      // See if derivative in z has changed sign
      if (dz0 * dz < 0) {
        s1 = s;
        p1 = position;
        dz1 = dz;
      } else if (dz1 * dz < 0) {
        s0 = s;
        p0 = position;
        dz0 = dz;
      } else break; // Tangent is horizontal, this is the minimum...

    }

    if (iter == maxIter) {
      event_logger::warn(GetTypeName(), GetName(),
                         "Maximum iteration in bisection has been reached while computing the lowest position in the element!");
    }
  }

  void FrCatenaryLine::AddPointMass(const double &s, const Force &force) {
    // TODO:gerer le fait qu'on decrit s a partir du fairlead ???
    auto pos = m_point_forces.cbegin();
    for (; pos != m_point_forces.cend(); pos++) {
      if (s > pos->s())
        break;
    }
    pos++;
    m_point_forces.insert(pos, internal::PointForce(this, s, force));
    BuildCache();
  }

  Force FrCatenaryLine::ti(const unsigned int &i, const double &s) const {
    return m_t0 - Fi(i) - m_pi * s;
  }

  double FrCatenaryLine::rho(const unsigned int &i, const double &s) const {
    return ti(i, s).norm() - m_pi.transpose() * ti(i, s); // On essaie d'exploiter Eigen...
  }

  Force FrCatenaryLine::Fi(const unsigned int &i) const {
    return c_Fi[i];
  }

  Force FrCatenaryLine::fi(const unsigned int &i) const {
    return m_point_forces[i].force();
  }

  double FrCatenaryLine::si(const unsigned int &i) const {
    return m_point_forces[i].s();
  }

  unsigned int FrCatenaryLine::N() const { return m_point_forces.size() - 1; }

  FrCatenaryLineBase::Tension FrCatenaryLine::t(const double &s) const {
    unsigned int i = SToI(s);
    return ti(i, s);
  }

  FrCatenaryLineBase::Tension FrCatenaryLine::tL() const {
    return t(1.);
  }

  unsigned int FrCatenaryLine::SToI(const double &s) const {
    assert(0. <= s && s <= 1.);
    // This method has a cost that is linear with respect to the number of localized forces.
    for (unsigned int j = 1; j <= N(); j++) {
      if (s <= si(j)) return j - 1;
    }
    return N();
  }

  void FrCatenaryLine::p_pi(Position &position, const unsigned int &i, const double &s) const {
    double scal = ti(i, s).norm() - ti(i, si(i)).norm();
    for (int j = 0; j < i; j++) {
      scal += ti(j, si(j + 1)).norm() - ti(j, si(j)).norm();
    }
    position -= m_pi * scal;
  }

  void FrCatenaryLine::p_perp(Position &position, const unsigned int &i, const double &s) const {
    Position vector = (m_t0 - Fi(i)) * std::log(rho(i, s) / rho(i, si(i)));
    for (int j = 0; j < i; j++) {
      vector += (m_t0 - Fi(j)) * std::log(rho(j, si(j + 1)) / rho(j, si(j)));
    }
    position += c_U * vector;
  }

  void FrCatenaryLine::pc(Position &position, const unsigned int &i, const double &s) const {
    p_pi(position, i, s);
    if (!IsSingular()) {
      p_perp(position, i, s);
    }
  }

  Force FrCatenaryLine::sum_fs(const unsigned int &i) const {
    return c_sum_fs[i];
  }

  void FrCatenaryLine::pe(Position &position, const unsigned int &i, const double &s) const {
    position += (c_qL / m_properties->GetEA()) * ((m_t0 - Fi(i)) * s + sum_fs(i) - 0.5 * m_pi * s * s);
  }

  void FrCatenaryLine::pi(Position &position, const unsigned int &i, const double &s) const {
    pc(position, i, s);
    if (m_elastic)
      pe(position, i, s);
  }

  void FrCatenaryLine::p(const double &s, Position &position) const {
    unsigned int i = SToI(s);
    pi(position, i, s);
  }

  Position FrCatenaryLine::p(const double &s) const {
    Position position;
    p(s, position);
    return position;
  }

  Position FrCatenaryLine::pL() const {
    return (m_endingNode->GetPositionInWorld(NWU) - m_startingNode->GetPositionInWorld(NWU)) / m_unstretchedLength;
  }

  void FrCatenaryLine::FirstGuess() {
    // Peyrot et goulois

    Position p0pL = m_endingNode->GetPositionInWorld(NWU) - m_startingNode->GetPositionInWorld(NWU);
    double lx = p0pL(0);
    double ly = p0pL(1);
    double lz = p0pL(2);

//    double V = lz;
    double H = sqrt(lx * lx + ly * ly);

    auto chord_length = p0pL.norm();
    Direction v_horiz = (m_pi.cross(p0pL).cross(m_pi)).normalized();

    double lu = GetUnstretchedLength();
    double lambda = 0;

    if (lu <= chord_length) {
      // The cable is taut
      lambda = 0.2;
    } else if ((m_pi.cross(p0pL)).norm() < 1e-4) {
      // The cable is collinear to main load
      lambda = 1e6;
    } else {
      lambda = sqrt(6. * (sqrt((lu * lu - lz * lz) / (lx * lx + ly * ly)) - 1.));
    }

    double f_vert = 0.5 * m_q * (-lz / tanh(lambda) + lu);
    double f_horiz = std::max(0.5 * m_q * H / lambda, 1e-3); // Attention au signe

    m_t0 = (f_vert * m_pi + f_horiz * v_horiz) / c_qL;
  }

  FrCatenaryLine::Residue3 FrCatenaryLine::GetResidue() const {
    return p(1.) - pL();
  }

  void FrCatenaryLine::dp_pi_dt(Jacobian33 &jacobian) const {
    Direction direction = ti(N(), 1.).normalized() - ti(N(), si(N())).normalized();
    for (unsigned int j = 0; j < N(); j++) {
      direction += ti(j, si(j + 1)).normalized() - ti(j, si(j)).normalized();
    }
    jacobian -= m_pi * direction.transpose();
  }

  Direction FrCatenaryLine::Lambda_tau(const unsigned int &i, const double &s) const {
    return (ti(i, s).normalized() - m_pi) / (rho(i, s));
  }

  void FrCatenaryLine::dp_perp_dt(Jacobian33 &jacobian) const {// FIXME: verifier le micma avec les matrices U !!!!
    // FIXME: ne pas faire appel a N() partout mais en faire un param local !! Pareil ailleurs...
    Jacobian33 matrix = (m_t0 - Fi(N())) * (Lambda_tau(N(), 1.) - Lambda_tau(N(), si(N()))).transpose();
    double scalar = std::log(rho(N(), 1.) / rho(N(), si(N())));
    for (unsigned int j = 0; j < N(); j++) {
      matrix += (m_t0 - Fi(j)) * (Lambda_tau(j, si(j + 1)) - Lambda_tau(j, si(j))).transpose();
      scalar += std::log(rho(j, si(j + 1)) / rho(j, si(j)));
    }
    jacobian += c_U * matrix + c_U * scalar;
  }

  void FrCatenaryLine::dpc_dt(Jacobian33 &jacobian) const {
    dp_pi_dt(jacobian);
    if (!IsSingular()) {
      dp_perp_dt(jacobian);
    }
  }

  void FrCatenaryLine::dpe_dt(Jacobian33 &jacobian) const {
    // FIXME: il semblerait qu'il manque un L !!!!! --> ben non en fait vu la diff dans la convergence... voir pourquoi j'ai pense ca...
    jacobian += (m_q * m_unstretchedLength / m_properties->GetEA()) *
                Eigen::Matrix3d::Identity(); // TODO: utiliser Eigen pour faire une matrice diag a partir du scalaire...
  }

  FrCatenaryLine::Jacobian33 FrCatenaryLine::GetJacobian() const {
    Jacobian33 jacobian;
    jacobian.SetNull();
    dpc_dt(jacobian);
    if (m_elastic) {
      dpe_dt(jacobian);
    }
    return jacobian;
  }

  void FrCatenaryLine::Compute(double time) { // TODO: voir si on passe pas dans la classe de base...
    solve(); // FIXME: c'est la seule chose à faire ??? Pas de rebuild de cache ?
    if (c_iter == m_maxiter) {
      FirstGuess();
      solve();
    }
  }

  void FrCatenaryLine::DefineLogMessages() {

    auto msg = NewMessage("State", "State messages");

    msg->AddField<double>("time", "s", "Current time of the simulation",
                          [this]() { return GetSystem()->GetTime(); });

    msg->AddField<double>("StrainedLength", "m", "Strained length of the catenary line",
                          [this]() { return GetStrainedLength(); });

    msg->AddField<int>("iter", "", "Newton-Raphson solver iterations",
                          [this]() { return GetIter(); });

    msg->AddField<double>("pos_error", "", "Error on the position",
                          [this]() { return GetErr(); });

    msg->AddField<double>("criterion", "", "Convergence criteria on the tension",
                          [this]() { return GetCriterion(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("StartingNodeTension", "N", fmt::format("Starting node tension in world reference frame in {}", GetLogFC()),
         [this]() { return GetStartingNodeTension(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("EndingNodeTension", "N", fmt::format("Ending node tension in world reference frame in {}", GetLogFC()),
         [this]() { return GetEndingNodeTension(GetLogFC()); });
  }

  void FrCatenaryLine::BuildCache() {
    c_U.SetIdentity();
    c_U -= m_pi * m_pi.transpose();

    c_qL = m_q * m_unstretchedLength;

    c_Fi.clear();
    c_sum_fs.clear();
    c_Fi.emplace_back(m_point_forces.front().force()); // FIXME: c'est de toute maniere 0 non ?
    c_sum_fs.emplace_back(m_point_forces.front().force()); // FIXME: c'est de toute maniere 0 non ?

    for (unsigned int i = 0; i < N(); i++) { // TODO: voir la borne
      auto point_force = m_point_forces[i + 1];
      c_Fi.emplace_back(c_Fi[i] + point_force.force());
      c_sum_fs.emplace_back(c_sum_fs[i] + point_force.force() * point_force.s());
    }
  }

  std::shared_ptr<FrCatenaryLine>
  make_catenary_line(const std::string &name,
                     const std::shared_ptr<FrNode> &startingNode,
                     const std::shared_ptr<FrNode> &endingNode,
                     const std::shared_ptr<FrCableProperties> &properties,
                     bool elastic,
                     double unstretchedLength,
                     FLUID_TYPE fluid_type) {

    auto line = std::make_shared<FrCatenaryLine>(name,
                                                 startingNode,
                                                 endingNode,
                                                 properties,
                                                 elastic,
                                                 unstretchedLength,
                                                 fluid_type);
    startingNode->GetBody()->GetSystem()->Add(line);
    return line;

  }

  namespace internal {

    internal::PointForce::PointForce(FrCatenaryLine *line, const double &s, const Force &force) :
        m_line(line), m_s(s), m_force(force) {}

    const double &PointForce::s() const { return m_s; }

    Force PointForce::force() const { return m_force / m_line->c_qL; }


  }  // end namespace frydom::internal

}  // end namespace frydom
