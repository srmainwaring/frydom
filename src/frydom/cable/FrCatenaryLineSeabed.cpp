//
// Created by frongere on 27/02/2020.
//

#include "FrCatenaryLineSeabed.h"

#include "FrCatenaryLine.h"
#include "frydom/logging/FrTypeNames.h"
#include "frydom/logging/FrEventLogger.h"
//
//#include "frydom/core/common/FrNode.h"
//#include "frydom/core/body/FrBody.h"
//
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"

namespace frydom {

  FrCatenaryLineSeabed::FrCatenaryLineSeabed(const std::string &name,
                                             const std::shared_ptr<FrNode> &anchorNode,
                                             const std::shared_ptr<FrNode> &fairleadNode,
                                             const std::shared_ptr<FrCableProperties> &properties,
                                             bool elastic,
                                             double unstretchedLength,
                                             FLUID_TYPE fluid,
                                             double seabed_friction_coeff) :
      m_Cb(seabed_friction_coeff),
      FrCatenaryLineBase(name,
                         TypeToString(this),
                         anchorNode,
                         fairleadNode,
                         properties,
                         elastic,
                         unstretchedLength) {

    // Creating the touchdown point node
    m_touch_down_node = GetSystem()->GetWorldBody()->NewNode(name + "_TouchDownPoint");

    m_catenary_line = std::make_unique<FrCatenaryLine>(
        name + "catenary_part",
        m_touch_down_node,
        fairleadNode,
        properties,
        elastic,
        unstretchedLength, // TODO: voir si on peut pas faire un meilleur guess tout de suite ?
        fluid
    );

    m_catenary_line->UseForShapeInitialization(true);

  }

  void FrCatenaryLineSeabed::AddClumpWeight(double s, const double &mass, bool reversed) {
    // TODO
  }

  void FrCatenaryLineSeabed::AddBuoy(double s, const double &mass, bool reversed) {
    // TODO
  }

  void FrCatenaryLineSeabed::Initialize() {

    m_q = m_properties->GetLinearDensity() *
          GetSystem()->GetEnvironment()->GetGravityAcceleration();
    m_pi = {0., 0., -1.}; // FIXME: en dur pour le moment...

    m_touch_down_node->SetPositionInWorld(m_startingNode->GetPositionInWorld(NWU), NWU);
    m_touch_down_node->Initialize();

    m_catenary_line->Initialize();
    BuildCache();

    GuessLb();
    solve();

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

    FrCable::Initialize();

  }

  Force FrCatenaryLineSeabed::GetTension(const double &s, FRAME_CONVENTION fc) const {
    if (0. <= s && s < m_Lb) {
      // On seabed
      if (HasNonZeroTensionAtAnchor()) {
        return (GetTensionAtTouchDown(fc).norm() + m_Cb * (s - m_Lb)) * GetCatenaryPlaneIntersectionWithSeabed(fc);
      } else {
        return std::max(0., m_Cb * (m_Lb - GetGammaLength())) * GetCatenaryPlaneIntersectionWithSeabed(fc);
      }

    } else {
      // catenary part
      return m_catenary_line->GetTension(s - m_Lb, fc);
    }
  }

  Force FrCatenaryLineSeabed::GetTensionAtTouchDown(FRAME_CONVENTION fc) const {
    return m_catenary_line->GetTension(0., fc);
  }

  Force FrCatenaryLineSeabed::GetTensionAtAnchor(FRAME_CONVENTION fc) const {
    return std::max(0., GetTensionAtTouchDown(fc).norm() - m_Cb * m_Lb) * GetCatenaryPlaneIntersectionWithSeabed(fc);
  }

  bool FrCatenaryLineSeabed::HasNonZeroTensionAtAnchor() const {
    if (m_Cb == 0.) {
      return true;
    } else {
      return m_Lb * m_Cb <= GetTensionAtTouchDown(NWU).norm();
    }
  }

  Position FrCatenaryLineSeabed::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
    if (0. <= s && s < m_Lb) {
      // FIXME: ajoute un <= a la énd condition pour que ce soit le cable pose qui donne la reponse en s=Lb
      // Verifier qu'on raccorde bien en s = Lb + eps

      double gamma = GetGammaLength();
      // On seabed
      if (HasNonZeroTensionAtAnchor()) {
        return m_startingNode->GetPositionInWorld(fc) +
               (s + (0.5 * m_Cb * m_unstretchedLength * m_q / m_properties->GetEA()) * // TODO: verifier que c'est bien m_unstretchedLength
                    (s * s - 2. * s * gamma)) * GetCatenaryPlaneIntersectionWithSeabed(fc);

      } else {
        double s_gamma = s - gamma;
        return m_startingNode->GetPositionInWorld(fc) +
               (s + (0.5 * m_Cb * m_unstretchedLength * m_q / m_properties->GetEA()) * s_gamma * s_gamma) * // TODO: verifier que c'est bien m_unstretchedLength
               GetCatenaryPlaneIntersectionWithSeabed(fc);
      }

//      return GetPositionInWorldOnSeabed(s, fc);

    } else {
      // Catenary part
      return m_catenary_line->GetPositionInWorld(s - m_Lb, fc);
    }
  }

  Position FrCatenaryLineSeabed::GetTouchDownPointPosition(FRAME_CONVENTION fc) const {
    return m_touch_down_node->GetPositionInWorld(fc);
  }

  double FrCatenaryLineSeabed::GetUnstretchedLength() const {
    return m_unstretchedLength;
  }

  double FrCatenaryLineSeabed::GetUnstretchedLengthOnSeabed() const {
    return m_Lb;
  }

  void FrCatenaryLineSeabed::solve() {
// TODO
  }

  void FrCatenaryLineSeabed::Compute(double time) {
    solve(); // FIXME: c'est la seule chose à faire ??? Pas de rebuild de cache ?
  }

  internal::FrPhysicsItemBase *FrCatenaryLineSeabed::GetChronoItem_ptr() const {
    return m_chronoPhysicsItem.get();
  }

  void FrCatenaryLineSeabed::DefineLogMessages() {
// TODO
  }

  void FrCatenaryLineSeabed::SetLb(const double& Lb) {
    m_Lb = Lb;
    m_catenary_line->SetUnstretchedLength(m_unstretchedLength - Lb);
  }

  void FrCatenaryLineSeabed::GuessLb() {

    // We take for Lb as a first approximate the abscissae of the cable which is clipping the seabed
    auto seabed = GetSystem()->GetEnvironment()->GetOcean()->GetSeabed();

    // Necessary for the bisection algorithm not to converge to the anchor position which is by definition on seabed
    double L = m_catenary_line->GetUnstretchedLength();
    double ds = L * 1e-3 / L;
    double sa = ds;

    Position position = m_catenary_line->GetPositionInWorld(sa, NWU);
    while (seabed->IsOnSeabed(position, NWU) ||
           seabed->IsAboveSeabed(position, NWU)) {
      sa += ds;
      if (sa > L) {
        event_logger::error(GetTypeName(), GetName(),
                            "This line {} is declared to have seabed interaction but seems too taut to lie on seabed",
                            GetName());
        exit(EXIT_FAILURE);
      }
      position = m_catenary_line->GetPositionInWorld(sa, NWU);
    }

    // Intersection point searching using a bisection algorithm
    double sb = L;
    while (std::fabs(sb - sa) > 1e-6) {

      double sm = 0.5 * (sa + sb);

      Position Pa = m_catenary_line->GetPositionInWorld(sa, NWU);
      double da = Pa.z() - seabed->GetBathymetry(Pa.x(), Pa.y(), NWU);

      Position Pm = m_catenary_line->GetPositionInWorld(sm, NWU);
      double dm = Pm.z() - seabed->GetBathymetry(Pm.x(), Pm.y(), NWU);

      if (da * dm <= 0.) {
        sb = sm;
      } else {
        sa = sm;
      }
    }

//    m_touch_down_node->SetPositionInWorld(m_catenary_line->GetPositionInWorld(sa, NWU), NWU);


//    auto pos = GetPositionInWorldOnSeabed(sa, NWU); // FIXME : cette modif ne fonctionne pas !!!
    auto pos = GetPositionInWorld(sa, NWU);
    m_touch_down_node->SetPositionInWorld(pos, NWU);
    SetLb(sa);

//    auto pos1 = GetPositionInWorld(sa+0.1, NWU);

//    double aa = m_catenary_line->GetUnstretchedLength() + m_Lb;

    m_catenary_line->solve();

//    auto pos1 = GetPositionInWorld(sa+0.1, NWU);


  }

  void FrCatenaryLineSeabed::BuildCache() {
    // TODO, notamment pour ub...
  }

  FrCatenaryLineSeabed::Jacobian44 FrCatenaryLineSeabed::GetJacobian() const {
    Jacobian44 jacobian;

    FrCatenaryLine::Jacobian33 catenary_jacobian = m_catenary_line->GetJacobian();

//    FrCatenaryLine


  }

  FrCatenaryLineSeabed::Residue4 FrCatenaryLineSeabed::GetResidue() const {
    // TODO
  }

  double FrCatenaryLineSeabed::GetGammaLength() const {
    return std::max(0., m_Lb - GetTensionAtTouchDown(NWU).norm() / m_Cb);
  }

  Position FrCatenaryLineSeabed::GetPositionInWorldOnSeabed(const double& s, FRAME_CONVENTION fc) const {
//    double gamma = GetGammaLength();
//    // On seabed
//    if (HasNonZeroTensionAtAnchor()) {
//      return m_startingNode->GetPositionInWorld(fc) +
//             (s + (0.5 * m_Cb * m_unstretchedLength * m_q / m_properties->GetEA()) * // TODO: verifier que c'est bien m_unstretchedLength
//                  (s * s - 2. * s * gamma)) * GetCatenaryPlaneIntersectionWithSeabed(fc);
//
//    } else {
//      double s_gamma = s - gamma;
//      return m_startingNode->GetPositionInWorld(fc) +
//             (s + (0.5 * m_Cb * m_unstretchedLength * m_q / m_properties->GetEA()) * s_gamma * s_gamma) * // TODO: verifier que c'est bien m_unstretchedLength
//             GetCatenaryPlaneIntersectionWithSeabed(fc);
//    }
  }

  Direction FrCatenaryLineSeabed::GetCatenaryPlaneIntersectionWithSeabed(FRAME_CONVENTION fc) const {
    auto p0 = m_startingNode->GetPositionInWorld(NWU);
    auto pL = m_endingNode->GetPositionInWorld(NWU);
    Direction z_axis{0., 0., 1.};

    Direction ub = (((pL - p0).cross(m_pi)).cross(z_axis)).normalized();
    if (IsNED(fc)) {
      internal::SwapFrameConvention(ub);
    }
    return ub; // TODO: mettre en cache !!
  }



  std::shared_ptr<FrCatenaryLineSeabed>
  make_catenary_line_seabed(const std::string &name,
                            const std::shared_ptr<FrNode> &startingNode,
                            const std::shared_ptr<FrNode> &endingNode,
                            const std::shared_ptr<FrCableProperties> &properties,
                            bool elastic,
                            double unstretchedLength,
                            FLUID_TYPE fluid_type,
                            double seabed_friction_coeff) {
    auto line = std::make_shared<FrCatenaryLineSeabed>(name,
                                                       startingNode,
                                                       endingNode,
                                                       properties,
                                                       elastic,
                                                       unstretchedLength,
                                                       fluid_type,
                                                       seabed_friction_coeff);
    startingNode->GetBody()->GetSystem()->Add(line);
    return line;
  }


}
//
//
//  FrCatenaryLineSeabed::FrCatenaryLineSeabed(const std::string &name,
//                                               const std::shared_ptr<FrNode> &anchorNode,
//                                               const std::shared_ptr<FrNode> &fairleadNode,
//                                               const std::shared_ptr<FrCableProperties> &properties,
//                                               bool elastic,
//                                               double unstretchedLength,
//                                               FLUID_TYPE fluid) :
//      FrCatenaryLine(name,
//                     anchorNode->GetBody()->NewNode(name + "_TDP_node"),
//                     fairleadNode,
//                     properties,
//                     elastic,
//                     unstretchedLength,
//                     fluid),
//      m_anchor_node(anchorNode),
//      m_Lb(0.),
//      c_Cb(1.),
//      m_use_seabed_interaction_solver(false) {
//
//  }
//
//  void FrCatenaryLineSeabed::Initialize() {
//
//    auto seabed = GetSystem()->GetEnvironment()->GetOcean()->GetSeabed();
//
//    // Testing if the seabed is not infinite
//    if (seabed->IsInfiniteDepth()) {
//      event_logger::error(GetTypeName(), GetName(),
//                          "Unable to build a seabed interaction line {} with infinite deabed depth", GetName());
//      exit(EXIT_FAILURE);
//    }
//
//    // We test that the anchor given is well on the seabed
//    if (!seabed->IsOnSeabed(m_anchor_node->GetPositionInWorld(NWU), NWU)) {
//      event_logger::error(GetTypeName(), GetName(), "Anchor node {} is not on seabed.", m_anchor_node->GetName());
//      exit(EXIT_FAILURE);
//    }
//
//
//
//    // Getting the cable seabed friction coeffient from seabed ??? pas de definition par ligne ???
//    c_Cb = 1.; // TODO: aller chercher le coeff dans FrSeabed !!! Constant par la suite donc on le met en cache
//
//    // Initially, we put the TDP at anchor position
//    GetTouchDownPointNode()->SetPositionInWorld(m_anchor_node->GetPositionInWorld(NWU), NWU);
//
//    FrCatenaryLine::Initialize();
//
//
//
//    // Getting the direction of the lying part of the line on seabed (intersection of the line plane and the seabed plane)
//    m_lying_direction = (GetFairleadNode()->GetPositionInWorld(NWU) -
//                         GetAnchorNode()->GetPositionInWorld(NWU)).normalized();
//    m_lying_direction = m_lying_direction.cross(m_u);
//    m_lying_direction = (m_lying_direction.cross(Direction(0, 0, 1))).normalized();
//
//
//    // Now we make a first guess on the TDP position by searching the intersection with the seabed
//    CorrectTouchDownPointAbscissae(GetSeabedIntersection() * 0.5);
//
//    // TDP node update
////    UpdateTouchDownPointPosition();
//    m_startingNode->SetPositionInWorld(m_anchor_node->GetPositionInWorld(NWU)
//                                       + m_Lb * m_lying_direction,
//                                       NWU);
////    Position estimated_TDP_position = m_anchor_node->GetPositionInWorld(NWU) + m_Lb * m_lying_direction;
//
//
////    Position estimated_TDP_position = GetPositionInWorld(m_Lb, NWU);
////
////
////    m_startingNode->SetPositionInWorld(estimated_TDP_position, NWU);
////    m_unstretchedLength -= m_Lb;
//
//    // Updating the catenary (to get also a first rough estimate of the tension at tdp)
//    guess_tension();
//    solve();
//
//    // Now we switch to the solver that deal with seabed interaction
//    m_use_seabed_interaction_solver = true;
//    solve();
//
//  }
//
//  void FrCatenaryLineSeabed::CorrectTouchDownPointAbscissae(const double &correction) {
//    m_Lb += correction;
//    m_unstretchedLength -= correction;
//  }
//
//  double FrCatenaryLineSeabed::GetSeabedIntersection() const {
//    auto seabed = GetSystem()->GetEnvironment()->GetOcean()->GetSeabed();
//
//
//    // Necessary for the bisection algorithm not to converge to the anchor position which is by definition on seabed
//    double L = GetUnstretchedLength();
//    double ds = L * 1e-3 / L;
//    double sa = ds;
//
//    Position position = GetPositionInWorld(sa, NWU);
//    while (seabed->IsOnSeabed(position, NWU) ||
//           seabed->IsAboveSeabed(position, NWU)) {
//      sa += ds;
//      if (sa > L) {
//        event_logger::error(GetTypeName(), GetName(),
//                            "This line {} is declared to have seabed interaction bu seems too taut to lie on seabed",
//                            GetName());
//        exit(EXIT_FAILURE);
//      }
//      position = GetPositionInWorld(sa, NWU);
//    }
//
//    // Intersection point searching using a bisection algorithm
//    double sb = GetUnstretchedLength();
//    while (std::fabs(sb - sa) > 1e-6) {
//
//      double sm = 0.5 * (sa + sb);
//
//      Position Pa = GetPositionInWorld(sa, NWU);
//      double da = Pa.z() - seabed->GetBathymetry(Pa.x(), Pa.y(), NWU);
//
//      Position Pm = GetPositionInWorld(sm, NWU);
//      double dm = Pm.z() - seabed->GetBathymetry(Pm.x(), Pm.y(), NWU);
//
//      if (da * dm <= 0.) {
//        sb = sm;
//      } else {
//        sa = sm;
//      }
//    }
//    return sa;
//  }
//
//  void FrCatenaryLineSeabed::solve() {
//
//    if (!m_use_seabed_interaction_solver) {
//      FrCatenaryLine::solve();
//      return;
//    }
//
//
//    mathutils::VectorN<double> res(4);
//    mathutils::MatrixMN<double> jac(4, 4);
//
//    res = get_residual_seabed();
//    std::cout << res << std::endl << std::endl;
//
//    jac = analytical_jacobian_seabed();
//
//    jac.Inverse();
//    mathutils::VectorN<double> correction(4);
//    correction = jac * (-res);
//
//
//    m_t0 += m_relax * correction.head(3);
//    CorrectTouchDownPointAbscissae(m_relax * correction.at(3));
//    m_startingNode->SetPositionInWorld(GetPositionInWorld(m_Lb, NWU), NWU);
////    FrCatenaryLine::solve();
//
//
//    res = get_residual_seabed();
//    std::cout << res << std::endl << std::endl;
//
//    double err = res.infNorm();
//
//    unsigned int iter = 1;
//
//    m_itermax = 10000; // FIXME: a retirer
//
//    while ((err > m_tolerance) && (iter < m_itermax)) {
//      iter++;
//
//      res = get_residual_seabed();
//      std::cout << res << std::endl << std::endl;
//
//      jac = analytical_jacobian_seabed();
//
//      jac.Inverse();
//      mathutils::VectorN<double> correction_temp(4);
//      correction_temp = jac * (-res);
//
//      while (correction.infNorm() < m_relax * correction_temp.infNorm()) {
//        m_relax *= 0.5;
//        if (m_relax < Lmin) {
//          event_logger::warn(GetTypeName(), GetName(), "DAMPING TOO STRONG. NO CATENARY CONVERGENCE.");
//        }
//      }
//
//      correction = correction_temp;
//      m_t0 += m_relax * correction.head(3);
//      CorrectTouchDownPointAbscissae(m_relax * correction.at(3));
//      m_startingNode->SetPositionInWorld(GetPositionInWorld(m_Lb, NWU), NWU);
////      FrCatenaryLine::solve();
//
//      m_relax = std::min(1., m_relax * 2.);
//
//      res = get_residual_seabed();
//      err = res.infNorm();
//    }  // end while
//
//    if (iter == m_itermax) {
//      event_logger::warn(GetTypeName(), GetName(), "Could not converge in max {} iterations", m_itermax);
//    }
//
//
//    std::cout << "cat line SEABED solver converged in " << iter << " iterations" << std::endl;
//
//
//  }
//
//  Position FrCatenaryLineSeabed::GetPositionOnSeabed(double s, FRAME_CONVENTION fc) const {
//    assert(0. <= s && s <= m_Lb);
//
//    Position position;
//
//    double H = GetHorizontalTensionAtTouchDownPoint();
//
//    double gamma = m_Lb - H /
//                          (c_Cb * m_q); // TODO: voir a mettre en cache ! c'est fixe suivant la configuration !!!
//
//    if (s <= gamma) {
//      // From anchor (s=0) to gamma, there is no tension
//      Position offset = m_lying_direction * s;
//      if (IsNED(fc)) { internal::SwapFrameConvention(offset); }
//      position = m_anchor_node->GetPositionInWorld(fc) + offset;
//
//    } else {
//      double lambda = (gamma > 0.) ? gamma : 0.;
//      Position offset = (s + (0.5 * c_Cb * m_q / m_properties->GetEA()) * (s * s - 2. * s * gamma + gamma * lambda)) *
//                        m_lying_direction;
//
//      position = m_anchor_node->GetPositionInWorld(fc) + offset;
//
//    }
//
//    return position;
//  }
//
//  Position FrCatenaryLineSeabed::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
//    assert(0. <= s);
//
//    if (!m_use_seabed_interaction_solver)
//      return FrCatenaryLine::GetPositionInWorld(s, fc);
//
//    if (s <= m_Lb) {
//      return GetPositionOnSeabed(s, fc);
//    } else {
//      return FrCatenaryLine::GetPositionInWorld(s - m_Lb, fc);
//    }
//
//  }
//
//  Force FrCatenaryLineSeabed::GetTension(const double &s, FRAME_CONVENTION fc) const {
//    assert(0. <= s && s <= GetUnstretchedLength());
//
//    Force tension;
//
//    if (!m_use_seabed_interaction_solver) {
//      return FrCatenaryLine::GetTension(s, fc);
//    }
//
//    if (s <= m_Lb) {
//      double tx = m_t0.GetFx();
//      double ty = m_t0.GetFy();
//      double H = GetHorizontalTensionAtTouchDownPoint();
//      tension = std::max(H + c_Cb * m_q * (s - m_Lb), 0.) * m_lying_direction;
//      if (IsNED(fc))
//        internal::SwapFrameConvention(tension);
//      // TODO: verifier qu'on est bien nul si plus petit que gamma...
//
//    } else {
//      tension = FrCatenaryLine::GetTension(s - m_Lb, fc);
//    }
//
//    return tension;
//  }
//
//  inline double FrCatenaryLineSeabed::GetHorizontalTensionAtTouchDownPoint() const {
//    double tx = m_t0.GetFx();
//    double ty = m_t0.GetFy();
//    return std::sqrt(tx * tx + ty * ty);
//  }
//
//  double FrCatenaryLineSeabed::GetUnstretchedLengthCatenaryPart() const {
//    return m_unstretchedLength;
//  }
//
//  double FrCatenaryLineSeabed::GetUnstretchedLength() const {
//    return m_unstretchedLength + m_Lb;
//  }
//
//  mathutils::VectorN<double> FrCatenaryLineSeabed::get_residual_seabed() const {
//    mathutils::VectorN<double> residual(4);
//    residual.head(3) = get_residual(NWU);
//    residual.at(3) = m_q * (m_unstretchedLength) + m_u.transpose() * (m_t0 - m_q * m_unstretchedLength * m_u);
//    return residual;
//  }
//
//  mathutils::MatrixMN<double> FrCatenaryLineSeabed::analytical_jacobian_seabed() const {
//
//    mathutils::MatrixMN<double> jacobian(4, 4);
//
//    jacobian.topLeftCorner(3, 3) = analytical_jacobian();
//
//    // Position part with respect to Lb
//    auto alpha = m_u.transpose() * GetTension(GetUnstretchedLength(), NWU).normalized();
//
//    jacobian.topRightCorner(3, 1) =
//        (c_Umat * m_t0 / _rho(m_unstretchedLength)) * (alpha - m_u.transpose() * m_u) - alpha * m_u - m_t0 / m_q;
//    // TODO: voir si c'est reellement GetUnstretchedLength() ou bien la longugueur dans l'eau...
//
//
//    jacobian.bottomLeftCorner(1, 3) = m_u.transpose();
//
//    jacobian.at(3, 3) = m_q * (1 - m_u.transpose() * m_u);
//
//    return jacobian;
//
//  }
//
//
//}  // end namespace frydom
