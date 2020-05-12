//
// Created by frongere on 27/02/2020.
//

#include "FrCatenaryLineSeabed.h"
#include "frydom/cable/common/FrCableProperties.h"

#include "FrCatenaryLine.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/logging/FrTypeNames.h"
#include "frydom/logging/FrEventLogger.h"
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
        unstretchedLength,
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

    m_touch_down_node->SetPositionInWorld(m_startingNode->GetPositionInWorld(NWU),
                                          NWU); // Placing the TDP at anchor position
    m_touch_down_node->Initialize();

    m_catenary_line->Initialize();
    BuildCache();

    FirstGuess();
    solve(); // FIXME: n'est a priori pas utile,

    FrCatenaryLineBase::Initialize();

  }

  Force FrCatenaryLineSeabed::GetTension(const double &s, FRAME_CONVENTION fc) const {
    Force tension = t(s / m_unstretchedLength) * c_qL;
    if (IsNED(fc))
      internal::SwapFrameConvention(tension);
    return tension;
  }

  Direction FrCatenaryLineSeabed::GetTangent(const double s, FRAME_CONVENTION fc) const {
    assert(0. <= s && s <= m_unstretchedLength);

    double s_ = s / m_unstretchedLength;

    Direction direction;
    if (0. <= s_ && s_ <= m_Lb) { // TODO: voir si on ne balance pas le cas Lb au catenaire...
      direction = GetCatenaryPlaneIntersectionWithSeabed(fc);
    } else if (m_Lb < s_ && s_ <= 1.) {
      direction = m_catenary_line->GetTangent(s - m_Lb * m_unstretchedLength, fc);
    }
    return direction;
  }

  Force FrCatenaryLineSeabed::GetTensionAtTouchDown(FRAME_CONVENTION fc) const {
    Force tension = t_seabed(m_Lb) * c_qL;
    if (IsNED(fc))
      internal::SwapFrameConvention(tension);
    return tension;
  }

  Force FrCatenaryLineSeabed::GetTensionAtAnchor(FRAME_CONVENTION fc) const {
    Force tension = t_seabed(0.) * c_qL;
    if (IsNED(fc))
      internal::SwapFrameConvention(tension);
    return tension;

  }

  bool FrCatenaryLineSeabed::HasTensionAtAnchor() const {
    if (m_Cb == 0.) {
      return true;
    } else {
      return gamma() <= 0.;
    }
  }

  double FrCatenaryLineSeabed::gamma() const {
//    return m_Lb - t_TDP().norm() / m_Cb; // FIXME: doit-on l'extraire de la ligne catenaire ou bien le calculer avec un t(m_Lb) ??
    return m_Lb - t_TDP().dot(GetCatenaryPlaneIntersectionWithSeabed(NWU)) / m_Cb;
    // FIXME: on est en mode seabed, on ne prend normalement que la composante suivant uB
  }

  Position FrCatenaryLineSeabed::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
    // Note that we add the starting node position as we are performing computations in adimensionalized in p with always
    // (0, 0, 0) point as starting node in adim
    Position position = p(s / m_unstretchedLength) * m_unstretchedLength + m_startingNode->GetPositionInWorld(NWU);
    if (IsNED(fc))
      internal::SwapFrameConvention(position);

    return position;
  }

  Position FrCatenaryLineSeabed::GetTouchDownPointPosition(FRAME_CONVENTION fc) const {
    // TODO: voire si on prefere sortir la position du noeud (peut ne pas etre a jour) ou bien recalculer avec un
    // GetPositionInWorld(m_Lb, fc)...
    return m_touch_down_node->GetPositionInWorld(fc);
  }

  double FrCatenaryLineSeabed::GetUnstretchedLength() const {
    return m_unstretchedLength;
  }

  double FrCatenaryLineSeabed::GetUnstretchedLengthOnSeabed() const {
    return m_Lb;
  }

  void
  FrCatenaryLineSeabed::solve() {

//    // TODO: Essais de debug
//
//    // Utiliser ces lignes pour faire du test unitaire !!!
//
//    // Tension au touchdown
//    Force tb_s = t_seabed(m_Lb) * c_qL; // adim par L
//    Force tb_c = m_catenary_line->m_t0 * m_catenary_line->c_qL; // adim par L-Lb
//    // En z, tb_s est nul mais c'est normal.  ---- > OK !
//
//    // Quand on construit la matrice jacobienne et qu'on importe des infos de la partie catenaire, il faut readimensionnaliser
//    // par l'adimensionnalisation globale !!
//
//    // Position du touchdown
//    Position p_TDP_s = m_startingNode->GetPositionInWorld(NWU) + p(m_Lb) * m_unstretchedLength;
//    Position p_TDP_c = m_catenary_line->m_startingNode->GetPositionInWorld(NWU);
//    Position p_TDP_node = m_touch_down_node->GetPositionInWorld(NWU);
//    // C'est OK !!!
//
//    // Tension au fairlead
//    Force t1_s = t(1.) * c_qL;
//    Force t1_c = m_catenary_line->t(1.) *
//                 m_catenary_line->c_qL; // FIXME: la valeur en x est bonne mais erreur sur la valeur en z -> fixer !!
//    // C'est OK !!
//
//    Force t_400_s = GetTension(400, NWU);
//    Force t_400_c = m_catenary_line->GetTension(400 - m_Lb * m_unstretchedLength, NWU);
//    // C'est OK !!
//
//    // Position au fairlead
//    Position p_fairlead_node = m_endingNode->GetPositionInWorld(NWU);
//    Position p_fairlead_c = m_catenary_line->GetPositionInWorld(m_unstretchedLength - m_Lb * m_unstretchedLength, NWU);
//    Position p_fairlead_s = GetPositionInWorld(m_unstretchedLength, NWU);
//    // Ca semble OK !! c et s sont egaux, pas exactement sur le node mais certainement a la tolerance du solveur catenaire
//
//    // TODO: FIN essais de debug



    // Checking the weight of the cable
    double vertical_tension_at_fairlead = -m_catenary_line->m_pi.dot(GetTension(m_unstretchedLength, NWU));
    double weight = (m_unstretchedLength - m_Lb * m_unstretchedLength) * m_q;

    if (std::fabs(vertical_tension_at_fairlead - weight) / std::min(vertical_tension_at_fairlead, weight) < 1e-4) {
      // Already solved, we just continue
      return;
    }

    // Testing if
    double sa, sb;
    double tbz = m_catenary_line->m_t0.z();
    if (tbz > 0.) {
      sa = 0.;
      sb = m_Lb;
    } else {
      sa = m_Lb;
      sb = 1.;
    }

//    Position pa = m_startingNode->GetPositionInWorld(NWU);
//    Direction ub = GetCatenaryPlaneIntersectionWithSeabed(NWU);

    // TODO: plutot que de prendre a chaque fois 0 et 1 comme bornes dans la dichotomie, il conviendrait de s'appuyer
    // sur la valeur courante de Lb... On pourra prendre les bornes 0 et 1 dans un appel a first guess...
    // Adapter l'initialisation de la dichotomie en consequence !!


    // Essai d'une dichotomie pour trouver le Lb...
//    double sa = 0.;
//    double sb = 1.;
    int iter_Lb = 0;
    while (std::fabs(sb - sa) / std::min(sb, sa) > 1e-4) {

      iter_Lb++;

      if (iter_Lb == m_maxiter) {
        event_logger::warn(GetTypeName(), GetName(),
                           "Failed to find the unstretched cable length lying on seabed in {} max iterations",
                           m_maxiter);
        break;
      }

      double sm = 0.5 * (sa + sb);

      SetLb(sm);
      SolveTDPPosition();
//      Position position_TDP = pa / m_unstretchedLength + sm * ub;
//      Position position_TDP_prec;
//
//      // Finding the correct position of the TDP
//      int iter_tdp = 0;
//      while ((position_TDP - position_TDP_prec).norm() > 1e-4) {
//        position_TDP_prec = position_TDP;
//        iter_tdp++;
//        if (iter_tdp == m_maxiter) {
//          event_logger::warn(GetTypeName(), GetName(),
//                             "Failed to find the Touchdown Point position in {} max iterations", m_maxiter);
//          break;
//        }
//        m_touch_down_node->SetPositionInWorld(position_TDP * m_unstretchedLength, NWU);
//        m_catenary_line->FirstGuess();
//        m_catenary_line->solve();
//        position_TDP = pa / m_unstretchedLength + p_seabed(sm);
//        // TODO: etablir un critere de convergence !!
//
//      }

      tbz = m_catenary_line->m_t0.z();

      if (tbz > 0.) {
        sb = sm;
      } else if (tbz < 0.) {
        sa = sm;
      } else {
        break;
      }
    }



//    unsigned int iter = 0;
//
//    // Defining the linear solver
//    Eigen::FullPivLU<Eigen::Matrix4d> linear_solver;
//
//    Residue4 residue;
//    GetResidue(residue);
//    double err = residue.norm();
//
//    if (err < m_tolerance) {
////      std::cout << "Already at equilibrium" << std::endl;
//      return;
//    }
//
//    while (iter < m_maxiter) {
//
//      iter++;
//
//      Jacobian44 jacobian;
//      GetJacobian(jacobian);
//
//      linear_solver.compute(jacobian);
//      Eigen::Vector4d correction = linear_solver.solve(-residue);
//
////      m_catenary_line->m_t0 += correction.head(3);
////      SetLb(m_Lb + correction(3));
//
//      GetResidue(residue);
//      err = residue.norm();
//
//      if (err < m_tolerance) {
//        break;
//      }
//
//    }
//
//    if (iter == m_maxiter) {
//      std::cout << "NO CONVERGENCE" << std::endl;
//    } else {
//      std::cout << "CONVERGENCE IN " << iter << std::endl;
//    }
  }

  bool FrCatenaryLineSeabed::HasSeabedInteraction() const {
    return true; // Obviously
  }

  void FrCatenaryLineSeabed::GetLowestPoint(Position &position,
                                            double &s,
                                            FRAME_CONVENTION fc,
                                            const double tol,
                                            const unsigned int maxIter) const {
    s = m_Lb * m_unstretchedLength;
    position = m_touch_down_node->GetPositionInWorld(fc);
  }

  void FrCatenaryLineSeabed::Compute(double time) { // TODO: voir si on passe pas dans la classe de base...
    solve(); // FIXME: c'est la seule chose à faire ??? Pas de rebuild de cache ?
  }

//  internal::FrPhysicsItemBase *FrCatenaryLineSeabed::GetChronoItem_ptr() const {
//    return m_chronoPhysicsItem.get();
//  }

  void FrCatenaryLineSeabed::DefineLogMessages() {
// TODO
  }

  void FrCatenaryLineSeabed::SetLb(const double &Lb) {
    assert(0. <= Lb && Lb <= 1.);
    m_Lb = Lb;
    m_catenary_line->SetUnstretchedLength((1. - Lb) * m_unstretchedLength);
  }

  void FrCatenaryLineSeabed::FirstGuess() {
    double Lb_guessed = 0.5;
    SetLb(Lb_guessed);
    SolveTDPPosition();
  }

  int FrCatenaryLineSeabed::SolveTDPPosition() {
    Position pa = m_startingNode->GetPositionInWorld(NWU) / m_unstretchedLength;
    Direction ub = GetCatenaryPlaneIntersectionWithSeabed(NWU);

    Position p_TDP = pa + m_Lb * ub;
    Position p_TDP_prec = pa;

    int iter = 0;
    while ((p_TDP - p_TDP_prec).norm() > 1e-4) {
      p_TDP_prec = p_TDP;
      iter++;
      if (iter == m_maxiter) {
        event_logger::warn(GetTypeName(), GetName(),
                           "Failed to find the Touchdown Point position in {} max iterations", m_maxiter);
        break;
      }
      m_touch_down_node->SetPositionInWorld(p_TDP * m_unstretchedLength, NWU);
      m_catenary_line->FirstGuess();
      m_catenary_line->solve();
      p_TDP = pa + p_seabed(m_Lb);
    }

    return iter;
  }

  void FrCatenaryLineSeabed::BuildCache() {
    c_qL = m_q * m_unstretchedLength;
  }

  void FrCatenaryLineSeabed::GetJacobian(Jacobian44 &jacobian) const {

    mathutils::Matrix33<double> jac_dp_Lb_dt;
    internal::JacobianBuilder::dp_dt(*this, *m_catenary_line, jac_dp_Lb_dt);
    jacobian.topLeftCorner(3, 3) = jac_dp_Lb_dt;

    mathutils::Vector3d<double> jac_dp_dLb;
    internal::JacobianBuilder::dp_dLb(*this, *m_catenary_line, jac_dp_dLb);
    jacobian.topRightCorner(3, 1) = jac_dp_dLb;

    jacobian.bottomLeftCorner(1, 3) = -m_pi.transpose();

    jacobian(3, 3) = 0.;

  }

  void FrCatenaryLineSeabed::GetResidue(Residue4 &residue) const {

    mathutils::Vector3d<double> cat_residue;
    internal::JacobianBuilder::catenary_residue(*this, *m_catenary_line, cat_residue);

    double Lb_residue;
    internal::JacobianBuilder::Lb_residue(*this, Lb_residue);

    residue.head(3) = cat_residue;
    residue(3) = Lb_residue;

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

  Position FrCatenaryLineSeabed::p(const double &s) const {
    assert(0. <= s && s <= 1.);
    Position position;
    if (0. <= s && s <= m_Lb) { // TODO: voir si on ne balance pas le cas Lb au catenaire...
      position = p_seabed(s);
    } else if (m_Lb < s && s <= 1.) {
      position = p_catenary(s);
    } else {
      std::cerr << "s must be between 0. and 1. as p(s) is admimensionnalized" << std::endl;
      exit(EXIT_FAILURE);
    }
    return position;
  }

  Position FrCatenaryLineSeabed::p_seabed(const double &s) const {
    assert(0. <= s && s <= m_Lb);

    double l;
    double gama = gamma();

    if (s <= gama) {
      // No tension in line at this position as there are too much friction on seabed from the Touch Down Point
      l = s;
    } else {
      double alpha = (gama > 0.) ? 1. : 0.;
      l = s + 0.5 * (m_Cb * c_qL / m_properties->GetEA()) * (s * s - 2. * s * gama + alpha * gama * gama);
    }

    return l * GetCatenaryPlaneIntersectionWithSeabed(NWU);
  }

  Position FrCatenaryLineSeabed::p_catenary(const double &s) const {
    assert(m_Lb <= s && s <= 1.);
    // Note that we have to remove the position of the starting node here as it is added by the calling method
    // FrCatenaryLineSeabed::GetPositionInWorld() but already taken into account in the FrCatenaryLine::GetPositionInWorld()
    return (m_catenary_line->GetPositionInWorld((s - m_Lb) * m_unstretchedLength, NWU) -
            m_startingNode->GetPositionInWorld(NWU)) / m_unstretchedLength;
  }

  FrCatenaryLineSeabed::Tension FrCatenaryLineSeabed::t(const double &s) const {
    assert(0. <= s && s <= 1.);
    if (0. <= s && s <= m_Lb) {
      return t_seabed(s);
    } else if (m_Lb < s && s <= 1.) {
      return t_catenary(s);
    }
  }

  FrCatenaryLineSeabed::Tension FrCatenaryLineSeabed::t_seabed(const double &s) const {
    assert(0. <= s && s <= m_Lb);
//    double ts = t_TDP().norm() + m_Cb * (s - m_Lb); // Bug ici... on ne prend pas la norme mais la projection sur ub
    double ts = t_TDP().dot(GetCatenaryPlaneIntersectionWithSeabed(NWU)) + m_Cb * (s - m_Lb);

    return std::max(0., ts) * GetCatenaryPlaneIntersectionWithSeabed(NWU);
  }

  FrCatenaryLineSeabed::Tension FrCatenaryLineSeabed::t_catenary(const double &s) const {
    assert(m_Lb <= s && s <= 1.); // FIXME: on ne fournit pas le bon s...
    return m_catenary_line->GetTension((s - m_Lb) * m_unstretchedLength, NWU) / c_qL;
  }

  FrCatenaryLineSeabed::Tension FrCatenaryLineSeabed::t_TDP() const {
    Tension tb = m_catenary_line->GetTension(0., NWU) / c_qL; // On readimentionnalise
    if (tb.dot(Direction(0, 0, 1)) != 0.) {
//      std::cerr << "Tension at touch down is not horizontal, line is not solved" << std::endl;
      event_logger::error(GetTypeName(), GetName(), "Tension at touch down is not horizontal, line is not solved");
    }
    return tb;
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


  namespace internal {

    void JacobianBuilder::dp_dt(const FrCatenaryLineSeabed &sl,
                                const FrCatenaryLine &cl,
                                mathutils::Matrix33<double> &mat) {

      mat = cl.GetJacobian();  // FIXME: renormaliser fonction de la longueur totale (* L_catenary / L_total un truc du genre...)

      double gama = sl.gamma();
      Force tb = sl.t_seabed(sl.m_Lb); // TODO: mettre en cache

      double a;
      if (gama > 0.) {
        a = sl.c_qL / (sl.m_properties->GetEA() * sl.m_Cb);
      } else {
        a = sl.c_qL * sl.m_Lb / (sl.m_properties->GetEA() * tb.norm());
      }

      mat += a * sl.GetCatenaryPlaneIntersectionWithSeabed(NWU) * tb.transpose();
    }

    void JacobianBuilder::dp_dLb(const FrCatenaryLineSeabed &sl,
                                 const FrCatenaryLine &cl,
                                 mathutils::Vector3d<double> &vec) {
      double gama = sl.gamma();
      Direction ub = sl.GetCatenaryPlaneIntersectionWithSeabed(NWU); // TODO: mettre en cache
      Force tb = sl.t_seabed(sl.m_Lb); // TODO: mettre en cache

      double qL_EA = sl.c_qL / sl.m_properties->GetEA();

      if (gama > 0.) {
        vec = ub;
      } else {
        vec = (1. + qL_EA * (tb.norm() - sl.m_Cb * sl.m_Lb)) * ub;
      }

      Force t1 = cl.t(1.);
      Force t1_n = t1.normalized();

      vec -= (cl.m_pi.dot(t1_n)) * cl.m_pi;

      if (!cl.IsSingular()) {
        vec += cl.c_U * ((cl.m_t0 - cl.Fi(cl.N())) / cl.rho(cl.N(), 1.)) *
               (cl.m_pi.transpose() * t1_n - 1.);
      } else {
        // Ne devrait jamais arriver...
        std::cerr << "A line with seabed interaction should never become singular. Please contact developers"
                  << std::endl;
      }

      if (cl.m_elastic) {
        vec -= qL_EA * t1;
      }
    }

    void JacobianBuilder::catenary_residue(const FrCatenaryLineSeabed &sl,
                                           const FrCatenaryLine &cl,
                                           mathutils::Vector3d<double> &vec) {
      vec = cl.GetResidue() * cl.m_unstretchedLength / sl.m_unstretchedLength; // FIXME: verifier que c'est juste !
    }

    void JacobianBuilder::Lb_residue(const FrCatenaryLineSeabed &sl, double &scalar) {
      scalar = sl.m_Lb - sl.m_pi.transpose() * sl.t(1.) - 1.;
    }

  }  // end namespace frydom::internal

}  // end namespace frydom
