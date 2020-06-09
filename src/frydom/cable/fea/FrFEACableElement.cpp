//
// Created by frongere on 29/04/2020.
//

#include <MathUtils/VectorGeneration.h>
#include <frydom/core/FrOffshoreSystem.h>
#include <frydom/environment/FrEnvironment.h>
#include <frydom/core/math/FrVector.h>

#include "FrFEACableElement.h"

#include "FrFEACableSection.h"


namespace frydom {

  namespace internal {

    FrFEACableElementBase::FrFEACableElementBase(FrEnvironment *environment) :
        chrono::fea::ChElementBeamIGA(),
        m_environment(environment) {}

    void FrFEACableElementBase::EvaluateSectionSpeed(const double eta, chrono::ChVector<> &point_speed) {

      // compute parameter in knot space from eta-1..+1
      double u1 = GetU1(); // extreme of span
      double u2 = GetU2();
      double u = u1 + ((eta + 1) / 2.0) * (u2 - u1);
      int nspan = order;

      chrono::ChVectorDynamic<> N((int) nodes.size());

      chrono::geometry::ChBasisToolsBspline::BasisEvaluate(
          this->order,
          nspan,
          u,
          knots,
          N);           ///< here return  in N

      point_speed = chrono::VNULL;
      for (int i = 0; i < nodes.size(); ++i) {
        point_speed += N(i) * nodes[i]->coord_dt.pos;
      }
    }

    void FrFEACableElementBase::EvaluateSectionAcceleration(const double eta, chrono::ChVector<> &point_acceleration) {

      // compute parameter in knot space from eta-1..+1
      double u1 = GetU1(); // extreme of span
      double u2 = GetU2();
      double u = u1 + ((eta + 1) / 2.0) * (u2 - u1);
      int nspan = order;

      chrono::ChVectorDynamic<> N((int) nodes.size());

      chrono::geometry::ChBasisToolsBspline::BasisEvaluate(
          this->order,
          nspan,
          u,
          knots,
          N);           ///< here return  in N

      point_acceleration = chrono::VNULL;
      for (int i = 0; i < nodes.size(); ++i) {
        point_acceleration += N(i) * nodes[i]->coord_dtdt.pos;
      }
    }

    void FrFEACableElementBase::ComputeKRMmatricesGlobal(chrono::ChMatrix<> &H, double Kfactor, double Rfactor,
                                                         double Mfactor) {
      assert((H.GetRows() == 6 * (int) nodes.size()) && (H.GetColumns() == 6 * (int) nodes.size()));
      assert(section);

      // BRUTE FORCE METHOD: USE NUMERICAL DIFFERENTIATION!

      //
      // The K stiffness matrix of this element span:
      //

      chrono::ChState state_x(this->LoadableGet_ndof_x(), nullptr);
      chrono::ChStateDelta state_w(this->LoadableGet_ndof_w(), nullptr);
      this->LoadableGetStateBlock_x(0, state_x);
      this->LoadableGetStateBlock_w(0, state_w);

      double Delta = 1e-10;

      int mrows_w = this->LoadableGet_ndof_w();
      int mrows_x = this->LoadableGet_ndof_x();

      chrono::ChMatrixDynamic<> K(mrows_w, mrows_w);

      // compute Q at current speed & position, x_0, v_0
      chrono::ChMatrixDynamic<> Q0(mrows_w, 1);
      this->ComputeInternalForces_impl(Q0, state_x, state_w, true);     // Q0 = Q(x, v)

      chrono::ChMatrixDynamic<> Q1(mrows_w, 1);
      chrono::ChVectorDynamic<> Jcolumn(mrows_w);
      chrono::ChState state_x_inc(mrows_x, nullptr);
      chrono::ChStateDelta state_delta(mrows_w, nullptr);

      // Compute K=-dQ(x,v)/dx by backward differentiation
      for (int i = 0; i < mrows_w; ++i) {
        state_delta(i) += Delta;
        this->LoadableStateIncrement(0, state_x_inc, state_x, 0,
                                     state_delta);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;

        Q1.Reset(mrows_w, 1);
        this->ComputeInternalForces_impl(Q1, state_x_inc, state_w, true);   // Q1 = Q(x+Dx, v)
        state_delta(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);   // - sign because K=-dQ/dx
        K.PasteMatrix(Jcolumn, 0, i);
      }

      // finally, store K into H:

      K.MatrScale(Kfactor);

      H.PasteMatrix(K, 0, 0);

      // Compute R=-dQ(x,v)/dv by backward differentiation
      if (this->section->GetDamping()) {
        chrono::ChStateDelta state_w_inc(mrows_w, nullptr);
        state_w_inc = state_w;
        chrono::ChMatrixDynamic<> R(mrows_w, mrows_w);

        for (int i = 0; i < mrows_w; ++i) {
          Q1.Reset(mrows_w, 1);

          state_w_inc(i) += Delta;
          this->ComputeInternalForces_impl(Q1, state_x, state_w_inc, true); // Q1 = Q(x, v+Dv)
          state_w_inc(i) -= Delta;

          Jcolumn = (Q1 - Q0) * (-1.0 / Delta);   // - sign because R=-dQ/dv
          R.PasteMatrix(Jcolumn, 0, i);
        }

        R.MatrScale(Rfactor);

        H.PasteSumMatrix(R, 0, 0);
      }


      //
      // The M mass matrix of this element span: (lumped version)
      //

      chrono::ChMatrixDynamic<> Mloc(6 * (int) nodes.size(), 6 * (int) nodes.size());

      double nmass = mass / (double) nodes.size();
      //Iyy and Izz: (orthogonal to spline) approx as 1/50 lumped mass at half dist:
      double linerx = (1. / 50.) * nmass * pow(length,
                                               2);  // note: 1/50 can be even less (this is 0 in many texts, but 0 means no explicit integrator could be used)
      //Ixx: (tangent to spline) approx as node cuboid
      double lineryz =
          (1. / 12.) * nmass * (pow(section->GetDrawThicknessY(), 2) + pow(section->GetDrawThicknessZ(), 2));


      // Get Added mass coefficient from section
      double Ca_n = dynamic_cast<FrFEACableSection *>(section.get())->GetCm();

      // Estimated unit length for a node
      double dL = length / (double) nodes.size();


      for (int i = 0; i < nodes.size(); ++i) {
        int stride = i * 6;
//        double nodelineryz = lineryz;
        //if (i == 0 || i == (nodes.size() - 1)) {
        // node overlapped in neighbouring element
        //	nodelineryz = lineryz * 0.5;
        //}

        Mloc(stride + 0, stride + 0) += Mfactor * nmass;  // node A x,y,z
        Mloc(stride + 1, stride + 1) += Mfactor * nmass;
        Mloc(stride + 2, stride + 2) += Mfactor * nmass;
        Mloc(stride + 3, stride + 3) += Mfactor * linerx;  // node A Ixx,Iyy,Izz
        Mloc(stride + 4, stride + 4) += Mfactor * lineryz;
        Mloc(stride + 5, stride + 5) += Mfactor * lineryz;

        // Including added mass

        double fluid_density = m_environment->GetFluidDensity(
            internal::ChVectorToVector3d<Position>(nodes[i]->coord.pos),
            NWU,
            true
        );

        auto node = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(GetNodeN(i));
        auto tangent = node->Frame().GetRot().GetXaxis();

        double tx = tangent.x();
        double ty = tangent.y();
        double tz = tangent.z();
        double txty = tx * ty;
        double txtz = tx * tz;
        double tytz = ty * tz;
        double tx2 = tx * tx;
        double ty2 = ty * ty;
        double tz2 = tz * tz;

        double beta = fluid_density * Ca_n * section->GetArea() * dL; // Essayer avec length

//        /// Normal added mass
        Mloc(stride + 0, stride + 0) -= Mfactor * beta * (tx2 - 1.);
        Mloc(stride + 0, stride + 1) -= Mfactor * beta * txty;
        Mloc(stride + 0, stride + 2) -= Mfactor * beta * txtz;
        Mloc(stride + 1, stride + 0) -= Mfactor * beta * txty;
        Mloc(stride + 1, stride + 1) -= Mfactor * beta * (ty2 - 1.);
        Mloc(stride + 1, stride + 2) -= Mfactor * beta * tytz;
        Mloc(stride + 2, stride + 0) -= Mfactor * beta * txtz;
        Mloc(stride + 2, stride + 1) -= Mfactor * beta * tytz;
        Mloc(stride + 2, stride + 2) -= Mfactor * beta * (tz2 - 1.);

        /// TODO: tagential added mass

      }

      H.PasteSumMatrix(Mloc, 0, 0);




//      //
//      // The M mass matrix of this element span: (lumped version)
//      //
//
//      chrono::ChMatrixDynamic<> Mloc(6 * (int) nodes.size(), 6 * (int) nodes.size());
//
//      double nmass = mass / (double) nodes.size();
//
//
//      /*
//       * FIXME: Il est tres etrange que ce soit lineryz qui soit egal a une inertie toute petite !!
//       * Ici on considere un rod et pas reelement un solide...
//       * D'autre part, j'aurais tendance a inverser ici les inerties entre linerx et lineryz, ce que semble confirmer
//       * le placement des inerties dans MLoc...
//       */
//
//
//      //Iyy and Izz: (orthogonal to spline) approx as 1/50 lumped mass at half dist:
//      // note: 1/50 can be even less (this is 0 in many texts, but 0 means no explicit integrator could be used)
////      double lineryz = (1. / 50.) * nmass * pow(length,
//      double linerx = (1. / 50.) * nmass * pow(length, 2);
//      //Ixx: (tangent to spline) approx as node cuboid
//      // FIXME: il n'est pas normal de considerer a priori un cuboid si on utilise une section circulaire
//      // mais surtout, les draw_thickness sont censee seulement alimenter la visu, pas la physique (chrono le dit lui
//      // meme...
//      // On peut certainement aller chercher la notion de section circulaire dans section->IsCircular() ??
//      auto is_circular = section->IsCircular();
//
//
////      double linerx = (1. / 12.) * nmass * (pow(section->GetDrawThicknessY(), 2) +
////                                            pow(section->GetDrawThicknessZ(), 2));
//      double lineryz = (1. / 12.) * nmass * (pow(section->GetDrawThicknessY(), 2) +
//                                             pow(section->GetDrawThicknessZ(), 2));
//
//
//      double dL = length / (double) nodes.size();
//      double Cm = dynamic_cast<FrFEACableSection *>(section.get())->GetCm();
//
//      std::vector<double> uvec = mathutils::linspace(-1., 1., nodes.size());
//
//      for (int i = 0; i < nodes.size(); ++i) {
//        int stride = i * 6;
//
//        /*
//         * TODO: c'est ici qu'il faut introduire la masse ajoutee car alors, cela apparait dans la matrice masse
//         * pour la dynamique mais pas dans le loader de gravite.
//         * Il faut dans ce cas uniquement jouer avec les 3 premieres equations de composantes lineaires (pas rotation)
//         * et s'assurer qu'on parvient a introduire un effet d'inertie de Morison qui soit suivant la normale et la
//         * tangente a l'element. On pourrait essayer de voir ce que fait une integration directe type Gauss mais
//         * cela changerait l'approche Chrono et ce sera a voir plus tard. La premiere question est, la partie
//         * force lineaire suppose-t-elle des accelerations linearires dans le repere absolu ou relatif ??
//         * Pour la partie rotation, d'apres les articles, c'est relatif. Franchement pas certain pour la partie
//         * lineair !! (en tout cas, quand on parle de force, la resultante est en absolu et le moment est en relatif...)
//         * Il y a des chances que ce soit pareil ici, auquel cas il faut faire des projections ici pour passer de
//         * relatif a absolu pour la masse ajoutee.
//         *
//         * CONFIRMATION: en regardant le IntStateGather qui donne w, on a bien la partie lineaire exprimee dans le
//         * repere absolu tandis que la partie angulaire est en relatif !! ---> Il faut passer d'une masse ajoutee
//         * relative au cable a une masse ajoutee absolue. Les termes des 3 premieres equations seront donc differents !!
//         *
//         * Question, comment cela se passe-t-il pour le inerties ??? on peut s'attendre a avoir des inerties ajoutees
//         * Ne peut-on pas les prendre en compte juste par des considerations sur la masse ajoutee lineaire ???
//         * (formule ou autre...)
//         */
//
//        // FIXME: ICI, ON AGIT fonction de ce qui precede !!!!!!!!!!!!!!!!!!!!!!!!!
//
//        // Get the fluid density at node
//        double fluid_density = m_environment->GetFluidDensity(
//            internal::ChVectorToVector3d<Position>(nodes[i]->coord.pos),
//            NWU,
//            false
//        );
//
//        double nadded_mass = fluid_density * section->GetArea() * Cm * dL;
//
//        // Computing the normal acceleration of the node
//        auto node_acceleration = nodes[i]->coord_dtdt.pos;
//        chrono::ChVector<double> tangent = GetTangent(uvec[i]);
//        chrono::ChVector<double> normal_acceleration_direction =
//            (node_acceleration - (tangent.Dot(node_acceleration)) * tangent).GetNormalized();
//
//        chrono::ChVector<double> n_ma = nadded_mass * normal_acceleration_direction;
//
//
//        Mloc(stride + 0, stride + 0) += Mfactor * (nmass + n_ma[0]);  // node A x,y,z
//        Mloc(stride + 1, stride + 1) += Mfactor * (nmass + n_ma[1]);
//        Mloc(stride + 2, stride + 2) += Mfactor * (nmass + n_ma[2]);
//        Mloc(stride + 3, stride + 3) += Mfactor * linerx;  // node A Ixx,Iyy,Izz
//        Mloc(stride + 4, stride + 4) += Mfactor * lineryz;
//        Mloc(stride + 5, stride + 5) += Mfactor * lineryz;
//      }
//
//      H.PasteSumMatrix(Mloc, 0, 0);

    }

    chrono::ChVector<double> FrFEACableElementBase::GetTangent(const double eta) {
      chrono::ChVector<double> position_;
      chrono::ChQuaternion<double> quaternion_;

      EvaluateSectionFrame(eta, position_, quaternion_);
      return quaternion_.GetXaxis();
    }


  }  // end namespace frydom::internal

}  // end namespace frydom
