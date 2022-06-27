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

    void FrFEACableElementBase::ComputeKRMmatricesGlobal(chrono::ChMatrixRef H, double Kfactor, double Rfactor,
                                                         double Mfactor) {
      assert((H.rows() == 6 * (int) nodes.size()) && (H.cols() == 6 * (int) nodes.size()));
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
      chrono::ChVectorDynamic<> Q0(mrows_w);
      this->ComputeInternalForces_impl(Q0, state_x, state_w, true);     // Q0 = Q(x, v)

      chrono::ChVectorDynamic<> Q1(mrows_w, 1);
      chrono::ChVectorDynamic<> Jcolumn(mrows_w);
      chrono::ChState state_x_inc(mrows_x, nullptr);
      chrono::ChStateDelta state_delta(mrows_w, nullptr);

      // Compute K=-dQ(x,v)/dx by backward differentiation
      state_delta.setZero(mrows_w, nullptr);

      for (int i = 0; i < mrows_w; ++i) {
        state_delta(i) += Delta;
        this->LoadableStateIncrement(0, state_x_inc, state_x, 0,
                                     state_delta);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;

        Q1.setZero(mrows_w);
        this->ComputeInternalForces_impl(Q1, state_x_inc, state_w, true);   // Q1 = Q(x+Dx, v)
        state_delta(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);   // - sign because K=-dQ/dx
        K.col(i) = Jcolumn;
      }

      // finally, store K into H:
      H.block(0, 0, mrows_w, mrows_w) = Kfactor * K;

      // Compute R=-dQ(x,v)/dv by backward differentiation
      if (this->section->GetDamping()) {
        chrono::ChStateDelta state_w_inc(mrows_w, nullptr);
        state_w_inc = state_w;
        chrono::ChMatrixDynamic<> R(mrows_w, mrows_w);

        for (int i = 0; i < mrows_w; ++i) {
          Q1.setZero(mrows_w);

          state_w_inc(i) += Delta;
          this->ComputeInternalForces_impl(Q1, state_x, state_w_inc, true); // Q1 = Q(x, v+Dv)
          state_w_inc(i) -= Delta;

          Jcolumn = (Q1 - Q0) * (-1.0 / Delta);   // - sign because R=-dQ/dv
          R.col(i) = Jcolumn;
        }

        H.block(0, 0, mrows_w, mrows_w) += Rfactor * R;
      }

      //
      // The M mass matrix of this element span: (lumped version)
      //

      chrono::ChMatrixDynamic<> Mloc(6 * nodes.size(), 6 * nodes.size());
      Mloc.setZero();

      double nmass = mass / (double) nodes.size();

      // FIXME: il y a litige sur qui est linerx et lineryz...
      //Iyy and Izz: (orthogonal to spline) approx as 1/50 lumped mass at half dist:
      double linerx = (1. / 50.) * nmass * pow(length, 2);
      // note: 1/50 can be even less (this is 0 in many texts, but 0 means no explicit integrator could be used)

      //Ixx: (tangent to spline) approx as node cuboid
      double lineryz =
          (1. / 12.) * nmass * (pow(section->GetDrawThicknessY(), 2) + pow(section->GetDrawThicknessZ(), 2));


      // Get Added mass coefficient from section
      double Ca_n = dynamic_cast<FrFEACableSection *>(section.get())->GetCm();

      // Estimated unit length for a node
      double node_equiv_length = length / (double) nodes.size();


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


        /**
         * Including added mass
         */

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

        double beta = fluid_density * Ca_n * section->GetArea() * node_equiv_length;

        /// Normal added mass
        Mloc(stride + 0, stride + 0) -= Mfactor * beta * (tx * tx - 1.);
        Mloc(stride + 0, stride + 1) -= Mfactor * beta * txty;
        Mloc(stride + 0, stride + 2) -= Mfactor * beta * txtz;
        Mloc(stride + 1, stride + 0) -= Mfactor * beta * txty;
        Mloc(stride + 1, stride + 1) -= Mfactor * beta * (ty * ty - 1.);
        Mloc(stride + 1, stride + 2) -= Mfactor * beta * tytz;
        Mloc(stride + 2, stride + 0) -= Mfactor * beta * txtz;
        Mloc(stride + 2, stride + 1) -= Mfactor * beta * tytz;
        Mloc(stride + 2, stride + 2) -= Mfactor * beta * (tz * tz - 1.);

        /// TODO: introduce tangential added mass too...

      }

      H.block(0, 0, Mloc.rows(), Mloc.cols()) += Mloc;

    }

    chrono::ChVector<double> FrFEACableElementBase::GetTangent(const double eta) {
      chrono::ChVector<double> position_;
      chrono::ChQuaternion<double> quaternion_;

      EvaluateSectionFrame(eta, position_, quaternion_);
      return quaternion_.GetXaxis();
    }


  }  // end namespace frydom::internal

}  // end namespace frydom
