//
// Created by frongere on 29/04/2020.
//

#include <memory>
#include <MathUtils/VectorGeneration.h>
#include "chrono/fea/ChBeamSectionCosserat.h"

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
      assert((H.rows() == 6 * nodes.size()) && (H.cols() == 6 * nodes.size()));
      assert(section);

      // BRUTE FORCE METHOD: USE NUMERICAL DIFFERENTIATION!

      //
      // The K stiffness matrix of this element span:
      //

      chrono::ChState      state_x(this->LoadableGet_ndof_x(), nullptr);
      chrono::ChStateDelta state_w(this->LoadableGet_ndof_w(), nullptr);
      this->LoadableGetStateBlock_x(0, state_x);
      this->LoadableGetStateBlock_w(0, state_w);

      int mrows_w = this->LoadableGet_ndof_w();
      int mrows_x = this->LoadableGet_ndof_x();

      // compute Q at current speed & position, x_0, v_0
      chrono::ChVectorDynamic<> Q0(mrows_w);
      this->ComputeInternalForces_impl(Q0, state_x, state_w, true);     // Q0 = Q(x, v)

      chrono::ChVectorDynamic<> Q1(mrows_w);
      chrono::ChVectorDynamic<> Jcolumn(mrows_w);

      if (Kfactor) {
        chrono::ChMatrixDynamic<> K(mrows_w, mrows_w);

        chrono::ChState       state_x_inc(mrows_x, nullptr);
        chrono::ChStateDelta  state_delta(mrows_w, nullptr);

        // Compute K=-dQ(x,v)/dx by backward differentiation
        state_delta.setZero(mrows_w, nullptr);

        for (int i = 0; i < mrows_w; ++i) {
          state_delta(i) += Delta;
          this->LoadableStateIncrement(0, state_x_inc, state_x, 0, state_delta);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;

          Q1.setZero(mrows_w);
          this->ComputeInternalForces_impl(Q1, state_x_inc, state_w, true);   // Q1 = Q(x+Dx, v)
          state_delta(i) -= Delta;

          Jcolumn = (Q1 - Q0) * (-1.0 / Delta);   // - sign because K=-dQ/dx
          K.col(i) = Jcolumn;
        }

        // finally, store K into H:
        H.block(0, 0, mrows_w, mrows_w) = Kfactor * K;
      }
      else
        H.setZero();


      //
      // The R damping matrix of this element:
      //

      if (Rfactor && this->section->GetDamping()) {
        // Compute R=-dQ(x,v)/dv by backward differentiation
        chrono::ChStateDelta  state_w_inc(mrows_w, nullptr);
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
      // The M mass matrix of this element span:
      //

      if (Mfactor) {
        chrono::ChMatrixDynamic<> Mloc(6 * nodes.size(), 6 * nodes.size());
        Mloc.setZero();
        chrono::ChMatrix33<> Mxw;

        if (ChElementBeamIGA::lumped_mass == true) {
          //
          // "lumped" M mass matrix
          //
          chrono::ChMatrixNM<double, 6, 6> sectional_mass;
          this->section->GetInertia()->ComputeInertiaMatrix(sectional_mass);

          // Get Added mass coefficient from section
          double Ca_n = dynamic_cast<FrFEACableSection *>(section.get())->GetCm();

          // Estimated unit length for a node
          double node_equiv_length = length / (double) nodes.size();

          double node_multiplier = length / (double)nodes.size();
          for (int i = 0; i < nodes.size(); ++i) {
            int stride = i * 6;
            // if there is no mass center offset, the upper right and lower left blocks need not be rotated,
            // hence it can be the simple (constant) expression
            //   Mloc.block<6, 6>(stride, stride) += sectional_mass * (node_multiplier * Mfactor);
            // but the more general case needs the rotations, hence:
            Mloc.block<3, 3>(stride,   stride  ) += sectional_mass.block<3, 3>(0,0) * (node_multiplier * Mfactor);
            Mloc.block<3, 3>(stride+3, stride+3) += sectional_mass.block<3, 3>(3,3) * (node_multiplier * Mfactor);
            Mxw = nodes[i]->GetA() * sectional_mass.block<3, 3>(0,3) * (node_multiplier * Mfactor);
            Mloc.block<3, 3>(stride,   stride+3) += Mxw;
            Mloc.block<3, 3>(stride+3, stride)   += Mxw.transpose();

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

            auto inertia = std::dynamic_pointer_cast<chrono::fea::ChInertiaCosseratSimple>(this->section->GetInertia());
            double beta = fluid_density * Ca_n * inertia->GetArea() * node_equiv_length;

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
        }
        else {
          //
          // "consistent" M mass matrix, via Gauss quadrature
          //
          chrono::ChMatrixNM<double, 6, 6> sectional_mass;
          this->section->GetInertia()->ComputeInertiaMatrix(sectional_mass);

          double u1 = knots(order);
          double u2 = knots(knots.size() - order - 1);

          double c1 = (u2 - u1) / 2;
          double c2 = (u2 + u1) / 2;

          // Do quadrature over the Gauss points - reuse the "b" bend Gauss points

          for (int ig = 0; ig < int_order_b; ++ig) {

            // absyssa in typical -1,+1 range:
            double eta = chrono::ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
            // absyssa in span range:
            double u = (c1 * eta + c2);
            // scaling = gauss weight
            double w = chrono::ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

            // Jacobian Jsu = ds/du
            double Jsu = this->Jacobian_b[ig];
            // Jacobian Jue = du/deta
            double Jue = c1;

            // compute the basis functions N(u) at given u:
            int nspan = order;

            chrono::ChVectorDynamic<> N((int)nodes.size());
            chrono::geometry::ChBasisToolsBspline::BasisEvaluate(this->order, nspan, u, knots, N);

            /*
            // interpolate rotation of section at given u, to compute R.
            ChQuaternion<> q_delta;
            ChVector<> da = VNULL;
            ChVector<> delta_rot_dir;
            double delta_rot_angle;
            for (int i = 0; i < nodes.size(); ++i) {
                ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
                q_delta = nodes[0]->coord.rot.GetConjugate() * q_i;
                q_delta.Q_to_AngAxis(delta_rot_angle, delta_rot_dir); // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
                da += delta_rot_dir * delta_rot_angle * N(i);  // a = N_i*a_i
            }
            ChQuaternion<> qda; qda.Q_from_Rotv(da);
            ChQuaternion<> qR = nodes[0]->coord.rot * qda;

            // compute the 3x3 rotation matrix R equivalent to quaternion above
            ChMatrix33<> R(qR);
            */

            // A simplification, for moderate bending, ignore the difference between beam section R
            // and R_i of nearby nodes, obtaining:

            for (int i = 0; i < nodes.size(); ++i) {
              for (int j = 0; j < nodes.size(); ++j) {
                int istride = i * 6;
                int jstride = j * 6;

                double Ni_Nj = N(i) * N(j);
                double gmassfactor = Mfactor * w * Jsu * Jue * Ni_Nj;

                // if there is no mass center offset, the upper right and lower left blocks need not be rotated,
                // hence it can be the simple (constant) expression
                //   Mloc.block<6, 6>(istride + 0, jstride + 0) += gmassfactor * sectional_mass;
                // but the more general case needs the rotations, hence:
                Mloc.block<3, 3>(istride,   jstride  ) += sectional_mass.block<3, 3>(0,0) * gmassfactor;
                Mloc.block<3, 3>(istride+3, jstride+3) += sectional_mass.block<3, 3>(3,3) * gmassfactor;
                Mxw = nodes[i]->GetA() * sectional_mass.block<3, 3>(0,3) * gmassfactor;
                Mloc.block<3, 3>(istride,   jstride+3) += Mxw;
                Mloc.block<3, 3>(istride+3, jstride)   += Mxw.transpose();
              }
            }

          } // end loop on gauss points

        }

        H.block(0, 0, Mloc.rows(), Mloc.cols()) += Mloc;
      }
    }


    chrono::ChVector<double> FrFEACableElementBase::GetTangent(const double eta) {
      chrono::ChVector<double> position_;
      chrono::ChQuaternion<double> quaternion_;

      EvaluateSectionFrame(eta, position_, quaternion_);
      return quaternion_.GetXaxis();
    }


  }  // end namespace frydom::internal

}  // end namespace frydom
