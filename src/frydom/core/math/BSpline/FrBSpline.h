//
// Created by frongere on 18/04/2020.
//

#ifndef FRYDOM_FRBSPLINE_H
#define FRYDOM_FRBSPLINE_H


#include <chrono/core/ChMatrixDynamic.h> // FIXME: This is missing into chrono in headers of ChBasisToolsBspline...
#include <chrono/geometry/ChBasisToolsBspline.h>

#include <Eigen/Dense>
#include "MathUtils/Matrix.h"
#include "MathUtils/VectorN.h"

#include "frydom/core/math/FrVector.h"


namespace frydom {

  // TODO: templater avec l'ordre et la dimension...

  class FrBSplineTools {
   public:
    static unsigned int FindSpan(const double &u,
                                 const std::vector<double> &knots,
                                 const unsigned int &order) {
      unsigned int n = knots.size() - 2 - order;

      // assuming (p+1)-multiple end knots
      if (u >= knots[n + 1])return n;
      if (u <= knots[order]) return order;

      unsigned int lo = order;
      unsigned int hi = n + 1;
      unsigned int mid = (lo + hi) / 2;
      while (u < knots[mid] || u >= knots[mid + 1]) {

        if (u < knots[mid])
          hi = mid;
        else
          lo = mid;

        mid = (lo + hi) / 2;
      }
      return mid;
    }

    static mathutils::VectorN<double> BasisFunctionEval(const double &u,
                                                        const unsigned int span,
                                                        const std::vector<double> &knots,
                                                        const unsigned int &order) {

      unsigned int n = knots.size() - order - 1; // Also number of control points

      mathutils::VectorN<double> basis_functions(n);

      basis_functions[0] = 1.0;

      int j, r;
      std::vector<double> left;
      std::vector<double> right;
      left.reserve(order + 1);
      right.reserve(order + 1);
      double saved, temp;

//      unsigned int i = FindSpan(u, knots, order);

      for (j = 1; j <= order; ++j) {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        saved = 0.0;
        for (r = 0; r < j; ++r) {
          temp = basis_functions[r] / (right[r + 1] + left[j - r]);
          basis_functions[r] = saved + right[r + 1] * temp;
          saved = left[j - r] * temp;
        }
        basis_functions[j] = saved;
      }
      return basis_functions;
    }

    static mathutils::MatrixMN<double> BasisFunctionEvalDeriv(const double &u,
                                                              const unsigned int span,
                                                              const unsigned int deriv_order,
                                                              const std::vector<double> &knots,
                                                              const unsigned int &order) {

      mathutils::MatrixMN<double> DN(deriv_order + 1, order + 1);

      std::vector<double> left;
      std::vector<double> right;
      left.reserve(order + 1);
      right.reserve(order + 1);

      mathutils::MatrixMN<double> ndu(order + 1, order + 1);
      mathutils::MatrixMN<double> a(order + 1, order + 1);

      ndu(0, 0) = 1.;
      for (int j = 1; j <= order; j++) {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; r++) {
          ndu(j, r) = right[r + 1] + left[j - r];
          double temp = ndu(r, j - 1) / ndu(j, r);

          ndu(r, j) = saved + right[r + 1] * temp;
          saved = left[j - r] * temp;
        }
        ndu(j, j) = saved;
      }
      for (int j = 0; j <= order; j++)
        DN(0, j) = ndu(j, order);

      if (deriv_order == 0)
        return DN;

      for (int r = 0; r <= order; r++) {
        int s1 = 0, s2 = 1;
        a(0, 0) = 1.0;

        for (int k = 1; k <= deriv_order; k++) {
          double d = 0.;
          int rk = r - k, pk = (int) order - k;
          if (r >= k) {
            a(s2, 0) = a(s1, 0) / ndu(pk + 1, rk);
            d = a(s2, 0) * ndu(rk, pk);
          }
          int j1 = rk >= -1 ? 1 : -rk;
          int j2 = (r - 1 <= pk) ? k - 1 : (int) order - r;
          for (int j = j1; j <= j2; j++) {
            a(s2, j) = (a(s1, j) - a(s1, j - 1)) / ndu(pk + 1, rk + j);
            d += a(s2, j) * ndu(rk + j, pk);
          }
          if (r <= pk) {
            a(s2, k) = -a(s1, k - 1) / ndu(pk + 1, r);
            d += a(s2, k) * ndu(r, pk);
          }
          DN(k, r) = d;
          int j = s1;
          s1 = s2;
          s2 = j;
        }
      }

      unsigned int r = order;
      for (unsigned int k = 1; k <= deriv_order; k++) {
        for (unsigned int j = 0; j <= order; j++)
          DN(k, j) *= r;
        r *= (order - k);
      }

    }
  };


  class FrBSpline {

   public:
    FrBSpline(const unsigned int order,
              const std::vector<double> &knots,
              const std::vector<Position> &ctrl_points) :
        m_order(order),
        m_knots(knots),
        m_ctrl_points(ctrl_points) {

      // TODO: mettre condition sur la relation entre le nombre de noeuds et l'ordre...

      assert(m_knots.size() == m_ctrl_points.size() + m_order + 1); // TODO: verifier theorie --> OK
    }

    unsigned int GetNbCtrlPoints() const {
      return m_ctrl_points.size();
    }

    unsigned int GetNbKnots() const {
      return m_knots.size();
    }

   private:
    unsigned int m_order;

    std::vector<Position> m_ctrl_points;
    std::vector<double> m_knots;

  };

  namespace bspline {
    enum KNOT_ARRANGEMENT {
      EQUALLY_SPACED,
      CHORD_LENGTH,
      CENTRIPETAL
    };


  }


  FrBSpline BSplineInterpFromPoints(const std::vector<Position> &points,
                                    const unsigned int order,
                                    bspline::KNOT_ARRANGEMENT knot_arrangement_type = bspline::CHORD_LENGTH) {

    unsigned int n = points.size(); // Number of points to interpolate
    unsigned int m = n + order + 1; // Number of nodes given the desired spline order



    // Computing the uk values corresponding to each point by using the specified (chord length by default)

    std::vector<double> uk(n, 0.);

    if (knot_arrangement_type == bspline::CHORD_LENGTH) {

      double d = 0.;
      for (unsigned int k = 1; k <= n; k++) {
        d += (points[k] - points[k - 1]).norm();
      }
      uk[0] = 0.;
      uk[n - 1] = 1.;

      for (unsigned int k = 1; k < n; k++) {
        uk[k] = uk[k - 1] + (points[k] - points[k - 1]).norm() / d;
      }

    } else {
      std::cerr << "Other knot arrangement methods not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }


    // Computing the node vector by averaging
    std::vector<double> knots(m, 0.);

    for (unsigned int p = 0; p < order; p++) {
      knots[p] = 0.;
      knots[m - p] = 1.;
    }
    for (unsigned int j = 1; j <= n - order; j++) {
      for (unsigned int i = j; i < j + order; i++) {
        knots[j + order] += uk[i];
      }
      knots[j + order] /= order;
    }


    // La suite est bof...

    mathutils::MatrixMN<double> A(n, n);
    A.SetNull();

    // Filling A
    for (unsigned int i = 0; i < n; i++) {
      unsigned int span = FrBSplineTools::FindSpan(uk[i], knots, order);
      A.row(i) = FrBSplineTools::BasisFunctionEval(uk[i], span, knots, order).transpose();
    }

    Eigen::LLT<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> solver;
    solver.compute(A);

    std::vector<Position> ctrl_points(n, Position());

    // Solving the fitting problem for each dimension
    for (unsigned int dim = 0; dim < 3; dim++) {
      // Building rhs
      mathutils::VectorN<double> rhs(n);
      for (unsigned int j = 0; j < n; j++) {
        rhs(j) = points[j][dim];
      }

      mathutils::VectorN<double> sol(n);
      for (unsigned int j = 0; j < n; j++) {
        // Solving the system A * ctrl_points (dim coord) = points
        // TODO: voir a optimiser la resolution en prenant en compte la structure...
        sol = solver.solve(rhs);
      }
      // Setting ctonrol points coordinates
      for (unsigned int j = 0; j < n; j++) {
        ctrl_points[j][dim] = sol[j];
      }

    }

    // Building the BSpline
    return FrBSpline(order, knots, ctrl_points);

  }


}  // end namespace frydom



#endif //FRYDOM_FRBSPLINE_H
