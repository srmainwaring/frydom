//
// Created by frongere on 18/04/2020.
//

#ifndef FRYDOM_FRBSPLINE_H
#define FRYDOM_FRBSPLINE_H

#include <Eigen/Dense>
#include <chrono/geometry/ChLineBspline.h>

#include "frydom/core/math/FrVector.h"

namespace frydom {

  namespace bspline {

    // TODO: retirer le template portant sur le degre !!!

    // Forward declaration
    template<unsigned int _dim>
    class FrBSpline;

    // Defining some pratical typedefs
    using KnotVector = std::vector<double>;

    template<unsigned int _dim = 3>
    using Point = Eigen::Matrix<double, _dim, 1>;

    template<unsigned int _dim = 3>
    using Direction = Eigen::Matrix<double, _dim, 1>;


    namespace internal {
      // In this namespace, internal calculations such as basis functions evaluations or interpolation algorithms are
      // implemented

      enum KNOT_ARRANGEMENT_ALGO {
        EQUALLY_SPACED,
        CHORD_LENGTH,
        CENTRIPETAL
      };

      class FrBSplineTools {


       public:

        static unsigned int FindSpan(const double &u,
                                     const KnotVector &knots,
                                     const int degree) {
          unsigned int n = knots.size() - 2 - degree;

          // assuming (p+1)-multiple end knots
          if (u >= knots[n + 1])return n;
          if (u <= knots[degree]) return degree;

          unsigned int lo = degree;
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

        static std::vector<double> BasisFunctionsEval(const double &u,
                                                      const unsigned int ispan,
                                                      const KnotVector &knots,
                                                      const int degree) {
          // Returns the minimal basis function vector with size p+1

          std::vector<double> basis_functions(degree + 1);

          basis_functions[0] = 1.0;

          std::vector<double> left(degree + 1);
          std::vector<double> right(degree + 1);

          for (int j = 1; j <= degree; ++j) {
            left[j] = u - knots[ispan + 1 - j];
            right[j] = knots[ispan + j] - u;
            double saved = 0.0;
            for (int r = 0; r < j; ++r) {
              double temp = basis_functions[r] / (right[r + 1] + left[j - r]);
              basis_functions[r] = saved + right[r + 1] * temp;
              saved = left[j - r] * temp;
            }
            basis_functions[j] = saved;
          }
          return basis_functions;
        }

        static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
        BasisFunctionsEvalDerivatives(const double &u,
                                      const unsigned int ispan,
                                      const unsigned int deriv_order,
                                      const KnotVector &knots,
                                      const int degree) {

          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DN(deriv_order + 1, degree + 1);

          std::vector<double> left;
          std::vector<double> right;
          left.reserve(degree + 1);
          right.reserve(degree + 1);

          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ndu(degree + 1, degree + 1);
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a(degree + 1, degree + 1);

          ndu(0, 0) = 1.;
          for (int j = 1; j <= degree; j++) {
            left[j] = u - knots[ispan + 1 - j];
            right[j] = knots[ispan + j] - u;
            double saved = 0.0;
            for (int r = 0; r < j; r++) {
              ndu(j, r) = right[r + 1] + left[j - r];
              double temp = ndu(r, j - 1) / ndu(j, r);

              ndu(r, j) = saved + right[r + 1] * temp;
              saved = left[j - r] * temp;
            }
            ndu(j, j) = saved;
          }
          for (int j = 0; j <= degree; j++)
            DN(0, j) = ndu(j, degree);

          if (deriv_order == 0)
            return DN;

          for (int r = 0; r <= degree; r++) {
            int s1 = 0, s2 = 1;
            a(0, 0) = 1.0;

            for (int k = 1; k <= deriv_order; k++) {
              double d = 0.;
              int rk = r - k, pk = (int) degree - k;
              if (r >= k) {
                a(s2, 0) = a(s1, 0) / ndu(pk + 1, rk);
                d = a(s2, 0) * ndu(rk, pk);
              }
              int j1 = rk >= -1 ? 1 : -rk;
              int j2 = (r - 1 <= pk) ? k - 1 : (int) degree - r;
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

          unsigned int r = degree;
          for (unsigned int k = 1; k <= deriv_order; k++) {
            for (unsigned int j = 0; j <= degree; j++)
              DN(k, j) *= r;
            r *= (degree - k);
          }

          return DN;
        }

        template<unsigned int _dim>
        static FrBSpline<_dim>
        BSplineInterpFromPoints(const std::vector<Point<_dim>> &points,
                                std::vector<double> &uk,
                                const int degree,
                                KNOT_ARRANGEMENT_ALGO knot_arrangement_type = CHORD_LENGTH) {

          unsigned int n = points.size(); // Number of points to interpolate
          unsigned int m = n + degree + 1; // Number of knots given the desired spline degree

          // Computing the uk values corresponding to each point by using the specified (chord length by default)

          uk.clear();
          uk.assign(n, 0.);

          if (knot_arrangement_type == CHORD_LENGTH) {

            double d = 0.;
            for (unsigned int k = 1; k < n; k++) {
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
          KnotVector knots(m, 0.);

          for (unsigned int p = 0; p <= degree; p++) {
            knots[p] = 0.;
            knots[m - p - 1] = 1.;
          }
          for (unsigned int j = 1; j < n - degree; j++) {
            for (unsigned int i = j; i < j + degree; i++) {
              knots[j + degree] += uk[i];
            }
            knots[j + degree] /= degree;
          }

          // TODO: Use sparse matrix features to exploit band structure inherent to BSpline basis functions
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(n, n);
          A.setZero();

          // Filling basis functions matrix A
          for (unsigned int i = 0; i < n; i++) {
            unsigned int ispan = FindSpan(uk[i], knots, degree);
            auto basis_functions = BasisFunctionsEval(uk[i], ispan, knots, degree);

            for (unsigned int j = 0; j <= degree; j++) {
              A(i, ispan - degree + j) = basis_functions[j];
            }
          }

          // Preparing the linear solver
          Eigen::PartialPivLU<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> solver;
          solver.compute(A);

          std::vector<Point<_dim>> ctrl_points(n, Point<_dim>());

          // Solving the fitting problem for each dimension
          for (unsigned int dim = 0; dim < _dim; dim++) {
            // Building rhs
            Eigen::Matrix<double, Eigen::Dynamic, 1> rhs(n);
            for (unsigned int j = 0; j < n; j++) {
              rhs(j) = points[j][dim];
            }

            Eigen::Matrix<double, Eigen::Dynamic, 1> sol = solver.solve(rhs);

            // Scattering control points coordinates for current dimension dim
            for (unsigned int j = 0; j < n; j++) {
              ctrl_points[j][dim] = sol[j];
            }

          }

          // Building the BSpline
          return FrBSpline<_dim>(knots, ctrl_points, degree);

        }

        template<unsigned int _dim>
        static std::shared_ptr<FrBSpline<_dim>>
        BSplineInterpFromPoints(const std::vector<Point<_dim>> &points,
                                const int degree,
                                KNOT_ARRANGEMENT_ALGO knot_arrangement_type = CHORD_LENGTH) {
          std::vector<double> uk;
          return BSplineInterpFromPoints<_dim>(points, uk, degree, knot_arrangement_type);
        }

        template<unsigned int _dim>
        static std::shared_ptr<FrBSpline<_dim>>
        BSplineInterpFromPoints(const std::vector<Point<_dim>> &points,
                                const Direction<_dim> &dir_p0,
                                const Direction<_dim> &dir_p1,
                                std::vector<double> &uk,
                                const int degree,
                                KNOT_ARRANGEMENT_ALGO knot_arrangement_type = CHORD_LENGTH) {

          unsigned int n = points.size(); // Number of points to interpolate
          unsigned int m = n + degree + 3; // Number of nodes given the desired spline degree

          // Computing the uk values corresponding to each point by using the specified (chord length by default)

          uk.clear();
          uk.assign(n, 0.);

          if (knot_arrangement_type == CHORD_LENGTH) {

            double d = 0.;
            for (unsigned int k = 1; k < n; k++) {
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
          KnotVector knots(m, 0.);

          for (unsigned int p = 0; p <= degree; p++) {
            knots[p] = 0.;
            knots[m - p - 1] = 1.;
          }
          for (unsigned int j = 0; j < n - degree + 1; j++) {
            for (unsigned int i = j; i < j + degree; i++) {
              knots[j + degree + 1] += uk[i];
            }
            knots[j + degree + 1] /= degree;
          }

          // TODO: Use sparse matrix features to exploit band structure inherent to BSpline basis functions
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(n + 2, n + 2);
          A.setZero();

          // Filling basis functions matrix A
          for (unsigned int i = 0; i < n; i++) {
            unsigned int ispan = FindSpan(uk[i], knots, degree);
            auto basis_functions = BasisFunctionsEval(uk[i], ispan, knots, degree);

            for (unsigned int j = 0; j <= degree; j++) {
              A(i, ispan - degree + j) = basis_functions[j];
            }
          }

          { // temp scope...

            // Filling last two lines for derivatives equations
            // TODO: on ajoute les equations pour les derivees aux frontieres en fin. Ce n'est clairement pas optimal et
            // il faudrait les ajouter respectivement en ligne 2 et en ligne n de la matrice de taille (n+2 x n+2)

            // Derivative at point 0
            double coeff = degree / knots[degree + 1];
            A(n, 0) = -coeff;
            A(n, 1) = coeff;

            // Derivative at point n
            coeff = degree / (1. - knots[m - degree + 1]);
            A(n + 1, n) = -coeff;
            A(n + 1, n + 1) = coeff;

          }  // end temp scope

          // Preparing the linear solver
          Eigen::PartialPivLU<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> solver;
          solver.compute(A);

          std::vector<Point<_dim>> ctrl_points(n, Point<_dim>());

          // Solving the fitting problem for each dimension
          for (unsigned int dim = 0; dim < _dim; dim++) {
            // Building rhs
            Eigen::Matrix<double, Eigen::Dynamic, 1> rhs(n);
            for (unsigned int j = 0; j < n; j++) {
              rhs(j) = points[j][dim];
            }
            rhs(n) = dir_p0[dim];
            rhs(n + 1) = dir_p1[dim];

            Eigen::Matrix<double, Eigen::Dynamic, 1> sol = solver.solve(rhs);

            // Scattering control points coordinates for current dimension dim
            for (unsigned int j = 0; j < n; j++) {
              ctrl_points[j][dim] = sol[j];
            }

          }

          // Building the BSpline
          return std::make_shared<FrBSpline<_dim>>(knots, ctrl_points, degree);

        }

      };

      template<unsigned int _dim = 3>
      chrono::geometry::ChLineBspline FrBSpline2ChBspline(const FrBSpline<_dim> fr_bspline) {
        auto ctrl_points = fr_bspline.GetCtrlPoints();
        unsigned int nb_ctrl_points = ctrl_points.size();
        std::vector<chrono::ChVector<double>> ch_points;
        ch_points.reserve(nb_ctrl_points);
        for (int i = 0; i < nb_ctrl_points; i++) {
          ch_points.push_back(frydom::internal::Vector3dToChVector(ctrl_points[i]));
        }

        auto knots = fr_bspline.GetKnotVector();
        chrono::ChVectorDynamic<double> ch_knots;
        ch_knots.resize(knots.size());
        for (int i = 0; i < knots.size(); i++) {
          ch_knots(i) = knots[i];
        }

        return chrono::geometry::ChLineBspline(3, ch_points, &ch_knots);
      }

    }  // end namespace frydom::bspline::internal

    template<unsigned int _dim = 3>
    class FrBSpline {

     public:
      FrBSpline(const KnotVector knots,
                const std::vector<Point<_dim>> &ctrl_points,
                const int degree) :
          m_knots(std::move(knots)),
          m_ctrl_points(ctrl_points),
          m_degree(degree) {

        // TODO: mettre condition sur la relation entre le nombre de noeuds et l'ordre...

        assert(m_knots.size() == m_ctrl_points.size() + m_degree + 1); // TODO: verifier theorie --> OK
      }

      unsigned int GetNbCtrlPoints() const {
        return m_ctrl_points.size();
      }

      unsigned int GetNbKnots() const {
        return m_knots.size();
      }

      unsigned int degree() const {
        return m_degree;
      }

      unsigned int order() const {
        return m_degree + 1;
      }

      void Eval(const double &u, Point<_dim> &point) const {

        unsigned int ispan = internal::FrBSplineTools::FindSpan(u, m_knots, m_degree);
        auto basis_functions = internal::FrBSplineTools::BasisFunctionsEval(u, ispan, m_knots, m_degree);

        Eval_(basis_functions, ispan, point);
      }

      Point<_dim> Eval(const double &u) const {
        Point<_dim> point;
        Eval(u, point);
        return point;
      }

      void EvalDeriv(const double &u, Direction<_dim> &direction) const {
        unsigned int ispan = internal::FrBSplineTools::FindSpan(u, m_knots, m_degree);
        auto basis_functions_deriv =
            internal::FrBSplineTools::BasisFunctionsEvalDerivatives(u, ispan, 1, m_knots, m_degree);

        EvalDeriv_(basis_functions_deriv, ispan, direction);
      }

      Direction<_dim> EvalDeriv(const double &u) const {
        Direction<_dim> derivative;
        EvalDeriv(u, derivative);
        return derivative;
      }

      std::vector<Point<_dim>> Eval(const std::vector<double> &uvec) const {
        assert(uvec.size() > 0);

        std::vector<Point<_dim>> result(uvec.size(), Point<_dim>());

        unsigned int ispan_last = internal::FrBSplineTools::FindSpan(uvec.front(), m_knots, m_degree);
        auto basis_functions =
            internal::FrBSplineTools::BasisFunctionsEval(uvec.front(), ispan_last, m_knots, m_degree);

        unsigned int k = 0;
        for (const double &u : uvec) {
          unsigned int ispan = internal::FrBSplineTools::FindSpan(u, m_knots, m_degree);
          if (ispan != ispan_last) {
            basis_functions = internal::FrBSplineTools::BasisFunctionsEval(uvec.front(), ispan, m_knots, m_degree);
            ispan_last = ispan;
          }
          Eval_(basis_functions, ispan, result[k]);
          k++;
        }
        return result;
      }


      KnotVector GetKnotVector() const {
        return m_knots;
      }

      double GetKnot(unsigned int i) const {
        return m_knots[i];
      }

      Point<_dim> GetCtrlPoint(unsigned int i) const {
        return m_ctrl_points[i];
      }

      std::vector<Point<_dim>> GetCtrlPoints() {
        return m_ctrl_points;
      }

      void WriteCSV(const std::vector<double> &uvec) {

      }

     private:
      inline void
      Eval_(const std::vector<double> &basis_functions, const unsigned int ispan, Point<_dim> &point) const {
        point.setZero();
        for (unsigned int i = 0; i <= m_degree; i++) {
          point += basis_functions[i] * m_ctrl_points[ispan - m_degree + i];
        }
      }

      inline void EvalDeriv_(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> basis_functions_deriv,
                             const unsigned int ispan,
                             Direction<_dim> &direction) const {
        direction.setZero();
        for (unsigned int i = 0; i <= m_degree; i++) {
          direction += basis_functions_deriv(1, i) * m_ctrl_points[ispan - m_degree + i];
        }
      }

     private:

      std::vector<Point<_dim>> m_ctrl_points;
      KnotVector m_knots;
      int m_degree;

    };

  }  // end namespace frydom::bspline

}  // end namespace frydom



#endif //FRYDOM_FRBSPLINE_H
