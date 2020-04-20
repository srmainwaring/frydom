//
// Created by frongere on 20/04/2020.
//

#include "frydom/frydom.h"

using namespace frydom;

void test_find_span() {
  int p = 2; // p is the spline degree. order 3...

  std::vector<double> knots;
  knots.push_back(0.);
  knots.push_back(0.);
  knots.push_back(0.);
  knots.push_back(1.);
  knots.push_back(2.);
  knots.push_back(3.);
  knots.push_back(4.);
  knots.push_back(4.);
  knots.push_back(5.);
  knots.push_back(5.);
  knots.push_back(5.);

  int span;

  assert(FrBSplineTools::FindSpan(0., knots, p) == 2);
  assert(FrBSplineTools::FindSpan(0.5, knots, p) == 2);
  assert(FrBSplineTools::FindSpan(1., knots, p) == 3);
  assert(FrBSplineTools::FindSpan(1.5, knots, p) == 3);
  assert(FrBSplineTools::FindSpan(3., knots, p) == 5);
  assert(FrBSplineTools::FindSpan(4., knots, p) == 7);
  assert(FrBSplineTools::FindSpan(4.2, knots, p) == 7);
  assert(FrBSplineTools::FindSpan(5., knots, p) == 7);
  assert(FrBSplineTools::FindSpan(6., knots, p) == 7);

}


void test_basis_function() {
  // Piegl p 68 expl 2.3

  double p = 2;

  std::vector<double> knots;
  knots.push_back(0.);
  knots.push_back(0.);
  knots.push_back(0.);
  knots.push_back(1.);
  knots.push_back(2.);
  knots.push_back(3.);
  knots.push_back(4.);
  knots.push_back(4.);
  knots.push_back(5.);
  knots.push_back(5.);
  knots.push_back(5.);

  double u = 5./2.;
  int span = FrBSplineTools::FindSpan(u, knots, p);

  auto basis = FrBSplineTools::BasisFunctionsEval(5./2., span, knots, p);

  assert(basis[0] == 1./8.);
  assert(basis[1] == 6./8.);
  assert(basis[2] == 1./8.);
//  std::cout << basis << std::endl;

}


void test_interpolation() {
  // From Piegl, p367

  int n = 5;

  Position p0 = {0., 0., 0.};
  Position p1 = {3., 4., 0.};
  Position p2 = {-1., 4., 0.};
  Position p3 = {-4., 0., 0.};
  Position p4 = {-4., -3., 0.};


  std::vector<Position> points;
  points.reserve(n);
  points.push_back(p0);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);


  auto interp = FrBSplineTools::BSplineInterpFromPoints(points, 3);

  // Checking back that the curve is passing among points

  // Computing back uk vector
  std::vector<double> uk(n, 0.);
  double d = 0.;
  for (unsigned int k = 1; k < n; k++) {
    d += (points[k] - points[k - 1]).norm();
  }
  uk[0] = 0.;
  uk[n - 1] = 1.;

  for (unsigned int k = 1; k < n; k++) {
    uk[k] = uk[k - 1] + (points[k] - points[k - 1]).norm() / d;
  }

  for (int i = 0; i<n; i++) {
    std::cout << points[i].transpose() << std::endl;
    std::cout << interp->Eval(uk[i]).transpose() << std::endl << std::endl;
  }



}


int main() {

  test_find_span();

  test_basis_function();

  test_interpolation();


}
