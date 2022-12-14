//
// Created by frongere on 20/04/2020.
//

#include "frydom/frydom.h"
#include <MathUtils/Check.h>

using namespace frydom::bspline;
using namespace frydom::bspline::internal;

// TODO: passer en unit testing


void test_find_span() {

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

  double order = 2;
  assert(FrBSplineTools::FindSpan(0., knots, order) == 2);
  assert(FrBSplineTools::FindSpan(0.5, knots, order) == 2);
  assert(FrBSplineTools::FindSpan(1., knots, order) == 3);
  assert(FrBSplineTools::FindSpan(1.5, knots, order) == 3);
  assert(FrBSplineTools::FindSpan(3., knots, order) == 5);
  assert(FrBSplineTools::FindSpan(4., knots, order) == 7);
  assert(FrBSplineTools::FindSpan(4.2, knots, order) == 7);
  assert(FrBSplineTools::FindSpan(5., knots, order) == 7);
  assert(FrBSplineTools::FindSpan(6., knots, order) == 7);

}


void test_basis_function() {
  // Piegl p 68 expl 2.3

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

  double u = 5. / 2.;
  double order = 2;
  auto span = FrBSplineTools::FindSpan(u, knots, order);

  auto basis = FrBSplineTools::BasisFunctionsEval(5. / 2., span, knots, order);

  assert(basis[0] == 1. / 8.);
  assert(basis[1] == 6. / 8.);
  assert(basis[2] == 1. / 8.);
//  std::cout << basis << std::endl;

}


void test_interpolation() {
  // From Piegl, p367

  int n = 5;

  Point<2> p0 = {0., 0.};
  Point<2> p1 = {3., 4.};
  Point<2> p2 = {-1., 4.};
  Point<2> p3 = {-4., 0.};
  Point<2> p4 = {-4., -3.};


  std::vector<Point<2>> points;
  points.reserve(n);
  points.push_back(p0);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);


  std::vector<double> uk;
  double order = 3;
  auto bspline_interp = FrBSplineTools::BSplineInterpFromPoints<2>(points, uk, order);

  // Checking back that the curve is passing among points
  for (int i = 0; i < n; i++) {
    std::cout << points[i].transpose() << std::endl;
    std::cout << bspline_interp.Eval(uk[i]).transpose() << std::endl << std::endl;
    assert(mathutils::IsClose((points[i] - bspline_interp.Eval(uk[i])).norm(), 0.));
  }

}


int main() {

  test_find_span();

  test_basis_function();

  test_interpolation();


}
