//
// Created by frongere on 29/04/2020.
//

#include "FrFEACableElement.h"


namespace frydom {

  namespace internal {

    void FrFEACableElementBase::SetNodesGenericOrder(std::vector<std::shared_ptr<internal::FrFEANodeBase>> nodes,
                                                     std::vector<double> knots, int myorder) {
      // TODO
    }

    void FrFEACableElementBase::EvaluateSectionSpeed(const double eta, chrono::ChVector<> &point_speed) {

      // compute parameter in knot space from eta-1..+1
      double u1 = knots(order); // extreme of span
      double u2 = knots(knots.GetRows() - order - 1);
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
      double u1 = knots(order); // extreme of span
      double u2 = knots(knots.GetRows() - order - 1);
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


  }  // end namespace frydom::internal

}  // end namespace frydom
