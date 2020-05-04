//
// Created by frongere on 28/04/2020.
//

#ifndef FRYDOM_FRFEACABLEBUILDER_H
#define FRYDOM_FRFEACABLEBUILDER_H

#include "frydom/core/math/bspline/FrBSpline.h"
#include "FrFEACable.h"
#include "FrFEANode.h"
#include "FrFEACableElement.h"


namespace frydom {

  namespace internal {

    class FrFEACableElementBase;

    class FrFEANodeBase;

    /// Class for an helper object that provides easy functions to create
    /// complex beams of ChElementBeamIGA class, for example subdivides a segment
    /// in multiple finite elements.
    class FrFEACableBuilder {
     protected:
      std::vector<std::shared_ptr<FrFEACableElementBase>> beam_elems;
      std::vector<std::shared_ptr<FrFEANodeBase>> beam_nodes;

     public:

      /// Helper function.
      /// Adds beam FEM elements to the mesh to create a spline beam
      /// using ChElementBeamIGA type elements, given a B-spline line in 3D space.
      /// Before running, each time resets lists of beam_elems and beam_nodes.
      template<unsigned int _order>
      void Build(FrFEACableBase *cable,                ///< mesh to store the resulting elements
                 std::shared_ptr<chrono::fea::ChBeamSectionCosserat> section, ///< section material for beam elements
                 const frydom::bspline::FrBSpline<_order, 3> &bspline                         ///< the B-spline to be used as the centerline
      ) {


        beam_elems.clear();
        beam_nodes.clear();

        int nb_knots = bspline.GetNbKnots();

        // compute nb_span of spans (excluding start and end multiple knots with zero lenght span):
        int nb_span = (int) bspline.GetNbKnots() - _order - _order - 1;  // = n+p+1 -p-p-1 = n-p

        // Create the 'complete' stl vector of control points, with uniform distribution
        std::vector<std::shared_ptr<internal::FrFEANodeBase>> fea_nodes;
        for (int i_node = 0; i_node < nb_knots; ++i_node) {
          double abscyssa = ((double) i_node / (double) (nb_knots - 1));

          // position of node
          chrono::ChVector<double> pos = internal::Vector3dToChVector(bspline.GetCtrlPoint(i_node));

          // rotation of node, x aligned to tangent at input spline
          chrono::ChMatrix33<> mrot;
          chrono::ChVector<double> tangent = internal::Vector3dToChVector(bspline.EvalDeriv(abscyssa));

          mrot.Set_A_Xdir(tangent, {0., 0., 1.});

          auto hnode_i = std::make_shared<internal::FrFEANodeBase>(chrono::ChFrame<>(pos, mrot));

          cable->AddNode(hnode_i);
          fea_nodes.push_back(hnode_i);
          beam_nodes.push_back(hnode_i);
        }

        // Create the single elements by picking a subset of the nodes and control points
        for (int i_el = 0; i_el < nb_span; ++i_el) {
          std::vector<double> element_knots;
          for (int i_el_knot = 0; i_el_knot < _order + _order + 1 + 1; ++i_el_knot) {
            element_knots.push_back(bspline.GetKnot(i_el + i_el_knot));
          }

          std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> element_nodes;
          for (int i_el_node = 0; i_el_node < _order + 1; ++i_el_node) {
            element_nodes.push_back(fea_nodes[i_el + i_el_node]);
          }

          auto belement_i = std::make_shared<internal::FrFEACableElementBase>();
          belement_i->SetNodesGenericOrder(element_nodes, element_knots, _order);
          belement_i->SetSection(section);
          cable->AddElement(belement_i);
          beam_elems.push_back(belement_i);
        }

      }


      /// Access the list of elements used by the last built beam.
      /// It can be useful for changing properties afterwards.
      /// This list is reset all times a 'Build...' function is called.
      std::vector<std::shared_ptr<FrFEACableElementBase>> &GetLastBeamElements() { return beam_elems; }

      /// Access the list of nodes used by the last built beam.
      /// It can be useful for adding constraints or changing properties afterwards.
      /// This list is reset all times a 'Build...' function is called.
      std::vector<std::shared_ptr<FrFEANodeBase>> &GetLastBeamNodes() { return beam_nodes; }
    };

  }  // end namespace frydom::internal

}  // end namespace frydom



#endif //FRYDOM_FRFEACABLEBUILDER_H
