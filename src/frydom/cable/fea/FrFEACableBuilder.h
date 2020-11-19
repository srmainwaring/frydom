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
      void Build(FrFEACableBase *cable,                ///< mesh to store the resulting elements
                 std::shared_ptr<chrono::fea::ChBeamSectionCosserat> section, ///< section material for beam elements
                 const frydom::bspline::FrBSpline<3> &bspline,                         ///< the B-spline to be used as the centerline
                 const int order
      ) {
        // TODO: voir si on passe le YDir en parametre (comme fait dans chrono) ou si on le calcule en place...

        beam_elems.clear();
        beam_nodes.clear();

        int nb_ctrl_points = bspline.GetNbCtrlPoints();

        double rest_length = cable->GetFEACable()->GetUnstretchedLength() / (nb_ctrl_points - 1);

        // compute nb_span of spans (excluding start and end multiple knots with zero lenght span):
        int nb_span = (int) bspline.GetNbKnots() - order - order - 1;  // = n+p+1 -p-p-1 = n-p

        double span_length = cable->GetFEACable()->GetUnstretchedLength() / nb_span;

        // Create the 'complete' stl vector of control points, with uniform distribution
        std::vector<std::shared_ptr<internal::FrFEANodeBase>> fea_nodes;
        for (int i_node = 0; i_node < nb_ctrl_points; ++i_node) {
          double abscyssa = ((double) i_node / (double) (nb_ctrl_points - 1));

          // position of node
          chrono::ChVector<double> pos = internal::Vector3dToChVector(bspline.GetCtrlPoint(i_node));

          // rotation of node, x aligned to tangent at input spline
          chrono::ChMatrix33<> mrot;
          chrono::ChVector<double> tangent = internal::Vector3dToChVector(bspline.EvalDeriv(abscyssa).normalized());

          // Computing ydir:
          // this is the direction normal to the vertical plane that owns the cable under the gravity field only
          Direction u = (bspline.Eval(1.) - bspline.Eval(0.));
          u.z() = 0.;
          u.Normalize();
          chrono::ChVector<double> ydir = internal::Vector3dToChVector(u.cross(Direction{0., 0., 1.}));

          // Set the rotation matrix
          mrot.Set_A_Xdir(tangent, ydir);

          auto hnode_i = std::make_shared<internal::FrFEANodeBase>(chrono::ChFrame<>(pos, mrot));

          cable->AddNode(hnode_i);
          fea_nodes.push_back(hnode_i);
          beam_nodes.push_back(hnode_i);
        }

        // Create the single elements by picking a subset of the nodes and control points
        for (int i_el = 0; i_el < nb_span; ++i_el) {
          std::vector<double> element_knots;
          for (int i_el_knot = 0; i_el_knot < order + order + 1 + 1; ++i_el_knot) {
            element_knots.push_back(bspline.GetKnot(i_el + i_el_knot));
          }

          std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> element_nodes;
          for (int i_el_node = 0; i_el_node < order + 1; ++i_el_node) {
            element_nodes.push_back(fea_nodes[i_el + i_el_node]);
          }

          auto environment = cable->GetFEACable()->GetSystem()->GetEnvironment();
          auto element_i = std::make_shared<internal::FrFEACableElementBase>(environment);
          element_i->SetNodesGenericOrder(element_nodes, element_knots, order);
          element_i->SetSection(section);
          element_i->SetRestLength(rest_length);
          cable->AddElement(element_i);
          beam_elems.push_back(element_i);
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
