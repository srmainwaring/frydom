//
// Created by frongere on 28/04/2020.
//

#ifndef FRYDOM_FRFEACABLEBUILDER_H
#define FRYDOM_FRFEACABLEBUILDER_H


namespace frydom {

//  /// Class for an helper object that provides easy functions to create
//  /// complex beams of ChElementBeamIGA class, for example subdivides a segment
//  /// in multiple finite elements.
//  class FrFEACableBuilder {
//   protected:
//    std::vector<std::shared_ptr<ChElementBeamIGA> > beam_elems;
//    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > beam_nodes;
//
//   public:
//    /// Helper function.
//    /// Adds beam FEM elements to the mesh to create a segment beam
//    /// from point A to point B, using ChElementBeamIGA type elements.
//    /// Before running, each time resets lists of beam_elems and beam_nodes.
//    void BuildBeam(std::shared_ptr<ChMesh> mesh,              ///< mesh to store the resulting elements
//                   std::shared_ptr<ChBeamSectionCosserat> sect, ///< section property for beam elements
//                   const int N,                               ///< number of elements in the segment
//                   const ChVector<> A,                        ///< starting point
//                   const ChVector<> B,                        ///< ending point
//                   const ChVector<> Ydir,                     ///< the 'up' Y direction of the beam
//                   const int order = 3                        ///< the order of spline (default=3,cubic)
//    );
//
//    /// Helper function.
//    /// Adds beam FEM elements to the mesh to create a spline beam
//    /// using ChElementBeamIGA type elements, given a B-spline line in 3D space.
//    /// Before running, each time resets lists of beam_elems and beam_nodes.
//    void BuildBeam(std::shared_ptr<ChMesh> mesh,                ///< mesh to store the resulting elements
//                   std::shared_ptr<ChBeamSectionCosserat> sect, ///< section material for beam elements
//                   geometry::ChLineBspline &spline,             ///< the B-spline to be used as the centerline
//                   const ChVector<> Ydirn                       ///< the 'up' Y direction of the beam
//    );
//
//    /// Access the list of elements used by the last built beam.
//    /// It can be useful for changing properties afterwards.
//    /// This list is reset all times a 'Build...' function is called.
//    std::vector<std::shared_ptr<ChElementBeamIGA> > &GetLastBeamElements() { return beam_elems; }
//
//    /// Access the list of nodes used by the last built beam.
//    /// It can be useful for adding constraints or changing properties afterwards.
//    /// This list is reset all times a 'Build...' function is called.
//    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > &GetLastBeamNodes() { return beam_nodes; }
//  };

}  // end namespace frydom



#endif //FRYDOM_FRFEACABLEBUILDER_H
