//
// Created by frongere on 22/04/2020.
//



#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverMINRES.h"
//#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChBeamSectionCosserat.h"
#include "chrono/fea/ChBuilderBeam.h"
//#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/physics/ChLoaderU.h"
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoadContainer.h"


#include "frydom/frydom.h"


using namespace chrono;
//using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;


//    ChElementBeamIGA
//    ChNodeFEAxyzrot
//    ChMesh
//    ChBeamSectionCosserat
//    ChElasticityCosseratSimple

using namespace frydom;


//class MyElasticityCosserat : public chrono::fea::ChElasticityCosseratSimple {
//
//};
//
//
//class MySectionCosserat : public chrono::fea::ChBeamSectionCosserat {
// public:
//  MySectionCosserat(std::shared_ptr <MyElasticityCosserat> elasticity) :
//      chrono::fea::ChBeamSectionCosserat(elasticity) {}
//};
//
//
//
//class MyCable;
//
//
//class MyNodeFEA {
// public:
//  explicit MyNodeFEA(FrFrame initial_frame = FrFrame()) :
//      m_chrono_node(std::make_shared<chrono::fea::ChNodeFEAxyzrot>(internal::FrFrame2ChFrame(initial_frame))) {}
//
// private:
//  std::shared_ptr <chrono::fea::ChNodeFEAxyzrot> m_chrono_node;
//
//  friend class MyCable;
//};
//
//
//class MyCable : public chrono::fea::ChMesh {
//
// public:
//  void AddNode(std::shared_ptr<MyNodeFEA> node) {
//    AddNode(node->m_chrono_node);
//  }
//
//};


class MyElementBeamIGA : public chrono::fea::ChElementBeamIGA {
 public:

//  void SetNodesGenericOrder(std::vector <std::shared_ptr<MyNodeFEA>> nodes,
//                            std::vector<double> knots,
//                            int order) {
//    std::vector <chrono::fea::ChNodeFEAxyzrot> ch_nodes;
//    ch_nodes.reserve(nodes.size());
//    for (const auto &node : nodes) {
//      auto ch_node = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(node);
//      ch_nodes.push_back(ch_node);
//    }
//
//  }

};

class MyLoader : public chrono::ChLoaderUdistributed {
 public:

  MyLoader(std::shared_ptr <chrono::ChLoadableU> loadable) : chrono::ChLoaderUdistributed(loadable) {}

  int GetIntegrationPointsU() override {
    return 1;
  }

  void ComputeF(const double U,
                chrono::ChVectorDynamic<> &F,
                chrono::ChVectorDynamic<> *state_x,
                chrono::ChVectorDynamic<> *state_w) override {
    auto element = std::dynamic_pointer_cast<MyElementBeamIGA>(loadable);

    chrono::ChVector<double> position_;
    chrono::ChQuaternion<double> quaternion_;

    element->EvaluateSectionFrame(U, position_, quaternion_);

    // TODO: Continuer...
    F.PasteVector(chrono::VNULL, 0, 0);
    F.PasteVector(chrono::VNULL, 3, 0);


  }

};

class MyLoad : public chrono::ChLoad<MyLoader> {
 public:

  MyLoad(std::shared_ptr <chrono::ChLoadableU> loadable) : chrono::ChLoad<MyLoader>(loadable) {}

};


/// Class for an helper object that provides easy functions to create
/// complex beams of ChElementBeamIGA class, for example subdivides a segment
/// in multiple finite elements.
class MyFEACableBuilder {
 protected:
  std::vector <std::shared_ptr<MyElementBeamIGA>> beam_elems;
  std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> beam_nodes;

 public:
  /// Helper function.
  /// Adds beam FEM elements to the mesh to create a segment beam
  /// from point starting_node to point ending_node, using ChElementBeamIGA type elements.
  /// Before running, each time resets lists of beam_elems and beam_nodes.
  void BuildBeam(std::shared_ptr <chrono::fea::ChMesh> mesh,              ///< mesh to store the resulting elements
                 std::shared_ptr <chrono::fea::ChBeamSectionCosserat> sect,    ///< section property for beam elements
                 const int N,                               ///< number of elements in the segment
                 const FrNode starting_node,                        ///< starting point
                 const FrNode ending_node,                        ///< ending point
//                 const ChVector<> Ydir,                     ///< the 'up' Y direction of the beam
                 const int order = 3                        ///< the order of spline (default=3,cubic)
  ) {
    beam_elems.clear();
    beam_nodes.clear();

//    // rotation of all nodes
//    ChMatrix33<> mrot;
//    mrot.Set_A_Xdir(ending_node - starting_node, Ydir);

    Position p0 = starting_node.GetPositionInWorld(NWU);
    Position p1 = ending_node.GetPositionInWorld(NWU);

    FrRotation rot;
    rot.SetXaxis(p1 - p0, {0., 0., 1.}, NWU);


    int p = order;

    // Create the 'complete' knot vector, with multiple at the ends
    ChVectorDynamic<> myknots(N + p + p + 1);
    geometry::ChBasisToolsBspline::ComputeKnotUniformMultipleEnds(myknots, p, 0.0, 1.0);

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < N + p; ++i_node) {
      double abscyssa = ((double) i_node / (double) (N + p - 1));

      // position of node
//      ChVector<> pos = starting_node + (ending_node - starting_node) * abscyssa;

      Position pos = p0 + (p1 - p0) * abscyssa;

      FrFrame node_frame(pos, rot, NWU);
//      node_frame.Set


      auto hnode_i = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(internal::FrFrame2ChFrame(node_frame));
      mesh->AddNode(hnode_i);
      mynodes.push_back(hnode_i);
      this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
      std::vector<double> local_knots;
      for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
        local_knots.push_back(myknots(i_el + i_el_knot));
      }

      std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> element_nodes;
      for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
        element_nodes.push_back(mynodes[i_el + i_el_node]);
      }

      auto belement_i = std::make_shared<MyElementBeamIGA>();
      belement_i->SetNodesGenericOrder(element_nodes, local_knots, p);
      belement_i->SetSection(sect);
      mesh->AddElement(belement_i);
      this->beam_elems.push_back(belement_i);
    }
  }

  /// Helper function.
  /// Adds beam FEM elements to the mesh to create a spline beam
  /// using ChElementBeamIGA type elements, given a B-spline line in 3D space.
  /// Before running, each time resets lists of beam_elems and beam_nodes.
  void BuildBeam(std::shared_ptr <chrono::fea::ChMesh> mesh,                ///< mesh to store the resulting elements
                 std::shared_ptr <chrono::fea::ChBeamSectionCosserat> sect, ///< section material for beam elements
                 geometry::ChLineBspline &spline,             ///< the B-spline to be used as the centerline
                 const ChVector<> Ydir                        ///< the 'up' Y direction of the beam
  ) {
    beam_elems.clear();
    beam_nodes.clear();

    int p = spline.GetOrder();

    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = spline.Knots().GetRows() - p - p - 1;  // = n+p+1 -p-p-1 = n-p

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < spline.Points().size(); ++i_node) {
      double abscyssa = ((double) i_node / (double) (spline.Points().size() - 1));

      // position of node
      ChVector<> pos = spline.Points()[i_node];

      // rotation of node, x aligned to tangent at input spline
      ChMatrix33<> mrot;
      ChVector<> tangent;
      spline.Derive(tangent, abscyssa);
      mrot.Set_A_Xdir(tangent, Ydir);

      auto hnode_i = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
      mesh->AddNode(hnode_i);
      mynodes.push_back(hnode_i);
      this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
      std::vector<double> my_el_knots;
      for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
        my_el_knots.push_back(spline.Knots()(i_el + i_el_knot));
      }

      std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> my_el_nodes;
      for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
        my_el_nodes.push_back(mynodes[i_el + i_el_node]);
      }

      auto belement_i = std::make_shared<MyElementBeamIGA>();
      belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
      belement_i->SetSection(sect);
      mesh->AddElement(belement_i);
      this->beam_elems.push_back(belement_i);
    }
  }

  /// Access the list of elements used by the last built beam.
  /// It can be useful for changing properties afterwards.
  /// This list is reset all times a 'Build...' function is called.
  std::vector <std::shared_ptr<MyElementBeamIGA>> &GetLastBeamElements() { return beam_elems; }

  /// Access the list of nodes used by the last built beam.
  /// It can be useful for adding constraints or changing properties afterwards.
  /// This list is reset all times a 'Build...' function is called.
  std::vector <std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> &GetLastBeamNodes() { return beam_nodes; }
};


int main() {


// Create a Chrono::Engine physical system
  ChSystemNSC my_system;


  // Creating a load container to load the cable
  auto load_container = std::make_shared<chrono::ChLoadContainer>();
  my_system.Add(load_container);



  // Create a mesh, that is a container for groups
  // of elements and their referenced nodes.
  // Remember to add it to the system.
  auto my_mesh = std::make_shared<chrono::fea::ChMesh>();
//  my_mesh->SetAutomaticGravity(false);
  my_system.Add(my_mesh);

  // Create a section, i.e. thickness and material properties
  // for beams. This will be shared among some beams.

  // Real cable
  double diam_ = 0.168; // m
  double EA_ = 602.68e6; // N
  double lambda_ = 141; // kg/m
  double cable_length = 500; // m

  double A_ = M_PI * diam_ * diam_ / 4.; // m**2
  double E = EA_ / A_; // Pa
  double density = lambda_ / A_; // kg/m**3

  // Reduced cable
  double scale = 1. / 1.; // The scale at which we are testing the model
  double diameter = diam_ * scale;


  // Constitutive model
  auto melasticity = std::make_shared<chrono::fea::ChElasticityCosseratSimple>();
  melasticity->SetYoungModulus(E);
  melasticity->SetGshearModulus(E * 0.3); // Jouer avec...
  melasticity->SetBeamRaleyghDamping(1e3);

  // Section definition
  auto msection = std::make_shared<chrono::fea::ChBeamSectionCosserat>(melasticity);
  msection->SetDensity(density);
  msection->SetAsCircularSection(diameter);



  // Building the BSpline to initialize
  std::vector <ChVector<>> my_points;

  int nb_ctrl_points = 200;
  double dz = cable_length * scale / (double) (nb_ctrl_points - 1);
  double z = 0.;
  for (int i = 0; i < nb_ctrl_points; i++) {
    my_points.emplace_back(ChVector<>(0., 0., z));
    z += dz;
  }

  geometry::ChLineBspline my_spline(2,          // order (3 = cubic, etc)
                                    my_points); // control points, will become the IGA nodes


  MyFEACableBuilder builderR;
  builderR.BuildBeam(my_mesh,            // the mesh to put the elements in
                     msection,           // section of the beam
                     my_spline,          // Bspline to match (also order will be matched)
                     VECT_Y);            // suggested Y direction of section

  builderR.GetLastBeamNodes().front()->SetFixed(true);
//  builderR.GetLastBeamNodes().back()->SetFixed(true);



  // Adding loads on each element newly created by the builder
  for (auto &element : builderR.GetLastBeamElements()) {
    auto load = std::make_shared<MyLoad>(element);
    load_container->Add(load);
  }


//  auto mbodywing = std::make_shared<ChBodyEasyBox>(0.01, 0.2, 0.05, 2000);

//  mbodywing->SetCoord(builderR.GetLastBeamNodes().back()->GetCoord());
//  application.GetSystem()->Add(mbodywing);


// Rigidly attach body to cable
//  auto myjoint = std::make_shared<ChLinkMateFix>();
//  myjoint->Initialize(builderR.GetLastBeamNodes().back(), mbodywing);
//  application.GetSystem()->Add(myjoint);


  // Attach a visualization of the FEM mesh.

  auto mvisualizebeamA = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(my_mesh.get()));
  mvisualizebeamA->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_SURFACE);
  mvisualizebeamA->SetSmoothFaces(true);
  my_mesh->AddAsset(mvisualizebeamA);

  auto mvisualizebeamC = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(my_mesh.get()));
  mvisualizebeamC->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
  mvisualizebeamC->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
  mvisualizebeamC->SetSymbolsThickness(0.006);
  mvisualizebeamC->SetSymbolsScale(0.01);
  mvisualizebeamC->SetZbufferHide(false);
  my_mesh->AddAsset(mvisualizebeamC);






  // System setting and irrlicht settings



  // Create the Irrlicht visualization (open the Irrlicht device,
  // bind a simple user interface, etc. etc.)
  ChIrrApp application(&my_system, L"IGA beams DEMO (SPACE for dynamics, F10 / F11 statics)",
                       core::dimension2d<u32>(800, 600),
                       false, true);

  // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
  application.AddTypicalLogo();
  application.AddTypicalSky(std::string(RESOURCES_VIZU_PATH) + "skybox/");
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

  application.SetVideoframeSave(true);
  application.SetVideoframeSaveInterval(2);

  // Solver default settings for all the sub demos:
  my_system.SetSolverType(ChSolver::Type::MINRES);
  my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
  my_system.SetMaxItersSolverSpeed(500);
  my_system.SetMaxItersSolverStab(500);
  my_system.SetTolForce(1e-14);


  auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
  msolver->SetVerbose(false);
  msolver->SetDiagonalPreconditioning(true);

  application.SetTimestep(0.01);


  // Clear previous demo, if any:
//  application.GetSystem()->Clear();
  application.GetSystem()->SetChTime(0);




  // This is needed if you want to see things in Irrlicht 3D view.
  application.AssetBindAll();
  application.AssetUpdateAll();

  // Mark completion of system construction
  application.GetSystem()->SetupInitial();

  while (application.GetDevice()->run()) {
    application.BeginScene();
    application.DrawAll();
    application.DoStep();
    application.EndScene();
  }


  return 0.;
}
