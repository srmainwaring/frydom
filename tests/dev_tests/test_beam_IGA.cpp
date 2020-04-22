//
// Created by frongere on 22/04/2020.
//

//#include <chrono/>


#include "chrono/physics/ChSystemNSC.h"

#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono/fea/ChBeamSectionCosserat.h"
#include "chrono/fea/ChBuilderBeam.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;


int main() {


// Create a Chrono::Engine physical system
  ChSystemNSC my_system;



  // Create the Irrlicht visualization (open the Irrlicht device,
  // bind a simple user interface, etc. etc.)
  ChIrrApp application(&my_system, L"IGA beams DEMO (SPACE for dynamics, F10 / F11 statics)", core::dimension2d<u32>(800, 600),
                       false, true);

  // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
  application.AddTypicalLogo();
  application.AddTypicalSky(std::string(RESOURCES_VIZU_PATH) + "skybox/");
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

//  // This is for GUI tweaking of system parameters..
//  MyEventReceiver receiver(&application);
//  // note how to add a custom event receiver to the default interface:
//  application.SetUserEventReceiver(&receiver);

//  // Some help on the screen
//  auto gad_textFPS = application.GetIGUIEnvironment()->addStaticText(L" Press 1: static analysis \n Press 2: curved beam connected to body \n Press 3: plasticity \n Press 4: Jeffcott rotor", irr::core::rect<irr::s32>(10, 80, 250, 150), false, true, 0);


  // Solver default settings for all the sub demos:
  my_system.SetSolverType(ChSolver::Type::MINRES);
  my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
  my_system.SetMaxItersSolverSpeed(500);
  my_system.SetMaxItersSolverStab(500);
  my_system.SetTolForce(1e-14);


  auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
  msolver->SetVerbose(false);
  msolver->SetDiagonalPreconditioning(true);

//  #ifdef USE_MKL
//  auto mkl_solver = std::make_shared<ChSolverMKL<>>();
//        my_system.SetSolver(mkl_solver);
//  #endif

  application.SetTimestep(0.01);


  // Clear previous demo, if any:
  application.GetSystem()->Clear();
  application.GetSystem()->SetChTime(0);




  // Create a mesh, that is a container for groups
  // of elements and their referenced nodes.
  // Remember to add it to the system.
  auto my_mesh = std::make_shared<ChMesh>();
  my_mesh->SetAutomaticGravity(false);
  application.GetSystem()->Add(my_mesh);

  // Create a section, i.e. thickness and material properties
  // for beams. This will be shared among some beams.

  double beam_wy = 0.012;
  double beam_wz = 0.025;

  auto melasticity = std::make_shared<ChElasticityCosseratSimple>();
  melasticity->SetYoungModulus(0.02e10);
  melasticity->SetGshearModulus(0.02e10 * 0.3);
  melasticity->SetBeamRaleyghDamping(0.0000);
  auto msection = std::make_shared<ChBeamSectionCosserat>(melasticity);
  msection->SetDensity(1000);
  msection->SetAsRectangularSection(beam_wy, beam_wz);


  ChBuilderBeamIGA builderR;

  std::vector< ChVector<> > my_points = { {0,0,0.2}, {0,0,0.3}, { 0,-0.01,0.4 } , {0,-0.04,0.5}, {0,-0.1,0.6} };

  geometry::ChLineBspline my_spline(  3,          // order (3 = cubic, etc)
                                      my_points); // control points, will become the IGA nodes

  builderR.BuildBeam(      my_mesh,            // the mesh to put the elements in
                           msection,           // section of the beam
                           my_spline,          // Bspline to match (also order will be matched)
                           VECT_Y);            // suggested Y direction of section

  builderR.GetLastBeamNodes().front()->SetFixed(true);

  auto mbodywing = std::make_shared<ChBodyEasyBox>(0.01,0.2,0.05,2000);
  mbodywing->SetCoord(builderR.GetLastBeamNodes().back()->GetCoord());
  application.GetSystem()->Add(mbodywing);

  auto myjoint = std::make_shared<ChLinkMateFix>();
  myjoint->Initialize(builderR.GetLastBeamNodes().back(), mbodywing);
  application.GetSystem()->Add(myjoint);


  // Attach a visualization of the FEM mesh.

  auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
  mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
  mvisualizebeamA->SetSmoothFaces(true);
  my_mesh->AddAsset(mvisualizebeamA);

  auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
  mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
  mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
  mvisualizebeamC->SetSymbolsThickness(0.006);
  mvisualizebeamC->SetSymbolsScale(0.01);
  mvisualizebeamC->SetZbufferHide(false);
  my_mesh->AddAsset(mvisualizebeamC);

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
