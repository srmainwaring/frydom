//
// Created by frongere on 22/09/17.
//



//#include <chrono_fea/ChMesh.h>
//#include <FEAcables.h>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "FEAcables.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystemNSC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Cables FEM", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo("../src/frydom/tests/data/frydom_logo.png");
    application.AddTypicalSky();
    application.AddTypicalLights();
//    application.SetVideoframeSave(true);
//    application.AddTypicalCamera(core::vector3df(0.f, 0.f, 5.f));
    application.AddTypicalCamera(core::vector3df(1.f, 1.f, -1.f));

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create one of the available models (defined in FEAcables.h)
    // model1(my_system, my_mesh);
     model2(my_system, my_mesh);
//    model3(my_system, my_mesh);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colours as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a coloured ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS); // E_GLYPH_NODE_CSYS
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!
    application.AssetUpdateAll();

    // Mark completion of system construction
    my_system.SetupInitial();

    // Change solver settings
    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(400);
    my_system.SetMaxItersSolverStab(200);
    my_system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(true);
    msolver->SetDiagonalPreconditioning(true);

    // Change type of integrator:
    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    // my_system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxiters(2);
        mystepper->SetAbsTolerances(1e-6);
    }

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}