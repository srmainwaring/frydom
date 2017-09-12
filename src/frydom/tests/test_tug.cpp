//
// Created by frongere on 07/09/2017.
//

#include <frydom/hydrodynamics/FrLinearDamping.h>
#include "frydom/core/FrCore.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/catenary/FrCatenaryLine.h"

#include "irrlicht.h"
#include "frydom/utils/FrIrrApp.h"

using namespace frydom;
using namespace environment;


int main(int argc, char* argv[]) {

    // The system
    FrOffshoreSystem system;

    // Set the free surface
    auto free_surface = std::make_unique<FrFlatFreeSurface>(0.);
    free_surface->Initialize(-200, 200, 50, -200, 200, 50);
    system.setFreeSurface(free_surface.release());


    // The current
    auto current_field = std::make_unique<FrCurrent>(WEST, 40, KNOT, NED, GOTO);
    // TODO: changer pour faire des move plutot que des release...
    system.SetCurrent(current_field.release());

    // Building a TUG
    auto tug = std::make_shared<FrShip>();
    system.AddBody(tug);
    tug->SetName("MyTug");
    tug->SetHydroMesh("../data/ship/MagneViking.obj", true);
    tug->SetMass(5e7);  // TODO: plutot dans les 5e9...
    tug->SetInertiaXX(chrono::ChVector<>(1e8, 1e9, 1e9));

    tug->SetTransverseUnderwaterArea(120.);
    tug->SetLateralUnderwaterArea(500.);
    tug->SetLpp(76.2);

    tug->SetPos(chrono::ChVector<>(-140, 0, 0));
    tug->Set3DOF_ON();

    tug->SetPos_dt(chrono::ChVector<>(5.*M_KNOT, 0, 0));

    // Adding a curent resistance
    std::string filename("../src/frydom/tests/data/PolarCurrentCoeffs.yml");
    auto current_force = std::make_shared<FrCurrentForce>(filename);
    tug->AddForce(current_force);

    // Adding a linear damping
    auto lin_damping_force = std::make_shared<FrLinearDamping>(1e7, 1e7, 1e8);
    tug->AddForce(lin_damping_force);


    // Creating a fairlead on the tug
    auto fairlead = tug->CreateNode(chrono::ChVector<>(40, 0.0, 0));
    fairlead->SetName("MyFairlead");

    // Creating an other node as an anchor
    auto anchor = std::make_shared<FrNode>();
    anchor->SetBody(system.GetWorldBodyPtr());  // Faire une classe anchor qui sait le faire toute seule

    auto anchor_pos = chrono::ChCoordsys<double>();
    anchor_pos.pos.x() = 0;

    // TODO: imposer un mouvement de l'ancre avec une fonction pour emuler un remorquage a vitesse constante
    anchor->Impose_Abs_Coord(anchor_pos);



    // Creating a catenary line
    double Lu = 120;
    bool elastic = true;
    auto u = chrono::ChVector<double>(0, 0, -1);
//    double q = 616.538;
//    double q = 2000;
    double q = 1000;
//    double q = 100;
//    double EA = 1e10;
//    double EA = 1.5708e9;
    double EA = 1e10;
    auto line = FrCatenaryLine(fairlead, anchor, elastic, EA, Lu, q, u);




    bool viz = true;
//float friction = 0.6f;
    double step_size = 1e-1;
    bool capture_video = false;

    // TODO: mettre la boucle suivante dans une fonction plutot que de recopier a chaque fois...
    // Il faut:
    // system
    // step_size
    // TryRealTime
    // Des trucs a sortir a chaque pas de temps
    if (viz) {

        // Using own class for irrlicht viz
        frydom::FrIrrApp app(&system, L"Frydom vizualization based on Irrlicht");
        app.AddTypicalLights();
        app.AddTypicalCamera(irr::core::vector3df(0, 0, 300), irr::core::vector3df(1, 0, -1));

        // Adding the FRyDoM logo
        auto device = app.GetDevice();
        app.AddTypicalLogo("../src/frydom/tests/data/frydom_logo.png");

        app.AssetBindAll();
        app.AssetUpdateAll();

//        app.SetStepManage(true);
        app.SetTimestep(step_size);
        app.SetTryRealtime(true);

        app.SetVideoframeSave(capture_video);

        auto tug_force = tug->Get_Xforce();

        while (app.GetDevice()->run()) {
            app.BeginScene();
            app.DrawAll();
            app.DoStep();
            app.EndScene();

            tug_force = tug->Get_Xforce();
            std::cout << tug_force[0] << "\t" << tug_force[1] << "\t" << tug_force[2] << std::endl;
        }
    }

    return 0;

}