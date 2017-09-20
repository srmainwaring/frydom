//
// Created by frongere on 13/07/17.
//

#include <frydom/utils/FrIrrApp.h>
#include "frydom/core/FrCore.h"
#include "frydom/environment/FrEnvironment.h"

using namespace chrono;
using namespace frydom;
using namespace environment;

int main(int argc, char* argv[]) {

    // ====================================================================================
    // Defining the system
    // ====================================================================================
    FrOffshoreSystem system;


    // TODO: pour l'environnement, un environnement par defaut devrait etre cree, quitte a cree des classes derivees de
    // OffshoreSystem... -> on devrait pouvoir creer un systeme en une ligne et ensuite tuner les parametres (ou le faire
    // via un constructeur avec plein d'arguments par defaut ?).
    // On peut avoir une methode de systeme du type SetFreeSurfaceModel qui connait une factory de free surface ?
    // NON, ce sera au Python de savoir faire tout ça !!!

    // ====================================================================================
    // Defining the current
    // ====================================================================================
    // TODO: avoir une factory de courant. Un courant nul par defaut devrait etre cree dans offshoreSystem
    auto current_field = std::make_unique<FrCurrent>(NORTH, 8, KNOT, NED, GOTO);

    // TODO: changer pour faire des move !!
    system.SetCurrent(current_field.release());

    // ====================================================================================
    // Defining the free surface
    // ====================================================================================
    auto free_surface = std::make_unique<frydom::environment::FrFlatFreeSurface>(0.);
    free_surface->Initialize(-400, 400, 200, -100, 100, 100);
    system.setFreeSurface(free_surface.release());
    // TODO: une surface libre flat (200mx200m) par defaut devrait etre presente a l'instanciation de system

    // ====================================================================================
    // Building a ship
    // ====================================================================================
    auto ship = std::make_shared<FrShip>();  // TODO: avoir une factory make_ship.

    system.AddBody(ship);  // TODO: avoir une factory make_hydro_body(system) ... qui renvoie un shared_ptr du bateau cree mais qui gere les differents pointages
    ship->Set3DOF_ON(); // TODO: Attention, il faut que le corps soit deja renseigne dans system... permettre de differer ...


    // Ship propertie
    ship->SetName("Magne Viking");
    ship->SetMass(7.7e6);
    ship->SetInertiaXX(chrono::ChVector<>(1e8, 5e9, 5e9));

    ship->SetTransverseUnderwaterArea(120.);
    ship->SetLateralUnderwaterArea(500.);
    ship->SetLpp(76.2);

    // Ship initial position and orientation
    ship->SetPos(ChVector<>(0., 0., 0.));
    ship->SetNEDHeading(SOUTH_WEST);
//    ship->SetNEDHeading(180, DEG);

    // Ship initial velocity
    auto ship_velocity = ship->TransformDirectionLocalToParent(chrono::ChVector<>(KNOT2MS(25.), 0., 0.)); // SetLocalVelocity
    ship->SetPos_dt(ship_velocity);
    // TODO: ajouter une methode SetVelocity avec la possibilite de preciser des noeuds...

    // Set the mesh as a hydrodynamic mesh and an asset
    ship->SetHydroMesh("../data/ship/MagneViking.obj", true);

    // TODO: voir plus tard a ajouter les asset etc...

    // Building current force
    std::string filename("../src/frydom/tests/data/PolarCurrentCoeffs.yml");
    auto current_force = std::make_shared<FrCurrentForce>(filename);
    ship->AddForce(current_force);



    // Using own class for irrlicht viz
    frydom::FrIrrApp app(&system, L"Frydom vizualization based on Irrlicht");
    app.AddTypicalLights();
    app.AddTypicalCamera(irr::core::vector3df(0, 0, 300), irr::core::vector3df(1, 0, -1));

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Time stepping data
    app.SetTimestep(0.1);
    app.SetTryRealtime(true);

//    app.SetVideoframeSave(capture_video);

    while (app.GetDevice()->run()) {
        app.BeginScene();
        app.DrawAll();
        app.DoStep();
        app.EndScene();

    }


    return 0;
}