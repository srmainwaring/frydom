//
// Created by camille on 05/07/18.
//

#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "frydom/frydom.h"


using namespace chrono;
using namespace frydom;


int main(int argc, char* argv[]) {

    // --------------------------------------------------
    // System
    // --------------------------------------------------

    FrOffshoreSystem system;
    //ChSystemSMC system;

    // --------------------------------------------------
    // Solid model
    // --------------------------------------------------

    auto barge = std::make_shared<FrHydroBody>();
    barge->SetName("Barge");
    barge->SetHydroMesh("Barge2.obj", true);
    barge->SetPos(chrono::ChVector<double>(0., 0., 0.));
    //barge->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    system.AddBody(barge);
    barge->SetBodyFixed(true);

    auto base_crane = std::make_shared<FrBody>();
    base_crane->SetName("Base_crane");
    base_crane->SetVisuMesh("BaseCrane.obj");
    //base_crane->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    base_crane->SetPos(chrono::ChVector<double>(0., +7.5, 3.15));
    //base_crane->SetPos(chrono::ChVector<double>(1., -3., 7.5));
    //base_crane->SetPos(chrono::ChVector<double>(-7.5, 0., 3.));
    system.AddBody(base_crane);
    //base_crane->SetBodyFixed(true);

    auto tige_crane = std::make_shared<FrBody>();
    tige_crane->SetName("Tige_crane");
    tige_crane->SetVisuMesh("TigeCrane.obj");
    //tige_crane->SetCOG(chrono::ChVector<double>(0., 0., 0.));
    tige_crane->SetPos(chrono::ChVector<double>(0., +5.5, 4.5));
    tige_crane->SetRot(Q_from_AngAxis(-CH_C_PI_4,VECT_X));
    system.AddBody(tige_crane);
    //tige_crane->SetBodyFixed(true);

    auto hub_box = std::make_shared<FrBody>();
    hub_box->SetName("HubBox");
    hub_box->SetVisuMesh("HubBox.obj");
    //hub_box->SetCOG(chrono::ChVector<double>(0., 0., 0.));
    hub_box->SetPos(chrono::ChVector<double>(0,-10,4.1));
    system.AddBody(hub_box);
    //hub_box->SetBodyFixed(true);

    // ---------------------------------------------
    // Markers
    // ---------------------------------------------

    //auto A1_barge = barge->CreateNode(ChVector<double>(0., 7.5, 3.));
    //auto A1_crane = base_crane->CreateNode(ChVector<double>(0., 0., 0.));
    //auto A2_crane = base_crane->CreateNode(ChVector<double>(2., 0., 1.5));
    //auto A2_tige = tige_crane->CreateNode(ChVector<double>(0., 0., 0.));
    auto A3_tige = tige_crane->CreateNode(ChVector<double>(0., -19., 0.));
    auto A4_hub = hub_box->CreateNode(ChVector<double>(0., 0., 0.));



    // Line properties
    double Lu = 14.25;
    auto u = chrono::ChVector<double>(0, 0, 1);
    double q = 1;//616.538;
    double EA = 1.5708e9;
    double A = 0.003;
    double E = EA/A;

    // ---------------------------------------------
    // Catenary Line
    // ---------------------------------------------

    auto Catenary = std::make_shared<FrCatenaryLine>(A3_tige, A4_hub, false, E, A, Lu, q, u);

    /*
    auto A1_barge = barge->CreateNode(ChVector<double>(0., 0., -5.));
    auto A1_crane = base_crane->CreateNode(ChVector<double>(0., 0., -5.));
    auto A2_crane = base_crane->CreateNode(ChVector<double>(0., 0., 0.));
    auto A2_tige = tige_crane->CreateNode(ChVector<double>(0., 0., 0.));
    auto A3_tige = tige_crane->CreateNode(ChVector<double>(0., 0., 0.));
    auto A4_hub = hub_box->CreateNode(ChVector<double>(0., 0., 0.));
    */
    // -----------------------------------------------
    // Link engine
    // -----------------------------------------------
/*
    auto rot_funct = std::make_shared<ChFunction_Const>();
    rot_funct->Set_yconst(0.3);

    auto motor_crane = std::make_shared<ChLinkEngine>();
    motor_crane->Initialize(A1_barge, A1_crane);
    //motor_crane->Initialize(A1_barge, A1_crane);
    motor_crane->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    motor_crane->Set_spe_funct(rot_funct);
    system.Add(motor_crane);
*/
 /*
    auto motor_tige = std::make_shared<ChLinkEngine>();
    motor_tige->Initialize(A2_crane, A2_tige);
    motor_tige->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    system.Add(motor_tige);
*/

    // ----------------------------------------------
    // Motors
    // ----------------------------------------------

    auto rotmotor_crane = std::make_shared<ChLinkMotorRotationAngle>();
    rotmotor_crane->Initialize(base_crane, barge, ChFrame<>(ChVector<>(0., 7.5, 3.))); //, CH_C_PI_2, VECT_Z
    //rotmotor->Initialize(base_crane, barge, true, ChVector<>(0, 0, 0), ChVector<>(0, 7.5, 3),ChVector<>(1, 0, 0), ChVector<>(1, 0, 0));
    system.Add(rotmotor_crane);

    auto rotmotor_tige = std::make_shared<ChLinkMotorRotationAngle>();
    rotmotor_tige->Initialize(tige_crane, base_crane, ChFrame<>(ChVector<>(0., 5.5 , 4.5), CH_C_PI_2, VECT_Y));
    //rotmotor_tige->Initialize(tige_crane, base_crane, true, ChVector<>(0, 0, 0), ChVector<>(0, 7.5, 1.5),ChVector<>(1, 0, 0), ChVector<>(1, 0, 0));
    system.Add(rotmotor_tige);

    auto rwspeed = std::make_shared<ChFunction_Sine>(
            0,      // phase [rad]
            0.05,   // frequency [Hz]
            CH_C_PI_4 // amplitude [rad]
    );
    rotmotor_crane->SetAngleFunction(rwspeed);
    auto rwangle_tige = std::make_shared<ChFunction_Const>(0);//-CH_C_PI_4
    rotmotor_tige->SetAngleFunction(rwangle_tige);


    // -----------------------------------------------
    // Simulation
    // -----------------------------------------------

    double dt = 0.005;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);
    system.Initialize();

    auto app = FrIrrApp(system);
    app.AddTypicalCamera(irr::core::vector3df(100, 0, 20), irr::core::vector3df(0, 0, 3));
    app.Run();

    /*// -----------------------------------------------
    // Simulation
    // -----------------------------------------------

    double dt = 0.005;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);

    auto app = chrono::irrlicht::ChIrrApp(&system,L"FRyDoM viewer",irr::core::dimension2d<irr::u32>(800, 600),
                               false,false,true,irr::video::EDT_OPENGL);


    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    app.AddTypicalLogo();
    app.AddTypicalSky();
    app.AddTypicalLights();
    app.AddLightWithShadow(vector3df(1.0f, 35.0f, -5.0f), vector3df(0, 0, 0), 45, 0.2, 45, 35, 512,
                                   video::SColorf(0.6f, 0.8f, 1.0f));
    app.AddTypicalCamera(irr::core::vector3df(1, 1, -30),irr::core::vector3df(0, 0, -1));

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    app.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    app.AssetUpdateAll();

    // This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
    // for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)
    app.AddShadowAll();

    app.SetTimestep(0.005);
    app.SetTryRealtime(true);


    while (app.GetDevice()->run()) {
        app.BeginScene(true, true, SColor(255, 140, 161, 192));

        app.DrawAll();

        app.DoStep();

        app.EndScene();
    }*/


}